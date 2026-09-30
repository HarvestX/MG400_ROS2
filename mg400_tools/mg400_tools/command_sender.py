#!/usr/bin/env python3
"""Replay MG400 commands extracted from log files."""
# Copyright 2026 HarvestX Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
import re
import socket
import time
from pathlib import Path


MOTION_PREFIXES = (
    "MovJ(",
    "MovL(",
    "JointMovJ(",
    "MovJIO(",
    "MovLIO(",
)

LOG_LINE_PATTERN = re.compile(r"send\s*:\s*(.+)$")
RAW_COMMAND_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\(.*\)$")
ROBOT_MODE_RESPONSE_PATTERN = re.compile(r"^[^,]+,\{(-?\d+)\},RobotMode\(\);$")

ROBOT_MODE_ENABLE = 5
ROBOT_MODE_RUNNING = 7
ROBOT_MODE_ERROR = 9


def robot_mode_name(mode: int) -> str:
    """Convert robot mode integer to a readable name."""
    names = {
        1: "INIT",
        2: "BRAKE_OPEN",
        4: "DISABLED",
        5: "ENABLE",
        6: "BACKDRIVE",
        7: "RUNNING",
        8: "RECORDING",
        9: "ERROR",
        10: "PAUSE",
        11: "JOG",
        12: "INVALID",
    }
    return names.get(mode, f"UNKNOWN({mode})")


def make_parser() -> argparse.ArgumentParser:
    """Create command line parser."""
    parser = argparse.ArgumentParser(
        description="Send MG400 commands from a text file."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("error_command.txt"),
        help="Command log file path.",
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="192.168.1.6",
        help="MG400 IP address.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="How many times to replay the selected command sequence.",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=1,
        help="1-based start index in extracted commands.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Max number of commands to send (0 means all after --start).",
    )
    parser.add_argument(
        "--dashboard-port",
        type=int,
        default=29999,
        help="Dashboard TCP port.",
    )
    parser.add_argument(
        "--motion-port",
        type=int,
        default=30003,
        help="Motion TCP port.",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=3.0,
        help="TCP connect timeout seconds.",
    )
    parser.add_argument(
        "--connect-retries",
        type=int,
        default=5,
        help="Connection retry count per port when connection is refused.",
    )
    parser.add_argument(
        "--connect-retry-interval",
        type=float,
        default=1.0,
        help="Seconds between connection retries.",
    )
    parser.add_argument(
        "--motion-send-retries",
        type=int,
        default=3,
        help="Retry count when motion send fails due to socket reset/broken pipe.",
    )
    parser.add_argument(
        "--response-timeout",
        type=float,
        default=3.0,
        help="Seconds to wait for each command response.",
    )
    parser.add_argument(
        "--motion-enable-sync-every",
        type=int,
        default=40,
        help="After every N motion commands, wait until RobotMode is ENABLE (0 to disable).",
    )
    parser.add_argument(
        "--wait-enable-timeout",
        type=float,
        default=120.0,
        help="Max seconds to wait for RobotMode=ENABLE after each motion batch.",
    )
    parser.add_argument(
        "--wait-enable-poll-interval",
        type=float,
        default=0.2,
        help="Polling interval seconds while waiting for RobotMode=ENABLE.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse and print summary only. No network access.",
    )
    return parser


def extract_commands(path: Path) -> list[str]:
    """Extract commands from a log file."""
    if not path.is_file():
        raise FileNotFoundError(f"Input file not found: {path}")

    commands: list[str] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue

        match = LOG_LINE_PATTERN.search(line)
        if match:
            command = match.group(1).strip()
            if command:
                commands.append(command)
            continue

        if RAW_COMMAND_PATTERN.match(line):
            commands.append(line)

    return commands


def is_motion_command(command: str) -> bool:
    """Check command should be sent to motion port."""
    return command.startswith(MOTION_PREFIXES)


def select_commands(commands: list[str], start: int, limit: int) -> list[str]:
    """Select a subset of commands with 1-based start and optional limit."""
    if start < 1:
        raise ValueError("--start must be >= 1")
    if limit < 0:
        raise ValueError("--limit must be >= 0")

    start_idx = start - 1
    if start_idx >= len(commands):
        return []

    if limit == 0:
        return commands[start_idx:]
    return commands[start_idx : start_idx + limit]


def open_socket(ip: str, port: int, timeout: float) -> socket.socket:
    """Open TCP socket."""
    sock = socket.create_connection((ip, port), timeout=timeout)
    sock.settimeout(timeout)
    return sock


def recv_tcp_response(
    sock: socket.socket,
    timeout: float,
    pending: bytearray,
) -> str:
    """Receive one ';'-terminated TCP response."""
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        sep = pending.find(b";")
        if sep != -1:
            one = bytes(pending[: sep + 1])
            del pending[: sep + 1]
            return one.decode("utf-8", errors="replace").strip()

        remaining = max(0.0, deadline - time.monotonic())
        sock.settimeout(remaining)
        try:
            chunk = sock.recv(256)
        except socket.timeout:
            break
        if not chunk:
            break
        pending.extend(chunk)

    return ""


def send_dashboard_command_and_recv(
    dashboard_sock: socket.socket | None,
    pending: bytearray,
    command: str,
    timeout: float,
) -> str:
    """Send one dashboard command and receive one response."""
    if dashboard_sock is None:
        return ""
    dashboard_sock.sendall(command.encode("utf-8"))
    return recv_tcp_response(dashboard_sock, timeout, pending)


def parse_robot_mode(response: str) -> int | None:
    """Parse RobotMode() response and return mode integer."""
    compact = re.sub(r"\s+", "", response)
    match = ROBOT_MODE_RESPONSE_PATTERN.match(compact)
    if not match:
        return None
    return int(match.group(1))


def wait_until_enable_mode(
    dashboard_sock: socket.socket | None,
    dashboard_pending: bytearray,
    response_timeout: float,
    wait_timeout: float,
    poll_interval: float,
    require_running_before_enable: bool,
) -> bool:
    """Wait until robot mode becomes ENABLE, optionally after seeing RUNNING once."""
    if dashboard_sock is None:
        return False

    deadline = time.monotonic() + wait_timeout
    last_mode: int | None = None
    running_seen = not require_running_before_enable
    while time.monotonic() < deadline:
        response = send_dashboard_command_and_recv(
            dashboard_sock,
            dashboard_pending,
            "RobotMode()",
            response_timeout,
        )
        print(f"           WAIT RECV {response if response else '(no response)'}")
        if response:
            mode = parse_robot_mode(response)
            if mode is not None:
                last_mode = mode
                print(
                    "           WAIT MODE "
                    f"{mode} ({robot_mode_name(mode)})"
                )
                if mode == ROBOT_MODE_ERROR:
                    return False

                if not running_seen:
                    if mode == ROBOT_MODE_RUNNING:
                        running_seen = True
                        print("           WAIT STATE RUNNING observed; now waiting for ENABLE")
                    else:
                        if mode == ROBOT_MODE_ENABLE:
                            print("           WAIT STATE still waiting for first RUNNING")
                        continue

                if mode == ROBOT_MODE_ENABLE:
                    return True

        if poll_interval > 0:
            time.sleep(poll_interval)

    print(
        "           WAIT timeout while waiting for ENABLE "
        f"(last_mode={last_mode if last_mode is not None else 'unknown'}"
        f" {f'({robot_mode_name(last_mode)})' if last_mode is not None else ''}, "
        f"running_seen={running_seen})"
    )
    return False


def open_socket_with_retry(
    ip: str,
    port: int,
    timeout: float,
    retries: int,
    retry_interval: float,
) -> socket.socket:
    """Open TCP socket with retry and clearer diagnostics."""
    if retries < 1:
        raise ValueError("--connect-retries must be >= 1")
    if retry_interval < 0:
        raise ValueError("--connect-retry-interval must be >= 0")

    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            return open_socket(ip, port, timeout)
        except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
            last_error = exc
            print(f"Connect failed {ip}:{port} (attempt {attempt}/{retries}): {exc}")
            if attempt < retries and retry_interval > 0:
                time.sleep(retry_interval)

    raise RuntimeError(
        f"Failed to connect to {ip}:{port} after {retries} attempts. "
        "Check power/cable/IP, and ensure no other client (e.g. mg400_node) "
        "is exclusively holding the MG400 TCP connection."
    ) from last_error


def reconnect_motion_socket(
    current_sock: socket.socket | None,
    ip: str,
    port: int,
    connect_timeout: float,
    connect_retries: int,
    connect_retry_interval: float,
) -> socket.socket:
    """Reconnect motion socket safely."""
    if current_sock is not None:
        try:
            current_sock.close()
        except OSError:
            pass
    return open_socket_with_retry(
        ip,
        port,
        connect_timeout,
        connect_retries,
        connect_retry_interval,
    )


def validate_args(args: argparse.Namespace) -> None:
    """Validate CLI arguments."""
    if args.repeat < 1:
        raise ValueError("--repeat must be >= 1")
    if args.motion_send_retries < 1:
        raise ValueError("--motion-send-retries must be >= 1")
    if args.response_timeout <= 0:
        raise ValueError("--response-timeout must be > 0")
    if args.motion_enable_sync_every < 0:
        raise ValueError("--motion-enable-sync-every must be >= 0")
    if args.wait_enable_timeout <= 0:
        raise ValueError("--wait-enable-timeout must be > 0")
    if args.wait_enable_poll_interval < 0:
        raise ValueError("--wait-enable-poll-interval must be >= 0")


def print_error_id_diag(
    dashboard_sock: socket.socket | None,
    dashboard_pending: bytearray,
    response_timeout: float,
) -> None:
    """Print GetErrorID diagnostic."""
    try:
        err_resp = send_dashboard_command_and_recv(
            dashboard_sock,
            dashboard_pending,
            "GetErrorID()",
            response_timeout,
        )
        print(f"           DIAG GetErrorID={err_resp or '(no response)'}")
    except OSError as exc:
        print(f"           DIAG failed to query GetErrorID: {exc}")


def print_robot_mode_and_error_id_diag(
    dashboard_sock: socket.socket | None,
    dashboard_pending: bytearray,
    response_timeout: float,
) -> None:
    """Print RobotMode/GetErrorID diagnostic."""
    try:
        mode_resp = send_dashboard_command_and_recv(
            dashboard_sock,
            dashboard_pending,
            "RobotMode()",
            response_timeout,
        )
        err_resp = send_dashboard_command_and_recv(
            dashboard_sock,
            dashboard_pending,
            "GetErrorID()",
            response_timeout,
        )
        print(
            "           DIAG dashboard "
            f"RobotMode={mode_resp or '(no response)'} "
            f"GetErrorID={err_resp or '(no response)'}"
        )
    except OSError as exc:
        print(f"           DIAG failed to query dashboard: {exc}")


def wait_motion_batch_enable(
    dashboard_sock: socket.socket | None,
    dashboard_pending: bytearray,
    response_timeout: float,
    wait_enable_timeout: float,
    wait_enable_poll_interval: float,
    batch_label: str,
) -> bool:
    """Wait for RUNNING->ENABLE and print diagnostics on failure."""
    print(
        "           WAIT start: checking RobotMode() until "
        f"RUNNING->ENABLE ({batch_label})"
    )
    ok = wait_until_enable_mode(
        dashboard_sock,
        dashboard_pending,
        response_timeout,
        wait_enable_timeout,
        wait_enable_poll_interval,
        True,
    )
    if not ok:
        print_error_id_diag(dashboard_sock, dashboard_pending, response_timeout)
    return ok


def send_motion_with_retry(
    command: str,
    motion_sock: socket.socket | None,
    dashboard_sock: socket.socket | None,
    dashboard_pending: bytearray,
    args: argparse.Namespace,
) -> tuple[socket.socket | None, bool]:
    """Send one motion command with reconnect retries."""
    if motion_sock is None:
        raise RuntimeError("Motion socket is not open")

    for send_attempt in range(1, args.motion_send_retries + 1):
        try:
            motion_sock.sendall(command.encode("utf-8"))
            return motion_sock, True
        except (ConnectionResetError, BrokenPipeError, OSError) as exc:
            print(
                "           WARN motion send failed "
                f"(attempt {send_attempt}/{args.motion_send_retries}): {exc}"
            )
            if send_attempt >= args.motion_send_retries:
                break
            try:
                motion_sock = reconnect_motion_socket(
                    motion_sock,
                    args.ip,
                    args.motion_port,
                    args.connect_timeout,
                    args.connect_retries,
                    args.connect_retry_interval,
                )
            except RuntimeError as reconnect_exc:
                print(f"           ERROR motion reconnect failed: {reconnect_exc}")
                print_robot_mode_and_error_id_diag(
                    dashboard_sock,
                    dashboard_pending,
                    args.response_timeout,
                )
                return motion_sock, False
            print(
                "           INFO motion socket reconnected: "
                f"{args.ip}:{args.motion_port}"
            )

    return motion_sock, False


def main() -> int:
    """Run replay."""
    args = make_parser().parse_args()
    validate_args(args)

    commands = extract_commands(args.input)
    selected = select_commands(commands, args.start, args.limit)
    if not selected:
        print("No commands selected. Check --start/--limit or input file content.")
        return 1

    dashboard_count = sum(1 for cmd in selected if not is_motion_command(cmd))
    motion_count = len(selected) - dashboard_count

    print(
        f"Loaded {len(commands)} commands, selected {len(selected)} "
        f"(dashboard={dashboard_count}, motion={motion_count})."
    )
    if args.motion_enable_sync_every > 0:
        print(
            "Motion sync: enabled "
            f"(every={args.motion_enable_sync_every}, "
            f"wait_timeout={args.wait_enable_timeout}s, "
            f"poll={args.wait_enable_poll_interval}s)"
        )
    else:
        print("Motion sync: disabled (set --motion-enable-sync-every > 0 to enable RobotMode logs)")
    if args.dry_run:
        print("Dry run mode: no command sent.")
        return 0

    dashboard_sock = None
    motion_sock = None
    dashboard_pending = bytearray()
    total_dashboard_sent = 0
    total_motion_sent = 0
    global_sent_index = 0

    try:
        if dashboard_count > 0:
            dashboard_sock = open_socket_with_retry(
                args.ip,
                args.dashboard_port,
                args.connect_timeout,
                args.connect_retries,
                args.connect_retry_interval,
            )
            print(f"Connected dashboard: {args.ip}:{args.dashboard_port}")
        if motion_count > 0:
            motion_sock = open_socket_with_retry(
                args.ip,
                args.motion_port,
                args.connect_timeout,
                args.connect_retries,
                args.connect_retry_interval,
            )
            print(f"Connected motion:    {args.ip}:{args.motion_port}")

        for replay_idx in range(1, args.repeat + 1):
            print(f"Replay {replay_idx}/{args.repeat} ...")
            motion_batch_count = 0
            for cmd_idx, command in enumerate(selected, start=1):
                motion_cmd = is_motion_command(command)
                global_sent_index += 1
                channel = "motion" if motion_cmd else "dashboard"
                print(
                    f"  [{global_sent_index:04d}] "
                    f"(replay={replay_idx}, idx={cmd_idx}, ch={channel}) "
                    f"SEND {command}"
                )
                if motion_cmd:
                    motion_sock, sent_ok = send_motion_with_retry(
                        command,
                        motion_sock,
                        dashboard_sock,
                        dashboard_pending,
                        args,
                    )
                    if not sent_ok:
                        print("Motion send failed after retries. Aborting.")
                        return 2
                    total_motion_sent += 1
                    motion_batch_count += 1
                    if (
                        args.motion_enable_sync_every > 0
                        and motion_batch_count >= args.motion_enable_sync_every
                    ):
                        ok = wait_motion_batch_enable(
                            dashboard_sock,
                            dashboard_pending,
                            args.response_timeout,
                            args.wait_enable_timeout,
                            args.wait_enable_poll_interval,
                            f"batch={motion_batch_count}",
                        )
                        if not ok:
                            return 2
                        motion_batch_count = 0
                    continue

                if dashboard_sock is None:
                    raise RuntimeError("Dashboard socket is not open")
                response = send_dashboard_command_and_recv(
                    dashboard_sock,
                    dashboard_pending,
                    command,
                    args.response_timeout,
                )
                total_dashboard_sent += 1
                print(f"           RECV {response if response else '(no response)'}")
                if not response:
                    print("No dashboard response. Aborting.")
                    return 2

            # Always synchronize the final partial motion batch in this replay.
            if args.motion_enable_sync_every > 0 and motion_batch_count > 0:
                ok = wait_motion_batch_enable(
                    dashboard_sock,
                    dashboard_pending,
                    args.response_timeout,
                    args.wait_enable_timeout,
                    args.wait_enable_poll_interval,
                    f"final batch={motion_batch_count}",
                )
                if not ok:
                    return 2

        print(
            "Completed. "
            f"sent_dashboard={total_dashboard_sent}, "
            f"sent_motion={total_motion_sent}, "
            f"total={total_dashboard_sent + total_motion_sent}"
        )
        return 0
    finally:
        if dashboard_sock is not None:
            dashboard_sock.close()
        if motion_sock is not None:
            motion_sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
