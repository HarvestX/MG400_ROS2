#!/usr/bin/env python3
"""Validate PositiveSolution/InverseSolution consistency on simple 4-axis cases."""
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
import math
import re
import socket
import time


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate PositiveSolution -> InverseSolution consistency using "
            "the confirmed Rx mapping. The script uses GetAngle() as the base "
            "state and tests 50 nearby cases."
        )
    )
    parser.add_argument("--ip", default="192.168.1.6", help="MG400 IP address.")
    parser.add_argument(
        "--dashboard-port", type=int, default=29999, help="Dashboard TCP port."
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=3.0, help="TCP connect timeout seconds."
    )
    parser.add_argument(
        "--response-timeout", type=float, default=3.0, help="TCP response timeout seconds."
    )
    parser.add_argument(
        "--connect-retries",
        type=int,
        default=5,
        help="Connection retry count when connection is refused.",
    )
    parser.add_argument(
        "--connect-retry-interval",
        type=float,
        default=1.0,
        help="Seconds between connection retries.",
    )
    parser.add_argument(
        "--pass-threshold-deg",
        type=float,
        default=0.01,
        help="Maximum allowed joint round-trip error in degrees for PASS.",
    )
    return parser


def open_socket_with_retry(
    ip: str,
    port: int,
    timeout: float,
    retries: int,
    retry_interval: float,
) -> socket.socket:
    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            sock = socket.create_connection((ip, port), timeout=timeout)
            sock.settimeout(timeout)
            return sock
        except (ConnectionRefusedError, TimeoutError, socket.timeout, OSError) as exc:
            last_error = exc
            print(f"Connect failed {ip}:{port} (attempt {attempt}/{retries}): {exc}")
            if attempt < retries and retry_interval > 0:
                time.sleep(retry_interval)

    raise RuntimeError(
        f"Failed to connect to {ip}:{port} after {retries} attempts."
    ) from last_error


def recv_tcp_response(sock: socket.socket, timeout: float, pending: bytearray) -> str:
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


def send_and_recv(
    sock: socket.socket,
    pending: bytearray,
    command: str,
    timeout: float,
) -> str:
    sock.sendall(command.encode("utf-8"))
    return recv_tcp_response(sock, timeout, pending)


def parse_dashboard_response(packet: str) -> tuple[int, str, str]:
    compact = re.sub(r"[ \t\r\n]+", "", packet)
    first_comma = compact.find(",")
    if first_comma == -1:
        raise ValueError(f"Malformed response: {packet}")

    error_id = int(compact[:first_comma])
    rest = compact[first_comma + 1 :]

    depth = 0
    array_end = -1
    for idx, char in enumerate(rest):
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                array_end = idx
                break

    if array_end == -1 or array_end + 1 >= len(rest) or rest[array_end + 1] != ",":
        raise ValueError(f"Malformed response array: {packet}")

    ret_val = rest[: array_end + 1]
    func_name = rest[array_end + 2 : -1]
    return error_id, ret_val, func_name


def parse_float_array(ret_val: str) -> list[float]:
    stripped = ret_val.strip()
    if stripped == "{}":
        return []
    if not stripped.startswith("{") or not stripped.endswith("}"):
        raise ValueError(f"Unexpected array format: {ret_val}")
    body = stripped[1:-1]
    if not body:
        return []
    return [float(item) for item in body.split(",") if item]


def format_command_values(values: list[float]) -> str:
    return ",".join(f"{value:.6f}" for value in values)


def format_values(values: list[float], digits: int = 3) -> str:
    return "[" + ", ".join(f"{value:.{digits}f}" for value in values) + "]"


def max_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=math.inf)


def is_valid_joint_set(joints6: list[float]) -> bool:
    j1, j2, j3, j4 = joints6[:4]
    j2_min = -25.0 if j3 > 0.0 else -19.0
    return (
        -160.0 <= j1 <= 160.0
        and j2_min <= j2 <= 85.0
        and -25.0 <= j3 <= 105.0
        and -160.0 <= j4 <= 160.0
        and -60.0 <= (j3 - j2) <= 60.0
    )


def normalize_case_key(joints6: list[float]) -> tuple[float, float, float, float]:
    return tuple(round(value, 6) for value in joints6[:4])


def build_cases_from_current(current_joints6: list[float], count: int = 50) -> list[list[float]]:
    base = current_joints6[:4]
    candidates: list[list[float]] = []
    seen: set[tuple[float, float, float, float]] = set()

    j1_offsets = [0.0, 10.0, -10.0, 20.0, -20.0]
    j2_offsets = [0.0, 10.0, -10.0]
    j3_offsets = [0.0, 10.0, -10.0]
    j4_offsets = [0.0, 20.0, -20.0, 40.0, -40.0]

    for j1_offset in j1_offsets:
        for j2_offset in j2_offsets:
            for j3_offset in j3_offsets:
                for j4_offset in j4_offsets:
                    joints6 = [
                        base[0] + j1_offset,
                        base[1] + j2_offset,
                        base[2] + j3_offset,
                        base[3] + j4_offset,
                        0.0,
                        0.0,
                    ]
                    key = normalize_case_key(joints6)
                    if key in seen or not is_valid_joint_set(joints6):
                        continue
                    seen.add(key)
                    candidates.append(joints6)
                    if len(candidates) >= count:
                        return candidates

    return candidates


def inverse_command_from_pose(pose4: list[float]) -> str:
    x, y, z, r = pose4
    return f"InverseSolution({x:.6f},{y:.6f},{z:.6f},{r:.6f},0.000000,0.000000,0,0)"


def run_case(
    sock: socket.socket,
    pending: bytearray,
    response_timeout: float,
    pass_threshold_deg: float,
    case_name: str,
    joints6: list[float],
) -> dict[str, object]:
    reference_joints = joints6[:4]

    positive_response = send_and_recv(
        sock,
        pending,
        f"PositiveSolution({format_command_values(joints6)},0,0)",
        response_timeout,
    )
    positive_error_id, positive_ret_val, _ = parse_dashboard_response(positive_response)
    pose = parse_float_array(positive_ret_val)
    if len(pose) < 4:
        return {
            "case_name": case_name,
            "status": "FAIL",
            "score": math.inf,
            "reference_joints": reference_joints,
            "positive_pose": None,
            "inverse_joints": None,
            "positive_error_id": positive_error_id,
            "inverse_error_id": None,
            "reason": "PositiveSolution did not return a 4-axis pose",
        }

    pose4 = pose[:4]
    inverse_response = send_and_recv(
        sock,
        pending,
        inverse_command_from_pose(pose4),
        response_timeout,
    )
    inverse_error_id, inverse_ret_val, _ = parse_dashboard_response(inverse_response)
    inverse_joints = parse_float_array(inverse_ret_val)
    if len(inverse_joints) < 4:
        return {
            "case_name": case_name,
            "status": "FAIL",
            "score": math.inf,
            "reference_joints": reference_joints,
            "positive_pose": pose4,
            "inverse_joints": None,
            "positive_error_id": positive_error_id,
            "inverse_error_id": inverse_error_id,
            "reason": "InverseSolution did not return a 4-axis result",
        }

    recovered_joints = inverse_joints[:4]
    diff = [recovered_joints[index] - reference_joints[index] for index in range(4)]
    score = max_abs(diff)
    status = "PASS" if score <= pass_threshold_deg else "FAIL"
    return {
        "case_name": case_name,
        "status": status,
        "score": score,
        "reference_joints": reference_joints,
        "positive_pose": pose4,
        "inverse_joints": recovered_joints,
        "positive_error_id": positive_error_id,
        "inverse_error_id": inverse_error_id,
        "reason": None,
    }


def print_case_summary(result: dict[str, object]) -> None:
    print(
        f"{result['status']:4s} {result['case_name']:<18} "
        f"diff={float(result['score']):.6f}deg "
        f"pos_err={result['positive_error_id']} inv_err={result['inverse_error_id']}"
        if math.isfinite(float(result["score"]))
        else f"{result['status']:4s} {result['case_name']:<18} reason={result['reason']}"
    )
    print(
        f"  PositiveSolution: {format_values(list(result['reference_joints']))} -> "
        f"{format_values(list(result['positive_pose'])) if result['positive_pose'] is not None else '(no 4-axis pose)'}"
    )
    print(
        f"  InverseSolution : "
        f"{format_values(list(result['inverse_joints'])) if result['inverse_joints'] is not None else '(no 4-axis joints)'} <- "
        f"{format_values(list(result['positive_pose'])) if result['positive_pose'] is not None else '(no 4-axis pose)'}"
    )
    if result["reason"] is not None:
        print(f"  reason: {result['reason']}")


def print_final_summary(results: list[dict[str, object]], pass_threshold_deg: float) -> None:
    total = len(results)
    passed = sum(1 for result in results if result["status"] == "PASS")
    failed = total - passed
    worst_result = max(results, key=lambda result: float(result["score"]))

    print("")
    print("Summary")
    print(
        f"  cases={total} pass={passed} fail={failed} "
        f"threshold={pass_threshold_deg:.6f}deg"
    )
    print(
        f"  worst_case: {worst_result['case_name']} "
        f"diff={worst_result['score']:.6f}deg"
        if math.isfinite(float(worst_result["score"]))
        else f"  worst_case: {worst_result['case_name']}"
    )


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()

    sock = open_socket_with_retry(
        args.ip,
        args.dashboard_port,
        args.connect_timeout,
        args.connect_retries,
        args.connect_retry_interval,
    )
    pending = bytearray()

    try:
        send_and_recv(sock, pending, "User(0)", args.response_timeout)
        send_and_recv(sock, pending, "Tool(0)", args.response_timeout)
        send_and_recv(sock, pending, "RobotMode()", args.response_timeout)
        send_and_recv(sock, pending, "GetErrorID()", args.response_timeout)

        response = send_and_recv(sock, pending, "GetAngle()", args.response_timeout)
        _, ret_val, _ = parse_dashboard_response(response)
        current_joints = parse_float_array(ret_val)
        if len(current_joints) >= 6:
            joints6 = current_joints[:6]
        elif len(current_joints) >= 4:
            joints6 = current_joints[:4] + [0.0, 0.0]
        else:
            raise RuntimeError("GetAngle() did not return a 4-axis result.")

        cases = build_cases_from_current(joints6, count=50)
        if len(cases) < 50:
            raise RuntimeError(f"Only {len(cases)} valid cases could be generated from GetAngle().")

        print(
            f"Running {len(cases)} cases "
            f"(threshold={args.pass_threshold_deg:.6f}deg)"
        )

        results = []
        for index, case_joints6 in enumerate(cases, start=1):
            result = run_case(
                sock,
                pending,
                args.response_timeout,
                args.pass_threshold_deg,
                f"case_{index:02d}",
                case_joints6,
            )
            results.append(result)
            print_case_summary(result)

        print_final_summary(results, args.pass_threshold_deg)
    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
