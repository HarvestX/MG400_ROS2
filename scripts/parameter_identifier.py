#!/usr/bin/env python3
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

"""Identify static current bias and posture compensation from MG400 panel MCAP recordings."""

import argparse
from bisect import bisect_right
from dataclasses import dataclass
import os
from pathlib import Path
import sys
import tempfile

import numpy as np
import yaml


JOINTS = [f'j{i}_rad' for i in range(1, 5)]
ACTUAL = [f'actual_j{i}_a' for i in range(1, 5)]
TARGET = [f'target_j{i}_a' for i in range(1, 5)]
PAYLOAD = ['panel_load_kg', 'panel_center_x_mm', 'panel_center_y_mm', 'panel_center_z_mm']
GOALS = [f'goal_j{i}_rad' for i in range(1, 5)]
KINDS = {'base', 'train', 'check'}
JOINT_NAMES = ('mg400_j1', 'mg400_j2_1', 'mg400_j4_2', 'mg400_j5')


class ExperimentLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys rather than silently accepting the last value."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=True)
        if not isinstance(key, str) or key in result:
            raise ValueError(f'duplicate or invalid YAML key: {key}')
        result[key] = loader.construct_object(value_node, deep=True)
    return result


ExperimentLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load_experiment(path):
    """Load the same explicit, unit-qualified settings used by the RViz panel."""
    return parse_experiment(Path(path).read_bytes())


def parse_experiment(raw):
    """Validate the experiment YAML stored beside a recording."""
    if len(raw) > 64 * 1024 * 1024:
        raise ValueError('experiment YAML exceeds 64 MiB')
    root = yaml.load(raw, Loader=ExperimentLoader)

    def keys(value, expected, name):
        if not isinstance(value, dict) or set(value) != set(expected.split()):
            raise ValueError(f'{name}: missing or unexpected settings')

    def number(value, minimum, maximum, name, integer=False):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not np.isfinite(value)
            or not minimum <= value <= maximum
            or (integer and not isinstance(value, int))
        ):
            raise ValueError(f'{name}: invalid numeric value')
        return value

    def vector(value, name):
        if not isinstance(value, list) or len(value) != 4:
            raise ValueError(f'{name}: expected four values')
        return [number(v, -np.inf, np.inf, name) for v in value]

    keys(root, 'schema_version payload motion measurement identify poses'
         + (' recording' if isinstance(root, dict) and 'recording' in root else ''), 'experiment')
    number(root['schema_version'], 1, 1, 'schema_version', integer=True)
    payload = root['payload']
    suffix = '_m' if isinstance(payload, dict) and 'center_x_m' in payload else '_mm'
    keys(payload, 'load_kg ' + ' '.join('center_' + axis + suffix for axis in 'xyz'), 'payload')
    payload_kg_mm = [number(payload['load_kg'], 0, 0.75, 'load_kg')]
    scale = 1000 if suffix == '_m' else 1
    for axis in 'xyz':
        key = 'center_' + axis + suffix
        payload_kg_mm.append(scale * number(payload[key], -500 / scale, 500 / scale, key))
    if any(abs(v * 1000 - round(v * 1000)) > 1e-7 for v in payload_kg_mm):
        raise ValueError('Enable payload supports at most three decimals in kg/mm')
    motion = root['motion']
    keys(motion, 'speed_percent acceleration_percent', 'motion')
    for key in motion:
        number(motion[key], 1, 100, key, integer=True)
    measurement = root['measurement']
    keys(measurement, 'settle_sec record_sec repeats', 'measurement')
    number(measurement['settle_sec'], 0.1, 30, 'settle_sec')
    number(measurement['record_sec'], 1.2, 60, 'record_sec')
    number(measurement['repeats'], 1, 100, 'repeats', integer=True)
    settings = root['identify']
    keys(
        settings,
        'torque_constants_nm_per_a torque_signs joint_signs reference_deg '
        'min_duration_sec max_span_deg',
        'identify',
    )
    if any(
        v <= 0 for v in vector(settings['torque_constants_nm_per_a'], 'torque_constants_nm_per_a')
    ):
        raise ValueError('torque_constants_nm_per_a must be positive')
    for key in ('torque_signs', 'joint_signs'):
        if any(v not in (-1, 1) for v in vector(settings[key], key)):
            raise ValueError(f'{key} must contain only -1 or 1')
    vector(settings['reference_deg'], 'reference_deg')
    number(settings['min_duration_sec'], 0.001, 60, 'min_duration_sec')
    number(settings['max_span_deg'], 0.001, 180, 'max_span_deg')
    if measurement['record_sec'] < settings['min_duration_sec'] + 0.1:
        raise ValueError('record_sec must exceed min_duration_sec by at least 0.1s')
    poses = root['poses']
    if not isinstance(poses, list) or not 1 <= len(poses) <= 1000:
        raise ValueError('poses must contain 1 to 1000 entries')
    for pose in poses:
        keys(pose, 'kind joints_deg', 'pose')
        if pose['kind'] not in KINDS | {'move'}:
            raise ValueError('pose.kind must be base, train, check, or move')
        vector(pose['joints_deg'], 'joints_deg')
    return root, np.array(payload_kg_mm)


@dataclass
class Measurement:
    """One completed, stationary measurement window with equal fit weight."""

    name: str
    kind: str
    angles: np.ndarray
    features: np.ndarray
    current: np.ndarray
    payload: np.ndarray
    joint_count: int
    current_count: int
    duration: float


def features(angles):
    """Match the estimator's ten nonconstant features, with angles in radians."""
    q = np.asarray(angles, dtype=float)
    result = np.empty(q.shape[:-1] + (10,))
    result[..., :8:2] = np.sin(q)
    result[..., 1:8:2] = np.cos(q)
    result[..., 8] = np.sin(q[..., 2] - q[..., 1])
    result[..., 9] = np.cos(q[..., 2] - q[..., 1])
    return result


def numbers(row, keys):
    """Read mandatory finite numeric values, without treating empty cells as zero."""
    try:
        result = np.array([float(row[key]) for key in keys])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'invalid or missing values for {keys}') from exc
    if not np.all(np.isfinite(result)):
        raise ValueError(f'non-finite values for {keys}')
    return result


def timestamp(row):
    """Preserve integer ROS timestamps until subtracting an epoch."""
    sec, nsec = int(row['stamp_sec']), int(row['stamp_nanosec'])
    if not 0 <= nsec < 1_000_000_000:
        raise ValueError('stamp_nanosec outside [0, 1000000000)')
    return sec * 1_000_000_000 + nsec


def summarize(rows, name, kind, joint_signs, min_duration, max_span_deg):
    """Average independent streams over their shared stationary time interval."""
    streams = {'joint_states': [], 'joint_currents': []}
    payloads = []
    goal = None
    for row in rows:
        source = row['source']
        if source not in streams:
            continue
        if row['latest_robot_mode'] != '5':
            raise ValueError(f'{name}: sampling requires robot mode ENABLE (5)')
        if row.get('enable_payload_confirmed') != '1':
            raise ValueError(f'{name}: payload was not confirmed by this panel')
        payloads.append(numbers(row, PAYLOAD))
        stamp = timestamp(row)
        if streams[source] and stamp <= streams[source][-1][0]:
            raise ValueError(f'{name}: {source} timestamps are duplicate or go backwards')
        values = numbers(row, JOINTS if source == 'joint_states' else ACTUAL + TARGET)
        streams[source].append((stamp, values))
        requested = numbers(row, GOALS)
        if goal is not None and not np.allclose(requested, goal, atol=1e-10, rtol=0):
            raise ValueError(f'{name}: goal changed within a segment')
        goal = requested
    if any(len(values) < 5 for values in streams.values()):
        raise ValueError(f'{name}: need at least five messages from each stream')
    if not np.allclose(payloads, payloads[0], atol=1e-9, rtol=0):
        raise ValueError(f'{name}: payload changed during measurement')
    if not 0 <= payloads[0][0] <= 0.75 or np.any(np.abs(payloads[0][1:]) > 500):
        raise ValueError(f'{name}: invalid payload metadata')
    start = max(values[0][0] for values in streams.values())
    end = min(values[-1][0] for values in streams.values())
    duration = (end - start) / 1e9
    if duration < min_duration:
        raise ValueError(f'{name}: shared time interval {duration:.3f}s < {min_duration}s')
    arrays = {}
    for source, samples in streams.items():
        selected = [(stamp, value) for stamp, value in samples if start <= stamp <= end]
        if len(selected) < 5:
            raise ValueError(f'{name}: too few overlapping {source} messages')
        # Sparse bursts are not evidence that the arm stayed still through a long gap.
        if any((b[0] - a[0]) / 1e9 > 0.5 for a, b in zip(selected, selected[1:])):
            raise ValueError(f'{name}: {source} has a gap greater than 0.5s')
        arrays[source] = np.array([value for _, value in selected])
    joints = arrays['joint_states']
    if np.max(np.ptp(joints, axis=0)) > np.deg2rad(max_span_deg):
        raise ValueError(f'{name}: joint movement exceeds {max_span_deg} degrees')
    if np.max(np.abs(joints - goal)) > np.deg2rad(1.0):
        raise ValueError(f'{name}: actual angles do not match the requested pose')
    signed = joints * joint_signs
    currents = arrays['joint_currents']
    return Measurement(
        name,
        kind,
        signed.mean(axis=0),
        features(signed).mean(axis=0),
        (currents[:, :4] - currents[:, 4:]).mean(axis=0),
        payloads[0],
        len(joints),
        len(currents),
        duration,
    )


def window_timestamp(value):
    """Read a YAML timestamp without losing nanosecond precision."""
    if (not isinstance(value, dict) or set(value) != {'sec', 'nanosec'}
            or any(type(value[key]) is not int for key in value)):
        raise ValueError('window timestamps require integer sec and nanosec')
    return timestamp({'stamp_sec': value['sec'], 'stamp_nanosec': value['nanosec']})


def read_mcap(path, recording=None):
    """Select raw telemetry using receipt-time measurement windows in the matching YAML."""
    # Keep --help usable without a sourced ROS environment.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    path = Path(path)
    if not path.is_file() or path.suffix.lower() != '.mcap':
        raise ValueError(f'{path}: expected a Run and record .mcap file')
    if recording is None:
        experiment, _ = load_experiment(path.with_suffix('.yaml'))
        recording = experiment.get('recording')
    if not isinstance(recording, dict) or not isinstance(recording.get('windows'), list):
        raise ValueError(f'{path}: matching YAML has no measurement windows')
    topics = recording['topics']
    source_types = {
        'joint_states': 'sensor_msgs/msg/JointState',
        'joint_currents': 'mg400_msgs/msg/JointCurrents',
        'robot_mode': 'mg400_msgs/msg/RobotMode',
    }
    payload = recording['enabled_payload']
    payload_values = [payload[key] for key in
                      ('load_kg', 'center_x_mm', 'center_y_mm', 'center_z_mm')]
    windows, starts, segments = [], [], set()
    rows = []
    for entry in recording['windows']:
        segment = entry['segment_id']
        start, end = window_timestamp(entry['start']), window_timestamp(entry['end'])
        if (type(segment) is not int or segment < 0 or segment in segments
                or type(entry['complete']) is not bool or entry['kind'] not in KINDS
                or not isinstance(entry['goal_rad'], list) or len(entry['goal_rad']) != 4):
            raise ValueError(f'{path}: invalid measurement window')
        if end < start or (windows and start < windows[-1][1]):
            raise ValueError(f'{path}: measurement windows overlap or go backwards')
        segments.add(segment)
        starts.append(start)
        base = {
            'segment_id': str(segment), 'sample_kind': entry['kind'],
            'enable_payload_confirmed': '1', **dict(zip(PAYLOAD, payload_values)),
            **dict(zip(GOALS, entry['goal_rad'])),
        }
        windows.append((start, end, entry['complete'], base))
        if entry['complete']:
            sec, nsec = divmod(end, 1_000_000_000)
            rows.append({**base, 'source': 'segment_end', 'phase': 'complete',
                         'stamp_sec': sec, 'stamp_nanosec': nsec})
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    try:
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        for source, expected in source_types.items():
            if types.get(topics[source]) != expected:
                raise ValueError(f'{path}: missing or incorrect {source} topic')
        sources = {topics[source]: source for source in source_types}
        classes = {source: get_message(type_name) for source, type_name in source_types.items()}
        mode, mode_time = None, None
        while reader.has_next():
            topic, data, received = reader.read_next()
            if topic not in sources:
                continue
            source = sources[topic]
            msg = deserialize_message(data, classes[source])
            index = bisect_right(starts, received) - 1
            window = windows[index] if index >= 0 and received < windows[index][1] else None
            if source == 'robot_mode':
                mode, mode_time = msg.robot_mode, received
                if window and window[2] and mode != 5:
                    raise ValueError(f'{path}: sampling requires robot mode ENABLE (5)')
                continue
            if window is None:
                continue
            row = {
                **window[3], 'source': source, 'phase': 'sample',
                'stamp_sec': msg.header.stamp.sec, 'stamp_nanosec': msg.header.stamp.nanosec,
                'latest_robot_mode': str(mode) if mode_time is not None
                and 0 <= received - mode_time < 1_000_000_000 else '',
            }
            # DDS can deliver pre-window telemetry after recording starts.
            if timestamp(row) < window[0]:
                continue
            if source == 'joint_states':
                angles = []
                for suffix in JOINT_NAMES:
                    indices = [i for i, name in enumerate(msg.name) if name.endswith(suffix)]
                    if len(indices) != 1 or indices[0] >= len(msg.position):
                        raise ValueError(f'{path}: invalid physical joint {suffix}')
                    angles.append(msg.position[indices[0]])
                row.update(zip(JOINTS, angles))
            else:
                row.update(zip(ACTUAL + TARGET, list(msg.actual) + list(msg.target)))
            rows.append(row)
    finally:
        del reader
    return rows


def read_recording(path, joint_signs, min_duration=1.0, max_span_deg=0.5):
    """Read completed measurement windows from a labeled panel MCAP."""
    return summarize_recording(read_mcap(path), str(path), joint_signs, min_duration, max_span_deg)


def summarize_recording(rows, path, joint_signs, min_duration=1.0, max_span_deg=0.5):
    """Exclude interrupted windows and retain the original identification checks."""
    groups, completions = {}, {}
    for row in rows:
        segment = row.get('segment_id', '')
        phase = row.get('phase', '')
        if phase == 'sample':
            if not segment or row.get('sample_kind') not in KINDS:
                raise ValueError(f'{path}: invalid sample segment metadata')
            groups.setdefault(segment, []).append(row)
        elif phase == 'complete' and row['source'] == 'segment_end':
            if not segment or segment in completions:
                raise ValueError(f'{path}: invalid/duplicate segment completion')
            completions[segment] = row
    if not groups:
        raise ValueError(f'{path}: no labeled samples; use a Run and record MCAP')
    if completions.keys() - groups.keys():
        raise ValueError(f'{path}: complete measurement window has no samples')
    result, skipped = [], []
    for segment, samples in groups.items():
        name = f'{path}:segment {segment}'
        if segment not in completions:
            skipped.append(name + ' (incomplete; excluded)')
            continue
        complete = completions[segment]
        kinds = {row['sample_kind'] for row in samples} | {complete['sample_kind']}
        if len(kinds) != 1 or not kinds.issubset(KINDS):
            raise ValueError(f'{name}: sample kind changed')
        if timestamp(complete) < max(timestamp(row) for row in samples):
            raise ValueError(f'{name}: completion precedes measurements')
        result.append(
            summarize(samples, name, kinds.pop(), joint_signs, min_duration, max_span_deg)
        )
    return result, skipped


def error_metrics(error):
    """Report unfiltered torque residuals; no deadband can hide fit error."""
    return {
        'rmse_nm': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        'max_abs_nm': np.max(np.abs(error), axis=0).tolist(),
    }


def identify(measurements, torque_constants, torque_signs, joint_signs, reference_deg):
    """Fit a reference-anchored posture model using equal weight per interval."""
    constants = np.asarray(torque_constants, dtype=float)
    signs = np.asarray(torque_signs, dtype=float)
    qsigns = np.asarray(joint_signs, dtype=float)
    if constants.shape != (4,) or not np.all(np.isfinite(constants)) or np.any(constants <= 0):
        raise ValueError('torque constants must be four positive finite values in Nm/A')
    if signs.shape != (4,) or not np.all(np.isin(signs, [-1, 1])):
        raise ValueError('torque signs must each be -1 or 1')
    if qsigns.shape != (4,) or not np.all(np.isin(qsigns, [-1, 1])):
        raise ValueError('joint signs must each be -1 or 1')
    reference = np.deg2rad(np.asarray(reference_deg, dtype=float)) * qsigns
    if reference.shape != (4,) or not np.all(np.isfinite(reference)):
        raise ValueError('reference must contain four finite angles')
    if not measurements:
        raise ValueError('no complete measurements')
    if not np.allclose(
        [m.payload for m in measurements], measurements[0].payload, atol=1e-9, rtol=0
    ):
        raise ValueError(
            'recordings use different payloads; identify each configuration separately'
        )
    baseline = [m for m in measurements if m.kind == 'base']
    train = [m for m in measurements if m.kind == 'train']
    validation = [m for m in measurements if m.kind == 'check']
    if not baseline:
        raise ValueError('at least one base measurement is required')
    if any(np.max(np.abs(m.angles - reference)) > np.deg2rad(0.5) for m in baseline):
        raise ValueError(
            'base measurements differ from YAML reference_deg by more than 0.5 degrees'
        )
    if len(train) < 10:
        raise ValueError('need at least ten independent train poses for the ten-feature model')
    bias = np.mean([m.current for m in baseline], axis=0)
    anchor = features(reference)
    x = np.array([m.features - anchor for m in train])
    y = (np.array([m.current for m in train]) - bias) * constants * signs
    theta, _, rank, singular = np.linalg.lstsq(x, y, rcond=None)
    if rank < 10:
        raise ValueError(
            f'pose feature matrix rank is {rank}/10; add varied poses (including J3-J2)'
        )
    condition = float(singular[0] / singular[-1])
    if condition > 1e6:
        raise ValueError(f'pose matrix is ill-conditioned ({condition:.3g}); widen pose coverage')
    coefficients = np.column_stack((-anchor @ theta, theta.T))
    report = {
        'base_intervals': len(baseline),
        'train_intervals': len(train),
        'check_intervals': len(validation),
        'feature_rank': int(rank),
        'condition_number': condition,
        'singular_values': singular.tolist(),
        'base_bias_std_a': np.std([m.current for m in baseline], axis=0).tolist(),
        'train_before': error_metrics(y),
        'train_after': error_metrics(y - x @ theta),
    }
    if validation:
        vx = np.array([m.features - anchor for m in validation])
        vy = (np.array([m.current for m in validation]) - bias) * constants * signs
        report['check_before'] = error_metrics(vy)
        report['check_after'] = error_metrics(vy - vx @ theta)
    report['warnings'] = []
    if not validation:
        report['warnings'].append(
            'No held-out validation data; training residual is not validation.'
        )
    if condition > 1e4:
        report['warnings'].append(
            'Feature condition number exceeds 1e4; coefficients may be sensitive to noise.'
        )
    config = {
        'use_target_current_compensation': True,
        'target_current_scale': [1.0] * 4,
        'torque_constants': constants.tolist(),
        'joint_torque_signs': signs.tolist(),
        'joint_signs': qsigns.tolist(),
        'joint_current_bias': bias.tolist(),
        'auto_bias_sample_count': 0,
        'use_posture_compensation': True,
        'posture_coefficients': coefficients.ravel().tolist(),
        'use_friction_compensation': False,
    }
    return {
        'schema_version': 1,
        'model': 'static_reference_anchored_posture',
        'reference_deg': list(map(float, reference_deg)),
        'payload_kg_mm': measurements[0].payload.tolist(),
        'config': config,
        'diagnostics': report,
        'measurements': [
            {
                'name': m.name,
                'kind': m.kind,
                'duration_s': m.duration,
                'joint_samples': m.joint_count,
                'current_samples': m.current_count,
                'mean_signed_angles_rad': m.angles.tolist(),
                'mean_current_difference_a': m.current.tolist(),
            }
            for m in measurements
        ],
    }


def write_atomic(path, text):
    """Write each output through a temporary file in the destination directory."""
    path = Path(path)
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=path.parent, prefix=path.name + '.', delete=False
    ) as stream:
        temporary = Path(stream.name)
        try:
            stream.write(text)
        except BaseException:
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main(argv=None):
    """Run offline identification; this command never connects to the robot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path,
                        help='MCAP files, each accompanied by a same-stem .yaml file')
    parser.add_argument('--output', type=Path, required=True, help='YAML coefficients only')
    parser.add_argument('--force', action='store_true', help='Allow replacing output files')
    args = parser.parse_args(argv)
    try:
        resolved = [path.resolve() for path in args.recordings]
        if len(set(resolved)) != len(resolved):
            raise ValueError('the same input file was supplied more than once')
        protected = set(resolved)
        protected.update(path.with_suffix('.yaml').resolve() for path in args.recordings)
        if args.output.resolve() in protected:
            raise ValueError('output must not replace an input recording or experiment YAML')
        if args.output.exists() and not args.force:
            raise ValueError(f'{args.output} already exists; use --force to replace it')
        measurements = []
        model_settings = None
        for path in args.recordings:
            config_path = path.with_suffix('.yaml')
            experiment, payload_kg_mm = load_experiment(config_path)
            settings = experiment['identify']
            # Files are paired solely by name. Only model inputs must agree when
            # combining runs; pose lists, comments and motion settings may differ.
            current_model = {key: settings[key] for key in (
                'torque_constants_nm_per_a', 'torque_signs', 'joint_signs', 'reference_deg')}
            if model_settings is not None and current_model != model_settings:
                raise ValueError('cannot combine runs with different torque or reference settings')
            model_settings = current_model
            rows = read_mcap(path, experiment.get('recording'))
            samples, _ = summarize_recording(
                rows, str(path), np.array(settings['joint_signs']),
                settings['min_duration_sec'], settings['max_span_deg'],
            )
            if any(not np.allclose(m.payload, payload_kg_mm, atol=1e-9, rtol=0) for m in samples):
                raise ValueError(f'{path}: recorded Enable payload differs from {config_path}')
            measurements.extend(samples)
        result = identify(
            measurements,
            settings['torque_constants_nm_per_a'],
            settings['torque_signs'],
            settings['joint_signs'],
            settings['reference_deg'],
        )
        output = {
            'joint_current_bias_a': result['config']['joint_current_bias'],
            'posture_coefficients_nm': result['config']['posture_coefficients'],
        }
        write_atomic(args.output, yaml.safe_dump(output, sort_keys=False, default_flow_style=None))
        print(f'Saved: {args.output}')
        return 0
    except (
        OSError,
        ImportError,
        RuntimeError,
        ValueError,
        KeyError,
        TypeError,
        yaml.YAMLError,
        np.linalg.LinAlgError,
    ) as exc:
        print(f'Identification failed: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
