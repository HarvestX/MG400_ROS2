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

"""Exercise MCAP input, source timestamps and coefficient recovery without hardware."""

from copy import deepcopy
from pathlib import Path
import sys

from mg400_msgs.msg import JointCurrents, RobotMode
import numpy as np
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import JointState
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import parameter_identifier as identifier  # noqa: E402


@pytest.fixture
def experiment():
    """Build a full-rank stationary experiment with a known compensation model."""
    rng = np.random.default_rng(40)
    poses = [{'kind': 'base', 'joints_deg': [0, 45, 45, 0]}]
    for index in range(25):
        q = rng.uniform([-100, 15, 20, -100], [100, 60, 65, 100])
        poses.append({'kind': 'train' if index < 22 else 'check', 'joints_deg': q.tolist()})
    return {
        'schema_version': 1,
        'payload': {'load_kg': 0.2, 'center_x_mm': 10, 'center_y_mm': -20, 'center_z_mm': 30},
        'motion': {'speed_percent': 7, 'acceleration_percent': 9},
        'measurement': {'settle_sec': 0.1, 'record_sec': 1.2, 'repeats': 1},
        'identify': {
            'torque_constants_nm_per_a': [3.4, 3.3, 5.6, 0.75],
            'torque_signs': [-1, 1, 1, 1], 'joint_signs': [1, 1, 1, 1],
            'reference_deg': [0, 45, 45, 0], 'min_duration_sec': 1.0, 'max_span_deg': 0.5,
        },
        'poses': poses,
    }


BIAS = np.array([0.1, -0.2, 0.3, -0.4])
THETA = np.arange(40).reshape(10, 4) * 0.001


def yaml_time(stamp):
    """Preserve integer nanoseconds in sidecar timestamps."""
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return {'sec': sec, 'nanosec': nanosec}


def make_bag(tmp_path, experiment, name='run', mutate=None, incomplete=None):
    """Write raw telemetry and a YAML describing receipt-time measurement windows."""
    snapshot = deepcopy(experiment)
    topics = {source: '/arm/' + source for source in
              ('joint_states', 'joint_currents', 'robot_mode')}
    recording = snapshot['recording'] = {
        'topics': topics, 'enabled_payload': experiment['payload'], 'windows': [],
    }
    records = []
    constants = np.array(experiment['identify']['torque_constants_nm_per_a'])
    signs = np.array(experiment['identify']['torque_signs'])
    reference = identifier.features(np.deg2rad(experiment['identify']['reference_deg']))
    for segment, pose in enumerate(experiment['poses']):
        q = np.deg2rad(pose['joints_deg'])
        delta = BIAS + (identifier.features(q) - reference) @ THETA / (constants * signs)
        goal = dict(zip(identifier.JOINT_NAMES, q))
        start = 10_000_000_000 + segment * 2_000_000_000
        end = start + 1_300_000_000
        recording['windows'].append({
            'segment_id': segment, 'kind': pose['kind'], 'goal_rad': q.tolist(),
            'start': yaml_time(start), 'end': yaml_time(end), 'complete': segment != incomplete,
        })
        for index in range(7):
            stamp = start + index * 200_000_000
            records.append((stamp, topics['robot_mode'], RobotMode(robot_mode=5)))
            for source in ('joint_states', 'joint_currents'):
                source_stamp = stamp + (10_000_000 if source == 'joint_currents' else 0)
                if source == 'joint_states':
                    msg = JointState()
                    msg.name = ['prefix_' + n for n in reversed(identifier.JOINT_NAMES)]
                    msg.position = [float(goal[n]) for n in reversed(identifier.JOINT_NAMES)]
                else:
                    msg = JointCurrents(actual=(delta + 0.5).tolist(), target=[0.5] * 4)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                    source_stamp, 1_000_000_000)
                records.append((source_stamp + 20_000_000, topics[source], msg))
        # Invalid telemetry outside the half-open receipt-time window is ignored,
        # even when its source timestamp lies inside it.
        outside = deepcopy(msg)
        outside.actual[0] = float('nan')
        records.extend([(start - 1, topics['joint_currents'], outside),
                        (end, topics['joint_currents'], outside)])
        # Buffered telemetry received inside the window but produced before it is ignored.
        buffered = deepcopy(outside)
        buffered.header.stamp.sec, buffered.header.stamp.nanosec = divmod(start - 1, 1_000_000_000)
        records.append((start + 1, topics['joint_currents'], buffered))
    if mutate:
        mutate(records, snapshot)
    writer = rosbag2_py.SequentialWriter()
    bag_dir = tmp_path / name
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    for topic, type_name in (
        (topics['robot_mode'], 'mg400_msgs/msg/RobotMode'),
        (topics['joint_states'], 'sensor_msgs/msg/JointState'),
        (topics['joint_currents'], 'mg400_msgs/msg/JointCurrents'),
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=type_name, serialization_format='cdr'))
    for received, topic, msg in sorted(records, key=lambda record: record[0]):
        writer.write(topic, serialize_message(msg), received)
    del writer
    path = tmp_path / (name + '.mcap')
    next(bag_dir.glob('*.mcap')).rename(path)
    path.with_suffix('.yaml').write_text(yaml.safe_dump(snapshot, sort_keys=False))
    return path


def test_fit_mcap_with_matching_yaml_recovers_coefficients(tmp_path, experiment):
    """Recover known coefficients from source-stamped, reordered physical joint names."""
    path = make_bag(tmp_path, experiment)
    measurements, skipped = identifier.read_recording(path, [1] * 4)
    assert len(measurements) == 26 and not skipped
    assert measurements[0].duration == pytest.approx(1.19)
    assert measurements[0].joint_count == measurements[0].current_count == 6
    np.testing.assert_allclose(measurements[0].current, BIAS)
    assert path.with_suffix('.yaml').exists()
    output = tmp_path / 'identified.yaml'
    assert identifier.main([str(path), '--output', str(output)]) == 0
    result = yaml.safe_load(output.read_text())
    assert set(result) == {'joint_current_bias_a', 'posture_coefficients_nm'}
    np.testing.assert_allclose(result['joint_current_bias_a'], BIAS, atol=1e-12)
    reference = identifier.features(np.deg2rad([0, 45, 45, 0]))
    expected = np.column_stack((-reference @ THETA, THETA.T))
    np.testing.assert_allclose(np.reshape(result['posture_coefficients_nm'], (4, 11)),
                               expected, atol=1e-10)


def test_incomplete_segment_is_excluded(tmp_path, experiment):
    """Keep completed poses while excluding a stopped measurement window."""
    path = make_bag(tmp_path, experiment, incomplete=25)
    samples, skipped = identifier.read_recording(path, [1] * 4)
    assert len(samples) == 25
    assert len(skipped) == 1 and 'incomplete' in skipped[0]


@pytest.mark.parametrize(('key', 'value', 'error'), [
    ('start', {'sec': 10, 'nanosec': 1_000_000_000}, 'stamp_nanosec'),
    ('end', {'sec': 9, 'nanosec': 0}, 'backwards'),
    ('segment_id', 1, 'invalid measurement window'),
    ('complete', 'true', 'invalid measurement window'),
    ('kind', 'move', 'invalid measurement window'),
])
def test_rejects_invalid_windows(tmp_path, experiment, key, value, error):
    """Reject ambiguous or malformed YAML measurement intervals."""
    def mutate(records, snapshot):
        snapshot['recording']['windows'][0][key] = value
    path = make_bag(tmp_path, experiment, mutate=mutate)
    with pytest.raises(ValueError, match=error):
        identifier.read_recording(path, [1] * 4)


def test_rejects_running_mode_during_sampling(tmp_path, experiment):
    """Read the robot state from its original topic before fitting a window."""
    def mutate(records, snapshot):
        next(msg for _, _, msg in records if isinstance(msg, RobotMode)).robot_mode = 7
    path = make_bag(tmp_path, experiment, mutate=mutate)
    with pytest.raises(ValueError, match='ENABLE'):
        identifier.read_recording(path, [1] * 4)


def test_rejects_complete_window_without_samples(tmp_path, experiment):
    """Do not silently drop a window incorrectly labeled as complete."""
    def mutate(records, snapshot):
        records[:] = [record for record in records if record[0] >= 12_000_000_000]
    path = make_bag(tmp_path, experiment, mutate=mutate)
    with pytest.raises(ValueError, match='no samples'):
        identifier.read_recording(path, [1] * 4)


def test_cli_pairs_yaml_by_filename(tmp_path, experiment):
    """Use the same-stem YAML, including changes to its identification settings."""
    path = make_bag(tmp_path, experiment)
    snapshot = path.with_suffix('.yaml')
    output = tmp_path / 'identified.yaml'
    args = [str(path), '--output', str(output)]
    # An unrelated YAML file is not selected when the matching one is missing.
    renamed = snapshot.rename(tmp_path / 'unrelated.yaml')
    assert identifier.main(args) == 2
    assert not output.exists()
    renamed.rename(snapshot)
    # No embedded snapshot or digest comparison: read the named YAML's values.
    experiment = yaml.safe_load(snapshot.read_text())
    experiment['identify']['torque_constants_nm_per_a'] = [
        2 * value for value in experiment['identify']['torque_constants_nm_per_a']]
    snapshot.write_text(yaml.safe_dump(experiment, sort_keys=True) + '\n# edited settings\n')
    assert identifier.main(args) == 0
    result = yaml.safe_load(output.read_text())
    coefficients = np.reshape(result['posture_coefficients_nm'], (4, 11))
    np.testing.assert_allclose(coefficients[:, 1:], 2 * THETA.T, atol=1e-10)


def test_cli_uses_each_yaml_and_protects_both_inputs(tmp_path, experiment):
    """Pair each run with its YAML while preserving the recordings and settings."""
    first = make_bag(tmp_path, experiment)
    second = make_bag(tmp_path, experiment, name='second')
    output = tmp_path / 'output.yaml'
    args = [str(first), str(second), '--output', str(output)]
    assert identifier.main(args) == 0
    assert identifier.main(args) == 2
    assert identifier.main(args + ['--force']) == 0
    for path in (first, second, first.with_suffix('.yaml'), second.with_suffix('.yaml')):
        original = path.read_bytes()
        assert identifier.main([str(first), str(second), '--output', str(path), '--force']) == 2
        assert path.read_bytes() == original
    # Non-model settings and formatting need not match between experiments.
    experiment = yaml.safe_load(second.with_suffix('.yaml').read_text())
    experiment['motion']['speed_percent'] = 8
    second.with_suffix('.yaml').write_text(yaml.safe_dump(experiment) + '\n# another run\n')
    assert identifier.main(args + ['--force']) == 0
    # A missing second YAML must not silently reuse the first run's settings.
    second.with_suffix('.yaml').unlink()
    assert identifier.main(args + ['--force']) == 2


def test_rejects_combining_incompatible_models(tmp_path, experiment):
    """Do not silently mix torque conversions when jointly fitting multiple runs."""
    first = make_bag(tmp_path, experiment)
    experiment['identify']['torque_constants_nm_per_a'][0] *= 2
    second = make_bag(tmp_path, experiment, name='second')
    output = tmp_path / 'output.yaml'
    assert identifier.main([str(first), str(second), '--output', str(output)]) == 2
    assert not output.exists()
