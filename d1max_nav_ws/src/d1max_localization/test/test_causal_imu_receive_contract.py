"""Causal queue admission, including original recorded IMU bytes/timestamps.

The bag tests exercise source/receipt admission, not physical clock calibration
or navigation success. They do not launch ROS, replay a bag or change headers.
"""
import os
from pathlib import Path
import sqlite3

import pytest
import yaml

from d1max_localization.estimation.causal_prediction import CausalInertialPredictor
from d1max_localization.estimation.prediction import InertialPredictor, PredictionLimits
from test_navigation_estimation import snapshot


ACC = (0., 0., 9.81)
ANGULAR = (0., 0., 0.)


def core():
    result = CausalInertialPredictor(PredictionLimits(
        max_horizon=.35, max_coast=.1, max_imu_gap=.1))
    assert result.accept(snapshot(100.), 100.)
    return result


def test_causal_queue_matches_input_clock_without_changing_legacy_consumer():
    legacy = InertialPredictor()
    causal = core()
    assert not legacy.push_imu(100.03, ACC, ANGULAR, 100.)
    assert causal.push_imu(100.03, ACC, ANGULAR, 100.)
    assert causal.imu[-1][0] == 100.03  # Original source, not receipt-stamped.
    assert not causal.push_imu(100.051, ACC, ANGULAR, 100.)
    assert not causal.push_imu(99.799, ACC, ANGULAR, 100.)
    assert causal.imu_received == 1 and causal.rejected == 2


def test_future_endpoint_is_never_consumed_before_the_actual_target():
    causal, baseline = core(), core()
    assert causal.push_imu(100., ACC, ANGULAR, 100.)
    assert baseline.push_imu(100., ACC, ANGULAR, 100.)
    assert causal.push_imu(100.04, (1., 0., 9.81), (0., 0., .2), 100.)
    before = tuple(causal.imu)
    state, expected = causal.evaluate(100.02), baseline.evaluate(100.02)
    assert state is not None and expected is not None
    assert state.imu_stamp == 100. and state.imu_stamp <= state.stamp
    assert state.pose == expected.pose and state.world_velocity == expected.world_velocity
    assert tuple(causal.imu) == before  # Filtering is not destructive consumption.
    admitted = causal.evaluate(100.04)
    assert admitted is not None and admitted.imu_stamp == 100.04
    assert admitted.source_stamp == 100. and admitted.stamp == 100.04
    assert admitted.world_velocity != expected.world_velocity


def test_future_only_queue_does_not_fabricate_past_coverage():
    causal = core()
    assert causal.push_imu(100.04, ACC, ANGULAR, 100.)
    assert causal.evaluate(100.) is None and causal.reason == 'imu_stale'
    assert causal.evaluate(100.04) is None and causal.reason == 'imu_start_uncovered'


def test_duplicate_future_packet_neither_renews_source_nor_buffer():
    causal = core()
    assert causal.push_imu(100., ACC, ANGULAR, 100.)
    assert causal.push_imu(100.04, ACC, ANGULAR, 100.)
    original = tuple(causal.imu)
    assert not causal.push_imu(100.04, ACC, ANGULAR, 100.05)
    assert not causal.push_imu(100.03, ACC, ANGULAR, 100.05)
    assert tuple(causal.imu) == original and causal.imu_received == 2
    assert causal.evaluate(100.141) is None and causal.reason == 'imu_stale'
    assert causal.snapshot[0] == 100.  # Original posterior deadline unchanged.


def test_future_buffer_keeps_raw_clock_reset_and_epoch_rules():
    causal = core()
    assert causal.push_imu(100., ACC, ANGULAR, 100.)
    assert causal.push_imu(100.04, ACC, ANGULAR, 100.)
    assert not causal.push_imu(99.98, ACC, ANGULAR, 100.)
    assert causal.fault == 'imu_clock_reset'
    assert causal.evaluate(100.04) is None
    old = snapshot(100.)
    old['epoch'] = 0
    with pytest.raises(ValueError):
        causal.accept(old, 100.04)
    assert causal.fault == 'imu_clock_reset'


def test_admission_does_not_extend_state_gap_or_rotation_limits():
    causal = core()
    assert causal.push_imu(100., ACC, ANGULAR, 100.)
    assert causal.push_imu(100.11, ACC, ANGULAR, 100.08)
    assert causal.evaluate(100.11) is None and causal.reason == 'imu_gap'
    rotation = core()
    assert rotation.push_imu(100., ACC, (0., 0., 2.), 100.)
    assert rotation.push_imu(100.05, ACC, (0., 0., 2.), 100.01)
    assert rotation.evaluate(100.05) is None and rotation.reason == 'imu_gap'
    assert rotation.gap_stats['rejected_reason'] == 'rotation'


def test_future_queue_is_bounded_even_under_unrealistic_input_rate():
    causal = core()
    for i in range(5000):
        assert causal.push_imu(100.+i*1e-6, ACC, ANGULAR, 100.)
    assert len(causal.imu) == 1024 and causal.imu.maxlen == 1024
    # Losing the real anchor because the bounded queue is flooded does not
    # justify inventing one or using the future head as an earlier observation.
    assert causal.evaluate(100.) is None


@pytest.fixture(scope='module')
def recorded_imu():
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    default = Path(__file__).resolve().parents[3]/'bags'/'slam_raw_20260923_010248_eb79d8'
    bag = Path(os.environ.get('D1MAX_CAUSAL_IMU_TEST_BAG', default))
    if not (bag/'metadata.yaml').exists():
        pytest.skip('Original 09-23 bag unavailable; pure causal tests remain mandatory')
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    rows = []
    first = offset = None
    for relative in metadata['relative_file_paths']:
        with sqlite3.connect('file:'+str(bag/relative)+'?mode=ro', uri=True) as db:
            topics = {name:tid for tid,name in db.execute('SELECT id,name FROM topics')}
            if first is None:
                ids = tuple(topics[n] for n in ('/front_lidar','/rear_lidar','/front_lidar/imu'))
                first = db.execute('SELECT MIN(timestamp) FROM messages WHERE topic_id IN (?,?,?)', ids).fetchone()[0]
                initial = db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 200',
                    (topics['/front_lidar/imu'],)).fetchall()
                deltas = []
                for receipt, data in initial:
                    message = deserialize_message(data, Imu)
                    deltas.append(message.header.stamp.sec*10**9+message.header.stamp.nanosec-receipt)
                offset = max(deltas)+2_000_000  # Exact original replay clock contract.
            for receipt, data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<=? ORDER BY timestamp,id',
                    (topics['/front_lidar/imu'], first, first+240_000_000_000)):
                message = deserialize_message(data, Imu)
                source = message.header.stamp.sec*10**9+message.header.stamp.nanosec
                rows.append((receipt+offset, source, message))
    return sorted(rows, key=lambda row:row[0])


def test_original_bag_phase_packets_are_buffered_not_lost_or_restamped(recorded_imu):
    causal, legacy = CausalInertialPredictor(), InertialPredictor()
    original = tuple((receipt,source) for receipt,source,_ in recorded_imu)
    future_count = 0
    legacy_phase_losses = 0
    for receipt, source, message in recorded_imu:
        now, stamp = receipt*1e-9, source*1e-9
        a, w = message.linear_acceleration, message.angular_velocity
        acceleration, angular = (a.x,a.y,a.z), (w.x,w.y,w.z)
        admitted = causal.push_imu(stamp, acceleration, angular, now)
        legacy_admitted = legacy.push_imu(stamp, acceleration, angular, now)
        if .010001 < stamp-now < .049999:
            future_count += 1
            assert admitted and not legacy_admitted
            assert causal.imu[-1][0] == stamp
            legacy_phase_losses += 1
        elif now-stamp > .200001:
            assert not admitted  # Genuine original receipt age still expires.
    assert future_count > 1000 and legacy_phase_losses == future_count
    assert causal.rejected < legacy.rejected and len(causal.imu) <= 1024
    assert tuple((receipt,source) for receipt,source,_ in recorded_imu) == original


def test_original_bag_future_header_cannot_authorize_an_earlier_state(recorded_imu):
    receipt, source, message = next(row for row in recorded_imu if .02 < (row[1]-row[0])*1e-9 < .04)
    causal = CausalInertialPredictor()
    # Original source and receipt, no real LIO posterior in this pure contract
    # test. A buffered IMU packet is not itself a navigation-state observation.
    assert causal.push_imu(source*1e-9, ACC, ANGULAR, receipt*1e-9)
    before = tuple(causal.imu)
    assert causal.evaluate(receipt*1e-9) is None and causal.reason == 'waiting_lio'
    assert tuple(causal.imu) == before and causal.imu[-1][0] == source*1e-9
    assert not causal.push_imu(source*1e-9, ACC, ANGULAR, (receipt+10_000_000)*1e-9)
    assert not causal.push_imu((receipt+51_000_000)*1e-9, ACC, ANGULAR, receipt*1e-9)
