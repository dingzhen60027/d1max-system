"""Deterministic observer math checks; no ROS graph or robot connection."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location("observe_replay", Path(__file__).with_name("observe_replay.py"))
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


def test_percentiles_and_nonfinite():
    assert observer.percentile([1, 2, 3, 4, float("nan")], .5) == 2.5
    assert observer.distribution([]) == {"n": 0, "median": None, "p95": None, "max": None}


@pytest.mark.parametrize("value,expected", [(b"\x00", 0), (b"\x01", 1), (b"\x02", 2), (b"\x03", 3),
                                          (bytearray(b"\xff"), 255), (memoryview(b"\x02"), 2), (0, 0), (1, 1), (255, 255)])
def test_ros_byte_and_integer_conversion(value, expected):
    assert observer.uint8_value(value) == expected


@pytest.mark.parametrize("value", [b"", b"\x00\x01", -1, 256, 1.5, "1"])
def test_ros_byte_invalid_rejected(value):
    with pytest.raises((TypeError, ValueError)):
        observer.uint8_value(value)


def test_real_humble_diagnostic_serialization():
    diagnostic_msgs = pytest.importorskip("diagnostic_msgs.msg")
    serialization = pytest.importorskip("rclpy.serialization")
    for code in range(4):
        status = diagnostic_msgs.DiagnosticStatus(level=bytes([code]), name="d1max_localization/matcher",
            message="quality sample", values=[diagnostic_msgs.KeyValue(key="inlier_ratio", value="0.8")])
        received = serialization.deserialize_message(serialization.serialize_message(status), diagnostic_msgs.DiagnosticStatus)
        row = observer.diagnostic_record(received, 123.)
        assert row["level"] == code
        assert row["values"]["inlier_ratio"] == "0.8"


def test_status_time_weighted_and_stale_is_unavailable(tmp_path):
    recorder = observer.Recorder(tmp_path, status_timeout=.5)
    try:
        recorder.statuses = [(0., {"state": "acquiring", "localized": False}),
                             (1., {"state": "tracking", "localized": True}),
                             (1.2, {"state": "tracking", "localized": True}),
                             (2., {"state": "lost", "localized": False})]
        result = recorder.status_summary(3.)
        assert result["first_localized_elapsed_s"] == 1.
        assert result["after_first_lock_s"] == 2.
        assert result["time_weighted_localized_fraction_after_first_lock"] == pytest.approx(.35)
        assert result["after_first_lock_state_s"]["status_stale"] == pytest.approx(.8)
        assert sum(result["time_by_state_s"].values()) == pytest.approx(3.)
    finally:
        recorder.close()


def test_no_lock_is_not_zero_accuracy(tmp_path):
    recorder = observer.Recorder(tmp_path)
    try:
        result = recorder.summary(3.)
        assert result["localization"]["time_weighted_localized_fraction_after_first_lock"] is None
        assert result["ground_truth_available"] is False
        assert result["odometry"]["global"]["active_received_hz"] is None
    finally:
        recorder.close()


def odom(x, stamp, velocity=.01):
    return {"position": [x, 0., 0.], "orientation": [0., 0., 0., 1.],
            "linear": [velocity, 0., 0.], "angular": [0., 0., 0.],
            "frame": "map", "child_frame": "body", "stamp": stamp}


def test_low_motion_proxy_and_source_rates(tmp_path):
    recorder = observer.Recorder(tmp_path, low_motion_seconds=1.)
    try:
        rows = [(index / 10., odom(index / 1000., index / 10.)) for index in range(21)]
        recorder.odom = {"local": rows, "global": rows}
        candidate = recorder.low_motion_intervals()["intervals"][0]
        assert candidate["duration_s"] == 2.
        assert candidate["local_endpoint_displacement_m"] == .02
        result = recorder.odom_summary("local", 3.)
        assert result["active_received_hz"] == 10.
        assert result["trajectory_length_m_including_corrections_and_gaps"] == pytest.approx(.02)
    finally:
        recorder.close()


def test_initial_zero_epoch_not_counted_as_reset(tmp_path):
    recorder = observer.Recorder(tmp_path)
    try:
        recorder.frontends = [(0., {"epoch": 0}), (1., {"epoch": 1}),
                             (2., {"epoch": 1}), (3., {"epoch": 2, "fault": "imu_gap"})]
        result = recorder.summary(4.)
        assert result["frontend"]["resets_after_first_epoch"] == 1
        assert len(result["frontend"]["fault_events"]) == 1
    finally:
        recorder.close()


def matcher_record(message="匹配已连续确认", **values):
    fields = {"attempts": "1", "source_points": "1000", "elapsed_ms": "15.0",
              "inlier_ratio": "0.8", "inlier_rmse_m": "0.15", "information_ratio": "0.002"}
    fields.update(values)
    return {"name": "d1max_localization/matcher", "message": message, "values": fields}


@pytest.mark.parametrize("record", [
    matcher_record(attempts="0"), matcher_record(source_points="0"),
    matcher_record(source_points=None), matcher_record(elapsed_ms="0"),
    matcher_record("等待初始位姿", attempts="0", source_points="0", inlier_ratio="0.0", inlier_rmse_m="0.0"),
    matcher_record("匹配计算中"), matcher_record("GICP 未收敛"),
    matcher_record("几何退化：拒绝不可靠校正"), matcher_record("匹配的高度或旋转超出门限"),
])
def test_uncompleted_quality_excluded_but_reason_preserved(tmp_path, record):
    recorder = observer.Recorder(tmp_path)
    try:
        recorder.matcher = [(0., record)]
        result = recorder.summary(1.)["matcher"]
        assert result["quality"]["inlier_rmse_m"] == {"n": 0, "median": None, "p95": None, "max": None}
        assert result["quality_evaluated_results"] == 0
        assert result["quality_excluded_results"] == 1
        assert result["reason_observations"] == {record["message"]: 1}
    finally:
        recorder.close()


def test_completed_quality_and_rejected_quality_both_reported(tmp_path):
    recorder = observer.Recorder(tmp_path)
    try:
        recorder.matcher = [(0., matcher_record()),
            (1., matcher_record("点云有效重叠不足或残差过大", attempts="2", inlier_ratio="0.1", inlier_rmse_m="0.4")),
            (2., matcher_record("GICP 未收敛", attempts="3", inlier_ratio="0.1", inlier_rmse_m="0.4"))]
        result = recorder.summary(3.)["matcher"]
        assert result["quality_evaluated_results"] == 2
        assert result["quality"]["inlier_rmse_m"]["n"] == 2
        assert result["quality"]["inlier_rmse_m"]["median"] == pytest.approx(.275)
        assert result["quality_excluded_results"] == 1
        assert len(result["reason_observations"]) == 3
    finally:
        recorder.close()
