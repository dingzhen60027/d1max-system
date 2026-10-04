"""Source/receipt audit is read-only and keeps the replay's exact interval."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import pytest

from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu


spec=importlib.util.spec_from_file_location('recorded_input_timing',
    Path(__file__).resolve().parents[3]/'tools/validation/recorded_input_timing.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_optional_earlier_imu_does_not_shift_replay_window_or_mutate_database(tmp_path):
    database=tmp_path/'input.db3'
    names=('/front_lidar','/rear_lidar','/front_lidar/imu','/imu_driver/imu_central')
    with sqlite3.connect(database) as db:
        db.execute('CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for tid,name in enumerate(names,1):
            # Timing audit consumes only the ROS header, not cloud geometry.
            db.execute('INSERT INTO topics VALUES (?,?,?)',(tid,name,'sensor_msgs/msg/Imu'))
            ticks=(99.9,100.01,100.02) if tid==4 else (100.,100.1,100.4,101.2)
            for tick in ticks:
                message=Imu();message.header.stamp.sec=int(tick)
                message.header.stamp.nanosec=round((tick-int(tick))*1e9)
                db.execute('INSERT INTO messages (topic_id,timestamp,data) VALUES (?,?,?)',
                    (tid,round(tick*1e9),serialize_message(message)))
    (tmp_path/'metadata.yaml').write_text(json.dumps({'rosbag2_bagfile_information':{
        'relative_file_paths':['input.db3']}}))
    before=hashlib.sha256(database.read_bytes()).hexdigest()
    result=module.audit(tmp_path,1.,0)
    assert result['topics']['/front_lidar']['samples']==3
    assert result['topics']['/imu_driver/imu_central']['samples']==2
    assert result['topics']['/front_lidar/imu']['receipt_interval_ms']['maximum']==pytest.approx(300.)
    assert result['topics']['/rear_lidar/imu']=={'present':False}
    assert result['physical_clock_calibration'] is False
    assert result['topics']['/imu_driver/imu_central']['optional_imu_shared_clock_verified'] is False
    assert before==hashlib.sha256(database.read_bytes()).hexdigest()
