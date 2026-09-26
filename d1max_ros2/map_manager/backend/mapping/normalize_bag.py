"""Read-only source bag -> task-owned SI copy; never connects to the ROS graph."""
import json
from pathlib import Path
import signal
import sys
import time
import yaml


def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from sensor_msgs.msg import Imu
    task = Path(sys.argv[1]).resolve()
    config = yaml.safe_load(task.read_text())
    value = config['input']; source = Path(value['bag_path']).resolve()
    target = task.parent / 'input_si'
    if target.exists() or target == source:
        raise RuntimeError('Refusing to overwrite a prepared or original bag')
    metadata = yaml.safe_load((source / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    selected = {value['lidar_topic'], value['imu_topic'], '/tf', '/tf_static'}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source),storage_id=metadata['storage_identifier']),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected)))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(target),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    for topic in reader.get_all_topics_and_types():
        if topic.name in selected:
            writer.create_topic(topic)
    counts = {}; last_report = time.monotonic(); scale = float(value['imu_acceleration_scale'])
    def cancel(_s,_f):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,cancel); signal.signal(signal.SIGTERM,cancel)
    try:
        while reader.has_next():
            topic, raw, timestamp = reader.read_next()
            if topic == value['imu_topic']:
                msg = deserialize_message(raw, Imu)
                for key in ('x','y','z'):
                    setattr(msg.linear_acceleration,key,getattr(msg.linear_acceleration,key)*scale)
                if msg.linear_acceleration_covariance[0] != -1:
                    msg.linear_acceleration_covariance = [x*scale*scale for x in msg.linear_acceleration_covariance]
                raw = serialize_message(msg)
            writer.write(topic,raw,timestamp)
            counts[topic] = counts.get(topic,0)+1
            if time.monotonic()-last_report >= 5:
                print(json.dumps({'stage':'normalize_input','messages':counts},ensure_ascii=False),flush=True)
                last_report=time.monotonic()
    finally:
        del writer
    (task.parent / 'input_conversion.json').write_text(json.dumps({
        'source':str(source),'prepared':str(target),'imu_acceleration_scale':scale,
        'counts':counts,'source_modified':False,'timestamps':'unchanged','complete':True},indent=2))
    print('SI input copy complete: '+str(target),flush=True)


if __name__ == '__main__':
    main()
