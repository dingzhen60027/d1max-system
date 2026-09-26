"""Small read-only rosbag sample, no rclpy.init(), node, publisher or SDK."""
import json
import math
import struct
from pathlib import Path
import sys
import yaml


def check_frame_contract(config, lidar_frame, imu_frame):
    # D1 Airy firmware labels physically DIFFERENT point/IMU axes identically.
    # A same-name TF lookup silently returns identity and is unsafe for this input.
    if (config['input']['lidar_topic'] in {'/front_lidar', '/rear_lidar'}
            and lidar_frame == imu_frame and lidar_frame in {'rslidar_head', 'rslidar_tail'}
            and config['sensors']['source'] != 'fixed'):
        raise ValueError('D1 Airy 点云/IMU 虽然 frame_id 相同，但物理坐标不同；必须分别配置固定外参，不能按单位变换建图。请使用 D1 903 标定配置。')


def inspect(config):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, Imu
    value = config['input']
    path = Path(value['bag_path'])
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[value['lidar_topic'], value['imu_topic']]))
    samples = {}
    for _ in range(10000):
        if not reader.has_next():
            break
        topic, data, _stamp = reader.read_next()
        if topic not in samples:
            samples[topic] = deserialize_message(data, PointCloud2 if topic == value['lidar_topic'] else Imu)
        if len(samples) == 2:
            break
    if len(samples) != 2:
        raise ValueError('无法读取雷达和 IMU 样本')
    cloud, imu = samples[value['lidar_topic']], samples[value['imu_topic']]
    check_frame_contract(config, cloud.header.frame_id, imu.header.frame_id)
    fields = [f.name for f in cloud.fields]
    if not {'x', 'y', 'z'}.issubset(fields) or not set(fields).intersection({'time', 'timestamp', 't'}):
        raise ValueError('点云缺少 XYZ 或逐点时间，拒绝静默关闭去畸变')
    count = cloud.width * cloud.height
    if count < 10 or cloud.point_step <= 0 or len(cloud.data) < cloud.row_step * cloud.height:
        raise ValueError('点云样本为空、点数不足或数据长度不完整')
    field = next(f for f in cloud.fields if f.name in {'time', 'timestamp', 't'})
    # Native MRPT accepts float seconds or unsigned integer nanoseconds.
    formats = {6: ('I', 1e-9), 7: ('f', 1.), 8: ('d', 1.)}
    if field.datatype not in formats or field.count != 1:
        raise ValueError('逐点时间类型不支持；需要 UINT32 纳秒或 FLOAT32/FLOAT64 秒')
    kind, scale = formats[field.datatype]
    fmt = ('>' if cloud.is_bigendian else '<') + kind
    if field.offset < 0 or field.offset + struct.calcsize(fmt) > cloud.point_step:
        raise ValueError('逐点时间字段超出 point_step')
    stamps = []
    for i in range(min(count, 64)):
        index = i * (count - 1) // (min(count, 64) - 1)
        row, column = divmod(index, cloud.width)
        offset = row * cloud.row_step + column * cloud.point_step + field.offset
        stamps.append(struct.unpack_from(fmt, cloud.data, offset)[0] * scale)
    if not all(math.isfinite(t) for t in stamps) or not 0 < max(stamps) - min(stamps) <= 2:
        raise ValueError('逐点时间不变化、非有限值或跨度超过 2 秒，请核对时间单位')
    inertial = [getattr(v, axis) for v in (imu.angular_velocity, imu.linear_acceleration) for axis in ('x', 'y', 'z')]
    if not all(math.isfinite(v) for v in inertial):
        raise ValueError('IMU 含非有限值')
    if imu.angular_velocity_covariance[0] == -1 or imu.linear_acceleration_covariance[0] == -1:
        raise ValueError('IMU 标记角速度或加速度不可用')
    lidar_time = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
    imu_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
    if abs(lidar_time - imu_time) > 2:
        raise ValueError('雷达/IMU 起始时间相差超过 2 秒，请核对时间轴')
    if config['sensors']['source'] == 'bag_tf':
        sensor_frames = {cloud.header.frame_id, imu.header.frame_id}
        if '' in sensor_frames:
            raise ValueError('传感器 frame_id 为空，无法建立外参关系')
        if sensor_frames != {value['base_frame']}:
            topics = {t['topic_metadata']['name'] for t in metadata['topics_with_message_count']}
            if not topics.intersection({'/tf', '/tf_static'}):
                raise ValueError('传感器坐标系不同且 bag 没有 TF；请配置真实固定外参')
    return {'lidar_frame': cloud.header.frame_id, 'imu_frame': imu.header.frame_id,
            'fields': fields, 'lidar_stamp': lidar_time, 'imu_stamp': imu_time,
            'point_time_span_sec': max(stamps) - min(stamps),
            'note': '样本检查通过；完整时间覆盖与 TF 由原生读取器继续检查，不等于标定已验收'}


if __name__ == '__main__':
    print(json.dumps(inspect(yaml.safe_load(Path(sys.argv[1]).read_text())), ensure_ascii=False))
