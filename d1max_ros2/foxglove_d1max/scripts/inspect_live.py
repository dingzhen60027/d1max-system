"""Read-only ROS discovery/sensor verification. No publisher, service or action clients."""
import json
import time
from collections import defaultdict
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, CompressedImage, Imu, JointState
from tf2_msgs.msg import TFMessage

rclpy.init()
node = rclpy.create_node('d1max_live_inspect', enable_rosout=False, start_parameter_services=False)
stats = defaultdict(lambda: {'count': 0})
frames = {}
start = time.monotonic()
subscriptions = []

def receive(topic, msg):
    item = stats[topic]
    item['count'] += 1
    if hasattr(msg, 'header'):
        item['frame'] = msg.header.frame_id
        item['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    if isinstance(msg, PointCloud2):
        item['points'] = msg.width * msg.height
    elif isinstance(msg, CompressedImage):
        item.update(format=msg.format, bytes=len(msg.data), magic=bytes(msg.data[:12]).hex())
    elif isinstance(msg, JointState):
        item['joint_names'] = list(msg.name)
    elif isinstance(msg, TFMessage):
        for t in msg.transforms:
            frames[t.child_frame_id] = {
                'parent': t.header.frame_id,
                'xyz': [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                'xyzw': [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w],
                'stamp': t.header.stamp.sec + t.header.stamp.nanosec / 1e9,
            }

for topic, cls in [
    ('/front_lidar', PointCloud2), ('/rear_lidar', PointCloud2),
    ('/front_lidar/imu', Imu), ('/rear_lidar/imu', Imu),
    ('/front_camera/image_compressed', CompressedImage), ('/rear_camera/image_compressed', CompressedImage),
    ('/joint_states', JointState), ('/tf', TFMessage), ('/tf_static', TFMessage),
]:
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == '/tf_static' else DurabilityPolicy.VOLATILE)
    subscriptions.append(node.create_subscription(cls, topic, lambda msg, t=topic: receive(t, msg), qos))

try:
    while time.monotonic() - start < 6:
        rclpy.spin_once(node, timeout_sec=.1)
    elapsed = time.monotonic() - start
    for item in stats.values():
        item['received_hz'] = round(item['count'] / elapsed, 2)
    report = {'wall_time': time.time(), 'duration': elapsed, 'topics': dict(stats), 'frames': frames,
              'joint_publishers': node.count_publishers('/joint_states')}
    output = Path(__file__).resolve().parents[1] / 'artifacts' / 'live-inspection.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
