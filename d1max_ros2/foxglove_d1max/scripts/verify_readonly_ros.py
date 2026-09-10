"""Read-only integration check: no SDK calls or application-topic publications."""
import json
import argparse
import time
import rclpy
from std_msgs.msg import String
parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['read_only', 'replay'], default='read_only')
args = parser.parse_args()
rclpy.init(args=[])
node = rclpy.create_node('d1max_console_readonly_check', enable_rosout=False, start_parameter_services=False)
samples = []
subscription = node.create_subscription(String, '/d1max/console/status', lambda m: samples.append(json.loads(m.data)), 10)
deadline = time.monotonic() + 6
try:
    while time.monotonic() < deadline and len(samples) < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    assert len(samples) >= 3, 'No gateway heartbeat'
    assert all(s['mode'] == args.mode and s['armed'] is False and s['can_arm'] is False for s in samples)
    services = node.get_service_names_and_types()
    assert not any(name.startswith('/d1max/console/s_') for name, _ in services), 'Unexpected live control service'
    assert not any(p.node_name == 'd1max_console_gateway' for p in node.get_publishers_info_by_topic('/cmd_vel'))
    print(f'PASS: 3 ROS heartbeats; {args.mode}; not armed; no control services; no gateway /cmd_vel publisher.')
finally:
    node.destroy_node()
    rclpy.shutdown()
