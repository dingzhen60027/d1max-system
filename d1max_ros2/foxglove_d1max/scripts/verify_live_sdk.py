"""Read-only live SDK verification; does not create a robot command client."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('d1max_verify_sdk_observer', enable_rosout=False, start_parameter_services=False)
states, connections = [], []
node.create_subscription(String, '/d1max_sdk_bridge/robot_state', lambda m: states.append(json.loads(m.data)), 10)
node.create_subscription(String, '/d1max_sdk_bridge/connection_state_text', lambda m: connections.append(m.data), 10)
start = time.monotonic()
try:
    while time.monotonic() - start < 7:
        rclpy.spin_once(node, timeout_sec=.1)
    services = node.get_service_names_and_types_by_node('d1max_sdk_telemetry', '/')
    subscribers = node.get_subscriber_names_and_types_by_node('d1max_sdk_telemetry', '/')
    publishers = node.get_publisher_names_and_types_by_node('d1max_sdk_telemetry', '/')
    assert len(states) >= 3, 'No fresh automatic RobotState stream'
    assert all(s['read_only'] is True and s['source'] == 'sdk_passive_robot_state' for s in states)
    assert states[-1]['received_at_unix'] > states[0]['received_at_unix']
    assert abs(time.time() - states[-1]['received_at_unix']) < 3
    assert connections[-1] == 'connected'
    assert not services, f'Unexpected observer services: {services}'
    # rclcpp internally subscribes to parameter events; there is no command path.
    assert all(name == '/parameter_events' for name, _ in subscribers), subscribers
    assert {name for name, _ in publishers} == {
        '/d1max_sdk_bridge/robot_state', '/d1max_sdk_bridge/faults',
        '/d1max_sdk_bridge/connection_state_text', '/d1max_sdk_bridge/behavior_state'}
    report = dict(time=time.time(), duration=time.monotonic()-start, samples=len(states),
                  last_state=states[-1], services=services, subscribers=subscribers,
                  publishers=publishers, connection=connections[-1])
    (Path(__file__).resolve().parents[1] / 'artifacts/live-sdk-checks.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
