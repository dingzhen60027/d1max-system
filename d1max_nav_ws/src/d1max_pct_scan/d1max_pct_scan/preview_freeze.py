"""Preview runtime heartbeat. True freezes SCAN's clock; it is NOT robot control."""
import argparse
import os
from pathlib import Path

from .live_view_reload import preview_reload_session, read_owned_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True, type=Path)
    args = parser.parse_args()
    session = read_owned_json(args.session / 'session.json')
    preview_reload_session(session)  # Reject motion and legacy/ambiguous ownership.
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_zenoh_cpp':
        raise ValueError('Preview heartbeat requires the unchanged Zenoh graph')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool
    rclpy.init()
    node = Node('d1max_preview_freeze')
    publisher = node.create_publisher(Bool, '/d1max/live_planning/execution_frozen', 10)
    node.create_timer(.2, lambda: publisher.publish(Bool(data=True)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
