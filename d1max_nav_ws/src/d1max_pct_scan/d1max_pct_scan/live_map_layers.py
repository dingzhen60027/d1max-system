"""Static read-only terrain publisher; no localization or planning callbacks."""
import argparse
from array import array
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from .live_map_geometry import build_layers, colored_records


class LiveMapLayers(Node):
    def __init__(self, directory):
        super().__init__('d1max_live_map_layers')
        session = json.loads((directory/'session.json').read_text())
        xyz, costs, blocked, metadata = build_layers(session)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.messages, self.layer_publishers = [], []
        for name, points, values in [('traversable_surface', xyz, costs), ('blocked_surface', blocked, None)]:
            records = colored_records(points, values)
            message = PointCloud2(header=Header(frame_id=session['frame_id']), height=1,
                width=len(records), fields=[PointField(name=n, offset=i*4,
                    datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z', 'rgb'))],
                point_step=16, row_step=16*len(records), is_dense=True, is_bigendian=False)
            message.data = array('B', records.tobytes())
            self.messages.append(message)
            self.layer_publishers.append(self.create_publisher(PointCloud2, '/d1max/live_planning/'+name, qos))
        self.metadata = self.create_publisher(String, '/d1max/live_planning/map_layer_status', qos)
        self.metadata.publish(String(data=json.dumps({**metadata, 'session_id': session['id']})))
        self.publish_layers()
        self.create_timer(5., self.publish_layers)
        self.get_logger().info(json.dumps(metadata))

    def publish_layers(self):
        for publisher, message in zip(self.layer_publishers, self.messages):
            message.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = LiveMapLayers(args.session)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RCLError:
            if rclpy.ok():
                raise


if __name__ == '__main__':
    main()
