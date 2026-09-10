#!/usr/bin/env python3
"""Read-only PCD -> PointCloud2 visualization; never publishes TF or navigation goals."""
import argparse
import json
import math
from pathlib import Path
import re
import numpy as np
import open3d as o3d
import yaml

def load_maps(config_path):
    config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    period = float(config.get("publish_period_seconds", 10))
    if not math.isfinite(period) or not 2 <= period <= 60:
        raise ValueError("publish_period_seconds must be 2..60")
    if config.get("enabled", True) is False:
        return period, []  # Do not even open historical PCD files when disabled.
    maps = []
    ids = set()
    for item in config["maps"]:
        ident = str(item["id"])
        frame = str(item["frame_id"])
        if not re.fullmatch(r"[a-z][a-z0-9_]*", ident) or ident in ids:
            raise ValueError("Invalid or duplicate map id")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", frame):
            raise ValueError("Invalid display frame")
        ids.add(ident)
        path = Path(item["path"]).expanduser().resolve(strict=True)
        if path.suffix.lower() != ".pcd":
            raise ValueError("Only PCD input is supported")
        voxel = float(item.get("voxel_size", 0))
        if not math.isfinite(voxel) or not 0 <= voxel <= 1:
            raise ValueError("voxel_size must be 0..1 metre")
        cloud = o3d.io.read_point_cloud(str(path))
        xyz = np.asarray(cloud.points)
        if not len(xyz):
            raise ValueError("Empty or unreadable PCD: " + str(path))
        source_count = len(xyz)
        cloud = cloud.remove_non_finite_points()
        if voxel > 0:
            cloud = cloud.voxel_down_sample(voxel)
        xyz = np.asarray(cloud.points, dtype="<f4")
        if not len(xyz) or not np.isfinite(xyz).all():
            raise ValueError("PCD has no finite display points")
        metadata = dict(id=ident, label=str(item.get("label", ident)), path=str(path),
            frame_id=frame, topic=f"/d1max/maps/{ident}/points", source_points=source_count,
            display_points=len(xyz), voxel_size=voxel, error="", localized=False,
            min=xyz.min(axis=0).tolist(), max=xyz.max(axis=0).tolist())
        maps.append((metadata, xyz.tobytes()))
    if not maps:
        raise ValueError("No maps configured")
    return period, maps

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(Path(__file__).resolve().parents[1]/"config/map-view.yaml"))
    parser.add_argument("--inspect", action="store_true", help="Parse files only; no ROS connection")
    parser.add_argument("--is-enabled", action="store_true", help="Check configuration only; exit 1 when disabled")
    args = parser.parse_args()
    if args.is_enabled:
        config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
        raise SystemExit(1 if config.get("enabled", True) is False else 0)
    period, maps = load_maps(args.config)
    if args.inspect:
        print(json.dumps({"enabled":bool(maps), "maps":[meta for meta,_ in maps]}, ensure_ascii=False, indent=2))
        return
    if not maps:
        print("Static PCD loading disabled; no files loaded and no ROS node started.", flush=True)
        return
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    node_name = config.get("node_name", "d1max_pcd_map_view")
    status_topic = config.get("status_topic", "/d1max/maps/status")
    if not re.fullmatch(r"[a-z][a-z0-9_]*", node_name):
        raise ValueError("Invalid map publisher node_name")
    if not re.fullmatch(r"/d1max/maps/(?:[a-z][a-z0-9_]*/)?status", status_topic):
        raise ValueError("Invalid map status_topic")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
    rclpy.init()
    node = Node(node_name, enable_rosout=False, start_parameter_services=False)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    output = []
    for metadata, data in maps:
        msg = PointCloud2()
        msg.header.frame_id = metadata["frame_id"]
        msg.height, msg.width = 1, metadata["display_points"]
        msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1) for i,name in enumerate(("x","y","z"))]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, 12*msg.width
        msg.data = data
        output.append((node.create_publisher(PointCloud2, metadata["topic"], qos), msg))
    status = node.create_publisher(String, status_topic, 1)
    def publish():
        for publisher, message in output:
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
    def publish_status():
        status.publish(String(data=json.dumps({"maps":[meta for meta,_ in maps], "localized":False}, ensure_ascii=False)))
    node.create_timer(period, publish)
    node.create_timer(1, publish_status)
    publish()
    print(json.dumps({"maps":[meta for meta,_ in maps]}, ensure_ascii=False), flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == "__main__":
    main()
