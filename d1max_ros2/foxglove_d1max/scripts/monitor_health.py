#!/usr/bin/env python3
"""Read-only telemetry freshness probe within the managed monitor cgroup."""
import json
import math
import os
from pathlib import Path
import time


def mc_health(report, wall, connected=True, replay=False):
    """MC readiness is separate from 1 Hz RobotState and SDK socket health."""
    result = {"mc_fresh": False, "mc_ready": False, "mc_observed_hz": 0., "speed_report": {}}
    try:
        if (not isinstance(report, dict) or report.get("source") != "sdk_mc" or
                not 0 <= wall - float(report.get("received_at_unix", 0)) <= 1.):
            return result
        hz = float(report.get("observed_hz", 0))
        if not math.isfinite(hz) or hz < 0:
            return result
        fresh = connected and not replay and report.get("stream_fresh") is True
        result.update(mc_fresh=fresh, mc_observed_hz=hz if fresh else 0.,
                      mc_ready=fresh and report.get("rate_ok") is True and
                      report.get("acknowledged") is True and report.get("ack_on") is True,
                      speed_report={key: report.get(key) for key in (
                          "source", "state", "stream_fresh", "observed_hz", "rate_ok", "acknowledged", "ack_on",
                          "attempts", "max_attempts", "total_attempts", "retry_cycles", "next_retry_sec", "write_error",
                          "samples", "timestamp_rejections", "connection_state", "last_sdk_error", "last_sdk_error_code", "ownership")})
    except (ValueError, TypeError, OverflowError):
        pass
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, CompressedImage
    from std_msgs.msg import String

    runtime = Path(os.environ["XDG_RUNTIME_DIR"]) / "d1max-session"
    runtime.mkdir(mode=0o700, parents=True, exist_ok=True)
    target = runtime / "monitor-health.json"
    invocation = os.environ.get("INVOCATION_ID", "")
    if not invocation:
        raise RuntimeError("Health probe must run inside the managed unit")
    rclpy.init()
    node = Node("d1max_monitor_health", enable_rosout=False, start_parameter_services=False)
    seen = {}
    sdk = {"received_at": 0, "connected": False, "replay": False}
    mc = {}

    def receive_robot(message):
        try:
            value = json.loads(message.data)
            sdk["received_at"] = float(value.get("received_at_unix", 0))
        except (ValueError, TypeError, AttributeError):
            sdk["received_at"] = 0

    def receive_connection(message):
        sdk["connected"] = message.data == "connected"
        seen["connection"] = time.monotonic()

    def receive_monitor(message):
        try:
            sdk["replay"] = json.loads(message.data).get("mode") == "replay"
        except (ValueError, AttributeError):
            sdk["replay"] = True

    def receive_mc(message):
        nonlocal mc
        try:
            mc = json.loads(message.data)
        except (ValueError, TypeError):
            mc = {}

    node.create_subscription(String, "/d1max_sdk_bridge/robot_state", receive_robot, 10)
    node.create_subscription(String, "/d1max_sdk_bridge/connection_state_text", receive_connection, 10)
    node.create_subscription(String, "/d1max/monitor/status", receive_monitor, 10)
    node.create_subscription(String, "/d1max_sdk_bridge/speed_report_status", receive_mc, 10)
    for side in ("front", "rear"):
        for kind, msg_type, topic in (("lidar", PointCloud2, f"/{side}_lidar"),
                                     ("image", CompressedImage, f"/{side}_camera/image_compressed")):
            key = side + "_" + kind
            node.create_subscription(msg_type, topic, lambda _msg, k=key: seen.update({k: time.monotonic()}), qos_profile_sensor_data)

    def save():
        now, wall = time.monotonic(), time.time()
        fresh = lambda key: key in seen and 0 <= now - seen[key] <= 3
        value = {"invocation": invocation, "wall_time": wall,
                 "sdk_fresh": sdk["connected"] and fresh("connection") and 0 <= wall - sdk["received_at"] <= 2.5,
                 "lidar_fresh": all(fresh(side + "_lidar") for side in ("front", "rear")),
                 "images_fresh": all(fresh(side + "_image") for side in ("front", "rear")),
                 "replay": sdk["replay"]}
        value.update(mc_health(mc, wall, value["sdk_fresh"], sdk["replay"]))
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(value), encoding="utf-8")
        os.replace(temporary, target)

    node.create_timer(.5, save)
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
