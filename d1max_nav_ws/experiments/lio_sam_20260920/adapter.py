#!/usr/bin/env python3
"""Isolated AIRY96 + central six-axis input experiment for ROS2 LIO-SAM.

No hardware attitude is claimed. Auxiliary orientation = confirmed low-motion
startup gravity alignment + gyro propagation; never reused as absolute tilt.
"""
import argparse
from array import array
from collections import Counter, deque
import copy
import json
from pathlib import Path
import signal

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

R_N_L = Rotation.from_quat([-.499867275, .503186620, .497953310, .498977388]).as_matrix()
OUTPUT_DTYPE = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time", "column", "sensor_range"],
                        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4", "<u2", "<f4"],
                        "offsets": [0, 4, 8, 12, 16, 20, 24, 28], "itemsize": 32})


def stamp_ns(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec


def rear_transform(cfg):
    ext = cfg["lidar_extrinsics"]["rear_to_front"]
    r_f_r = np.array(ext["rotation"], dtype=float).reshape(3, 3)
    t_f_r = np.array(ext["translation"], dtype=float)
    if (not np.isfinite(r_f_r).all() or not np.isfinite(t_f_r).all() or
            not np.allclose(r_f_r.T@r_f_r, np.eye(3), atol=1e-8) or
            abs(np.linalg.det(r_f_r)-1) > 1e-8):
        raise ValueError("Invalid recorded rear-to-front calibration")
    return R_N_L@r_f_r, R_N_L@t_f_r


class ScanPairer:
    """Consume each source scan once; never substitute a stale rear scan."""
    def __init__(self, counts, max_delta_ns=5_000_000):
        self.queues = {"front": deque(), "rear": deque()}
        self.latest = {}
        self.counts, self.max_delta_ns = counts, max_delta_ns

    def add(self, source, message):
        stamp = stamp_ns(message)
        if source in self.latest and stamp <= self.latest[source]:
            raise RuntimeError(f"Non-monotonic {source} LiDAR header")
        self.latest[source] = stamp
        self.queues[source].append(message)
        if len(self.queues[source]) > 8:
            self.queues[source].popleft()
            self.counts[source+"_unpaired"] += 1
        pairs = []
        front, rear = self.queues["front"], self.queues["rear"]
        while front and rear:
            delta = stamp_ns(front[0])-stamp_ns(rear[0])
            if abs(delta) <= self.max_delta_ns:
                pairs.append((front.popleft(), rear.popleft()))
            else:
                older = "front" if delta < 0 else "rear"
                self.queues[older].popleft()
                self.counts[older+"_unpaired"] += 1
        return pairs


def align_up(acc):
    unit = acc/np.linalg.norm(acc)
    cross = np.cross(unit, [0., 0., 1.])
    if np.linalg.norm(cross) < 1e-10:
        return Rotation.identity() if unit[2] > 0 else Rotation.from_rotvec([np.pi, 0, 0])
    angle = np.arctan2(np.linalg.norm(cross), unit[2])
    q = Rotation.from_rotvec(cross/np.linalg.norm(cross)*angle)
    roll, pitch, _ = q.as_euler("xyz")
    return Rotation.from_euler("xyz", [roll, pitch, 0.])


def convert_cloud(msg, rotation=R_N_L, translation=None, ring_offset=0, common_start_ns=None):
    expected = {"x": 7, "y": 7, "z": 7, "intensity": 7, "ring": 4, "timestamp": 8}
    fields = {f.name: f for f in msg.fields}
    if msg.is_bigendian or any(k not in fields or fields[k].datatype != d for k, d in expected.items()):
        raise ValueError("Unexpected AIRY point schema; refusing guessed conversion")
    dtype = np.dtype({"names": list(expected),
                      "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                      "offsets": [fields[k].offset for k in expected], "itemsize": msg.point_step})
    if msg.row_step != msg.width*msg.point_step:
        raise ValueError("Padded rows not supported by this audited input adapter")
    raw = np.frombuffer(msg.data, dtype=dtype, count=msg.width*msg.height)
    xyz = np.column_stack([raw[k] for k in ("x", "y", "z")])
    scan_sec = stamp_ns(msg)*1e-9
    native_rel = raw["timestamp"]-scan_sec
    common_start_ns = stamp_ns(msg) if common_start_ns is None else common_start_ns
    rel = raw["timestamp"]-common_start_ns*1e-9
    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(rel) & np.isfinite(raw["intensity"])
    ranges = np.linalg.norm(xyz, axis=1)
    valid = finite & (raw["ring"] < 96) & (native_rel >= -1e-6) & (native_rel < .15) & (rel >= -1e-6) & (rel < .16) & (ranges > .2)
    if valid.sum() < 1000:
        raise ValueError("Cloud has too few valid timed points")
    order = np.argsort(rel[valid], kind="stable")
    out = np.zeros(int(valid.sum()), dtype=OUTPUT_DTYPE)
    native_xyz = xyz[valid][order]
    points = native_xyz@np.asarray(rotation).T
    if translation is not None:
        points += np.asarray(translation)
    for j, k in enumerate(("x", "y", "z")):
        out[k] = points[:, j]
    out["ring"] = raw["ring"][valid][order]+ring_offset
    out["intensity"] = raw["intensity"][valid][order]
    out["time"] = np.maximum(0., rel[valid][order])
    # Match C++ round (half away from zero), keeping each physical optical origin.
    angular_bin = (np.degrees(np.arctan2(native_xyz[:, 0], native_xyz[:, 1]))-90.)/.4
    rounded = np.sign(angular_bin)*np.floor(np.abs(angular_bin)+.5)
    out["column"] = (-rounded.astype(np.int32)+450)%900
    out["sensor_range"] = ranges[valid][order]
    return out, {"input_points": len(raw), "output_points": len(out), "scan_duration_s": float(out["time"][-1])}


def merge_clouds(front, rear, r_n_r, t_n_r):
    start = min(stamp_ns(front), stamp_ns(rear))
    skew = abs(stamp_ns(front)-stamp_ns(rear))
    if skew > 5_000_000:
        raise ValueError("Refusing unmatched front/rear scans")
    f, fd = convert_cloud(front, common_start_ns=start)
    r, rd = convert_cloud(rear, r_n_r, t_n_r, 96, start)
    points = np.concatenate((f, r))
    points = points[np.argsort(points["time"], kind="stable")]
    return points, start, {"front": fd, "rear": rd, "header_skew_ns": skew,
                           "output_points": len(points), "scan_duration_s": float(points["time"][-1])}


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--lidar-mode", choices=["front", "dual"], default="dual")
    args = parser.parse_args()
    cfg = yaml.safe_load(args.calibration.read_text())
    r_n_c = np.array(cfg["lio_extrinsic"]["rotation"]).reshape(3, 3).T
    offset = round(float(cfg["timestamp_offset_sec"])*1e9)
    counts = Counter()
    r_n_r, t_n_r = rear_transform(cfg) if args.lidar_mode == "dual" else (None, None)
    pairer = ScanPairer(counts)
    report = {"mode": "six-axis startup-gravity plus gyro-propagated auxiliary orientation, NOT hardware AHRS",
              "calibration": str(args.calibration), "time_offset_ns": offset,
              "R_N_C": r_n_c.tolist(), "R_N_L": R_N_L.tolist(),
              "lidar_mode": args.lidar_mode, "paired_header_tolerance_ns": 5_000_000,
              "R_N_rear": r_n_r.tolist() if r_n_r is not None else None,
              "t_N_rear": t_n_r.tolist() if t_n_r is not None else None,
              "absolute_attitude_fusion": "Both LIO-SAM weights must be zero; only initialization and relative fallback guesses use orientation",
              "counts": counts}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("d1max_lio_sam_sixaxis_adapter")
    imu_group = MutuallyExclusiveCallbackGroup()
    cloud_group = MutuallyExclusiveCallbackGroup()
    qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
    pub_imu = node.create_publisher(Imu, "/lio_sam/input/imu_sixaxis", qos)
    pub_cloud = node.create_publisher(PointCloud2, "/lio_sam/input/points", QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
    buffer, q, bias, last_gyro, last_ns, init_ns = [], None, None, None, None, None
    pending_clouds = deque()

    def publish_imu(message, ns, acc, gyro):
        nonlocal q, last_gyro, last_ns
        gyro = gyro-bias
        if last_ns is not None:
            dt = (ns-last_ns)*1e-9
            if dt <= 0:
                raise RuntimeError("IMU timestamp did not increase")
            if dt > .15:
                raise RuntimeError("IMU gap above 150 ms: abort experiment")
            if dt > .03:
                counts["imu_gaps_over_30ms"] += 1
            q = q * Rotation.from_rotvec((last_gyro+gyro)*.5*dt)
        out = copy.deepcopy(message)
        ns += offset
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        out.header.frame_id = "lio_central_axes_n"
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = map(float, acc)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = map(float, gyro)
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = map(float, q.as_quat())
        # Unknown/integrated orientation uncertainty: not a measured AHRS covariance.
        out.orientation_covariance = [0.]*9
        pub_imu.publish(out)
        counts["imu_published"] += 1
        report.setdefault("first_output_imu_ns", ns)
        report["last_output_imu_ns"] = ns
        last_gyro, last_ns = gyro, ns-offset

    def on_imu(message):
        nonlocal q, bias, init_ns
        counts["imu_received"] += 1
        ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        acc = r_n_c@np.array([message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z])*cfg["acceleration_scale"]
        gyro = r_n_c@np.array([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])*cfg["gyro_scale"]
        if not np.isfinite(np.r_[acc, gyro]).all():
            raise RuntimeError("Non-finite IMU input")
        if q is None:
            buffer.append((message, ns, acc, gyro))
            if ns-buffer[0][1] < 2_000_000_000:
                return
            accelerations = np.array([x[2] for x in buffer])
            gyros = np.array([x[3] for x in buffer])
            mean = accelerations.mean(axis=0)
            if (len(buffer) < 300 or not 9.3 < np.linalg.norm(mean) < 10.3 or
                    np.max(accelerations.std(axis=0)) > .20 or np.max(gyros.std(axis=0)) > .02 or
                    np.linalg.norm(gyros.mean(axis=0)) > .03):
                raise RuntimeError("Startup interval failed low-motion initialization gate")
            q, bias, init_ns = align_up(mean), gyros.mean(axis=0), ns+offset
            report["initialization"] = {"sample_count": len(buffer), "end_corrected_ns": init_ns,
                                        "mean_acceleration_N": mean.tolist(), "gyro_bias_removed_N": bias.tolist(),
                                        "accel_axis_std": accelerations.std(axis=0).tolist(),
                                        "q_world_from_N_xyzw": q.as_quat().tolist(),
                                        "yaw": "arbitrary zero, no magnetic/GPS reference"}
            for item in buffer:
                publish_imu(*item)
            buffer.clear()
            print(json.dumps({"initialized": report["initialization"]}), flush=True)
        else:
            publish_imu(message, ns, acc, gyro)

    last_cloud_ns = None

    def drain_clouds():
        if last_ns is None:
            return
        watermark = last_ns+offset
        while pending_clouds:
            points, ns, detail = pending_clouds[0]
            end_ns = ns+round(float(points["time"][-1])*1e9)
            if watermark < end_ns+5_000_000:
                break
            pending_clouds.popleft()
            publish_cloud(points, ns, detail)

    def queue_cloud(points, ns, detail):
        pending_clouds.append((points, ns, detail))
        drain_clouds()
        if len(pending_clouds) > 12:
            raise RuntimeError("Central IMU did not cover pending LiDAR scans")

    def publish_cloud(points, ns, detail):
        nonlocal last_cloud_ns
        if last_cloud_ns is not None and ns <= last_cloud_ns:
            raise RuntimeError("Merged scan header did not increase")
        last_cloud_ns = ns
        out = PointCloud2()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        out.header.frame_id = "lio_lidar"
        size = OUTPUT_DTYPE.itemsize
        out.height, out.width, out.point_step = 1, len(points), size
        out.row_step, out.is_dense, out.is_bigendian = len(points)*size, True, False
        out.fields = [PointField(name=k, offset=OUTPUT_DTYPE.fields[k][1], datatype=(PointField.UINT16 if k in ("ring", "column") else PointField.FLOAT32), count=1) for k in OUTPUT_DTYPE.names]
        # Humble validates a generic bytes sequence byte-by-byte in Python
        # (~160 ms for a dual scan). The typed buffer takes its fast path.
        out.data = array("B", points.tobytes())
        pub_cloud.publish(out)
        counts["clouds_published"] += 1
        counts["front_points_published"] += int(np.count_nonzero(points["ring"] < 96))
        counts["rear_points_published"] += int(np.count_nonzero(points["ring"] >= 96))
        report.setdefault("first_cloud_ns", ns)
        report["last_cloud_ns"] = ns
        report["last_cloud_schema_check"] = detail

    def on_cloud(message, source="front"):
        expected_frame = "rslidar_head" if source == "front" else "rslidar_tail"
        if message.header.frame_id != expected_frame:
            raise ValueError(f"Expected raw {expected_frame}, got {message.header.frame_id}")
        counts["clouds_received"] += 1
        counts[source+"_clouds_received"] += 1
        ns = stamp_ns(message)
        if init_ns is None or ns < init_ns:
            counts["clouds_skipped_startup"] += 1
            counts[source+"_clouds_skipped_startup"] += 1
            return
        if args.lidar_mode == "front":
            points, detail = convert_cloud(message)
            queue_cloud(points, ns, detail)
        else:
            for front, rear in pairer.add(source, message):
                points, start, detail = merge_clouds(front, rear, r_n_r, t_n_r)
                counts["paired_scans"] += 1
                report["max_pair_skew_ns"] = max(report.get("max_pair_skew_ns", 0), detail["header_skew_ns"])
                queue_cloud(points, start, detail)

    def save_audit():
        report["pending_unpaired"] = {source: len(queue) for source, queue in pairer.queues.items()}
        report["pending_imu_coverage"] = len(pending_clouds)
        snapshot = dict(report)
        snapshot["counts"] = dict(counts)
        args.audit.write_text(json.dumps(snapshot, indent=2)+"\n")

    subscriptions = [node.create_subscription(Imu, "/imu_driver/imu_central", on_imu, qos, callback_group=imu_group),
                     node.create_subscription(PointCloud2, "/front_lidar", on_cloud, QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE), callback_group=cloud_group)]
    if args.lidar_mode == "dual":
        subscriptions.append(node.create_subscription(PointCloud2, "/rear_lidar", lambda msg: on_cloud(msg, "rear"), QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE), callback_group=cloud_group))
    timer = node.create_timer(3., save_audit, callback_group=cloud_group)
    coverage_timer = node.create_timer(.01, drain_clouds, callback_group=cloud_group)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        save_audit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
