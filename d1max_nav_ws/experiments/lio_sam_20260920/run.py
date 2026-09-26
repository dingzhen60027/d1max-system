#!/usr/bin/env python3
"""Owned-process, local-Zenoh offline LIO-SAM experiment supervisor."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import time

import numpy as np
import yaml
from adapter import R_N_L

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
ROBOT = Path("/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2")


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+"\n")


def make_parameters(calibration, loop, lidar_mode="dual", rate=1.):
    base = yaml.safe_load((HERE/"ws/src/LIO-SAM/config/params.yaml").read_text())
    cfg = base["/**"]["ros__parameters"]
    r_c_n = np.array(calibration["lio_extrinsic"]["rotation"]).reshape(3, 3)
    t_c_n = np.array(calibration["lio_extrinsic"]["translation"])
    cfg.update(pointCloudTopic="/lio_sam/input/points", imuTopic="/lio_sam/input/imu_sixaxis",
               odomTopic="/lio_sam/odometry/imu", gpsTopic="/lio_sam/unused_gps",
               lidarFrame="lio_lidar", baselinkFrame="lio_lidar", odometryFrame="odom", mapFrame="odom",
               sensor="velodyne", N_SCAN=192 if lidar_mode == "dual" else 96, Horizon_SCAN=900, downsampleRate=1,
               lidarMinRange=1., lidarMaxRange=80., imuGravity=9.80665,
               imuRPYWeight=0., imuIncrementalRPYWeight=0., useImuHeadingInitialization=False,
               extrinsicRot=np.eye(3).ravel().tolist(), extrinsicRPY=np.eye(3).ravel().tolist(),
               extrinsicTrans=(-r_c_n.T@t_c_n).tolist(), projectionRot=R_N_L.T.ravel().tolist(),
               useProvidedProjection=True,
               odometrySurfLeafSize=.2, mappingCornerLeafSize=.1, mappingSurfLeafSize=.2,
               mappingProcessInterval=.05, numberOfCores=4,
               surroundingkeyframeAddingDistThreshold=.7, surroundingkeyframeAddingAngleThreshold=.15,
               surroundingKeyframeDensity=1., loopClosureEnableFlag=loop, loopClosureFrequency=rate,
               globalMapVisualizationPoseDensity=.5, globalMapVisualizationLeafSize=.1,
               globalMapVisualizationSearchRadius=1000., savePCD=False,
               use_sim_time=False)
    return base


def validate_full_input(bag, audit, observed, lidar_mode):
    """A completed player is insufficient: require every recorded input message."""
    metadata = yaml.safe_load((bag/"metadata.yaml").read_text())["rosbag2_bagfile_information"]
    expected = {item["topic_metadata"]["name"]: item["message_count"] for item in metadata["topics_with_message_count"]}
    counts = audit["counts"]
    checks = {"all_central_imu_received": counts.get("imu_received") == expected["/imu_driver/imu_central"],
              "all_central_imu_published": counts.get("imu_published") == expected["/imu_driver/imu_central"],
              "all_front_scans_received": counts.get("front_clouds_received") == expected["/front_lidar"],
              "imu_coverage_queue_drained": audit.get("pending_imu_coverage") == 0,
              "projected_frames_preserved": observed["deskewed_scans"] >= counts.get("clouds_published", 0)-3,
              "mapping_frames_preserved": observed["odometry"] >= observed["deskewed_scans"]-1}
    if lidar_mode == "dual":
        checks.update(all_rear_scans_received=counts.get("rear_clouds_received") == expected["/rear_lidar"],
                      both_scanners_published=counts.get("front_points_published", 0) > 0 and counts.get("rear_points_published", 0) > 0,
                      every_paired_scan_published=counts.get("paired_scans") == counts.get("clouds_published"))
        baseline = json.loads((HERE/"bag_pair_audit.json").read_text())
        valid_baseline = (Path(baseline["bag"]).resolve() == bag.resolve()
                          and baseline["metadata_sha256"] == hashlib.sha256((bag/"metadata.yaml").read_bytes()).hexdigest()
                          and baseline["init_cutoff_ns"] == audit["initialization"]["end_corrected_ns"])
        checks["pairing_baseline_matches_bag_and_startup"] = valid_baseline
        checks["paired_scans_match_recorded_headers"] = valid_baseline and counts.get("paired_scans") == baseline["full_bag"]["paired_scans"]
        for source in ("front", "rear"):
            checks[source+"_unpaired_match_source"] = counts.get(source+"_unpaired", 0) == baseline["full_bag"]["counts"].get(source+"_unpaired", 0)
        checks["unpaired_queue_matches_source"] = audit.get("pending_unpaired") == baseline["full_bag"]["pending_unpaired"]
    return {"passed": all(checks.values()), "checks": checks,
            "expected_bag_topic_counts": expected, "received_counts": counts,
            "pairing_note": "5 ms header gate rejects unmatched raw scans; unpaired counts remain explicit, not claimed as zero."}


def main():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from lio_sam.msg import CloudInfo
    from lio_sam.srv import SaveMap
    from visualization_msgs.msg import MarkerArray

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60., help="Input sensor seconds; 0=full bag")
    parser.add_argument("--rate", type=float, default=1.)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--hold", action="store_true", help="After completion keep saved map and RViz available, no SLAM/replay")
    parser.add_argument("--label", default="test")
    parser.add_argument("--lidar-mode", choices=["front", "dual"], default="dual")
    parser.add_argument("--publish-web", action="store_true", help="Register only a completed full-bag global map in the Web library")
    parser.add_argument("--debug-imu", action="store_true", help="Capture an IMU preintegration crash backtrace")
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args()
    if args.calibration is None:
        name = "device_front_plus_runtime_rear.yaml" if args.lidar_mode == "dual" else "device_front_rotation.yaml"
        args.calibration = WS/"experiments/lio_frontend_reliability_20260919/extrinsic_trials_20260919/calibrations"/name
    if not 0 < args.rate <= 2 or args.duration < 0:
        raise ValueError("Invalid playback rate/duration")
    if args.publish_web and args.duration:
        raise ValueError("Web publication requires a full bag, not a diagnostic fragment")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_zenoh_cpp" or os.environ.get("ROS_DOMAIN_ID") != "220":
        raise RuntimeError("Use run.sh: isolated Zenoh/domain 220 required")
    lock = (HERE/"runtime.lock").open("a+")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    with socket.socket() as sock:
        if sock.connect_ex(("127.0.0.1", 7448)) == 0:
            raise RuntimeError("Port 7448 is occupied; existing process left untouched")
    bag = WS/"bags/slam_raw_20260917_171716_fe8f38"
    label = "".join(c for c in args.label if c.isalnum() or c in "-_")[:60]
    out = HERE/"runs"/(time.strftime("%Y%m%d_%H%M%S")+"_"+label)
    out.mkdir(parents=True, exist_ok=False)
    parameters = make_parameters(yaml.safe_load(args.calibration.read_text()), args.loop, args.lidar_mode, args.rate)
    (out/"params.yaml").write_text(yaml.safe_dump(parameters, sort_keys=False))
    children, logs, subscriptions = {}, {}, []
    node = None
    cancelled = False
    begin = time.monotonic()
    obs = {"odometry": 0, "deskewed_scans": 0, "feature_scans": 0, "global_maps": 0,
           "loop_constraint_line_points": 0}
    last_odom_wall = begin
    first_odom = None
    final_odom = None
    latest_map = None
    trajectory = (out/"odometry.tum").open("w")
    manifest = {"bag": str(bag), "output": str(out), "rmw": "rmw_zenoh_cpp", "domain": 220,
                "router": "loopback only, port 7448", "rate": args.rate, "duration_limit_sensor_s": args.duration,
                "loop_enabled": args.loop, "lidar_mode": args.lidar_mode,
                "input": "front and rear AIRY96 + central IMU" if args.lidar_mode == "dual" else "single front AIRY96 + central IMU",
                "mode": "Official LIO-SAM ROS2 with disclosed six-axis/projection experiment adapter",
                "calibration": str(args.calibration), "no_new_calibration": True,
                "upstream_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=HERE/"ws/src/LIO-SAM", text=True).strip(),
                "source_hashes": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/"adapter.py", HERE/"run.py", args.calibration]},
                "imu_noise": "upstream defaults, NOT central-IMU calibrated noise",
                "clock": "Original sensor headers retained; no bag receipt clock substitution, no robot clock changes"}
    write_json(out/"manifest.json", manifest)
    (out/"code").mkdir()
    for source in (HERE/"adapter.py", HERE/"run.py", HERE/"view.rviz", args.calibration):
        shutil.copy2(source, out/"code"/source.name)
    (out/"source.patch").write_bytes(subprocess.check_output(["git", "diff"], cwd=HERE/"ws/src/LIO-SAM"))

    def status(stage):
        data = {"stage": stage, "wall_seconds": time.monotonic()-begin, "output": str(out),
                **obs, "first_odometry": first_odom, "last_odometry": final_odom,
                "processes": {k: {"pid": p.pid, "returncode": p.poll()} for k, p in children.items()}}
        write_json(out/"status.json", data)
        print(json.dumps({"stage": stage, "wall_seconds": round(data["wall_seconds"], 1),
                          "sensor_seconds": round((final_odom["stamp_ns"]-first_odom["stamp_ns"])*1e-9, 2) if first_odom else 0,
                          "poses": obs["odometry"], "map_points": obs.get("latest_global_points", 0),
                          "xyz": final_odom["xyz"] if final_odom else None,
                          "loop_edges": obs["loop_constraint_line_points"]//2,
                          "output": str(out)}), flush=True)

    def start(name, cmd):
        logs[name] = (out/(name+".log")).open("w")
        children[name] = subprocess.Popen(cmd, stdout=logs[name], stderr=subprocess.STDOUT,
                                           start_new_session=True, cwd=HERE)
        return children[name]

    def stop(name):
        p = children.get(name)
        if not p or p.poll() is not None:
            return
        for sig, wait in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
            if p.poll() is not None:
                break
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                p.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                pass

    def cancel(*_):
        nonlocal cancelled
        cancelled = True

    def check():
        if cancelled:
            raise KeyboardInterrupt()
        for name in ("router", "adapter", "imageProjection", "featureExtraction", "imuPreintegration", "mapOptimization"):
            if name in children and children[name].poll() is not None:
                raise RuntimeError(f"{name} exited {children[name].returncode}; see log")

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            check()
            rclpy.spin_once(node, timeout_sec=.02)

    def on_odom(message):
        nonlocal first_odom, final_odom, last_odom_wall
        p, q = message.pose.pose.position, message.pose.pose.orientation
        ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        xyzq = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(xyzq).all():
            raise RuntimeError("Non-finite LIO-SAM odometry")
        item = {"stamp_ns": ns, "xyz": xyzq[:3], "q_xyzw": xyzq[3:]}
        first_odom = first_odom or item
        final_odom = item
        obs["odometry"] += 1
        last_odom_wall = time.monotonic()
        trajectory.write(f"{ns//10**9}.{ns%10**9:09d} "+" ".join(f"{v:.12g}" for v in xyzq)+"\n")
        trajectory.flush()
        if np.linalg.norm(xyzq[:3]) > 1000:
            raise RuntimeError("LIO-SAM exceeded 1 km bound for this short indoor dataset")

    def on_feature(message):
        obs["feature_scans"] += 1
        obs["last_corner_features"] = message.cloud_corner.width*message.cloud_corner.height
        obs["last_surface_features"] = message.cloud_surface.width*message.cloud_surface.height

    def on_projection(message):
        # featureExtraction intentionally clears these arrays before publishing.
        # Observe the projection output to verify both physical scanner banks.
        counts = [max(0, end-start+11) for start, end in zip(message.start_ring_index, message.end_ring_index)]
        obs["last_front_projected_points"] = sum(counts[:96])
        obs["last_rear_projected_points"] = sum(counts[96:])
        if args.lidar_mode == "dual" and len(counts) != 192:
            raise RuntimeError("Dual LiDAR expected 192 separate projection rows")
        if args.lidar_mode == "dual" and not (obs["last_front_projected_points"] and obs["last_rear_projected_points"]):
            raise RuntimeError("A dual-LiDAR feature scan has lost one complete sensor")

    def on_deskew(message):
        obs["deskewed_scans"] += 1
        obs["last_deskewed_points"] = message.width*message.height

    def on_map(message):
        nonlocal latest_map
        latest_map = message
        obs["global_maps"] += 1
        obs["latest_global_points"] = message.width*message.height

    def on_loop(message):
        obs["loop_constraint_line_points"] = sum(len(m.points) for m in message.markers if m.type == m.LINE_LIST)

    signal.signal(signal.SIGINT, cancel)
    signal.signal(signal.SIGTERM, cancel)
    complete = False
    try:
        start("router", [str(ROBOT/"local/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd")])
        deadline = time.monotonic()+8
        while True:
            try:
                with socket.create_connection(("127.0.0.1", 7448), timeout=.2):
                    break
            except OSError:
                if time.monotonic() > deadline or children["router"].poll() is not None:
                    raise RuntimeError("Local Zenoh router startup failed")
                time.sleep(.1)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("lio_sam_experiment_supervisor")
        subscriptions += [node.create_subscription(Odometry, "/lio_sam/mapping/odometry", on_odom, 200),
                          node.create_subscription(CloudInfo, "/lio_sam/feature/cloud_info", on_feature, 100),
                          node.create_subscription(CloudInfo, "/lio_sam/deskew/cloud_info", on_projection, 100),
                          node.create_subscription(PointCloud2, "/lio_sam/deskew/cloud_deskewed", on_deskew, 20),
                          node.create_subscription(PointCloud2, "/lio_sam/mapping/map_global", on_map, 2),
                          node.create_subscription(MarkerArray, "/lio_sam/mapping/loop_closure_constraints", on_loop, 10)]
        save = node.create_client(SaveMap, "/lio_sam/save_map")
        start("adapter", ["/usr/bin/python3", str(HERE/"adapter.py"), "--calibration", str(args.calibration), "--audit", str(out/"adapter.json"), "--lidar-mode", args.lidar_mode])
        binaries = HERE/"ws/install/lio_sam/lib/lio_sam"
        for executable in ("imuPreintegration", "imageProjection", "featureExtraction", "mapOptimization"):
            command = [str(binaries/("lio_sam_"+executable)), "--ros-args", "--params-file", str(out/"params.yaml")]
            if executable == "imuPreintegration" and args.debug_imu:
                command = ["gdb", "--batch", "-ex", "set pagination off", "-ex", "run",
                           "-ex", "thread apply all bt 16", "--args", *command]
            start(executable, command)
        if not args.no_rviz:
            start("rviz", ["/opt/ros/humble/lib/rviz2/rviz2", "-d", str(HERE/"view.rviz")])
        deadline = time.monotonic()+25
        while not (save.service_is_ready() and node.count_subscribers("/lio_sam/input/imu_sixaxis") >= 2
                   and node.count_subscribers("/lio_sam/input/points") >= 1
                   and node.count_subscribers("/front_lidar") >= 1
                   and (args.lidar_mode != "dual" or node.count_subscribers("/rear_lidar") >= 1)):
            spin(.1)
            if time.monotonic() > deadline:
                raise RuntimeError("LIO-SAM input/services not ready")
        # Reliable input QoS is explicit; this only affects isolated playback.
        qos_path = out/"playback_qos.yaml"
        topics = ["/front_lidar", "/imu_driver/imu_central"]
        if args.lidar_mode == "dual":
            topics.append("/rear_lidar")
        qos_path.write_text(yaml.safe_dump({t: {"history": "keep_last", "depth": 2000 if "imu" in t else 100, "reliability": "reliable", "durability": "volatile"}
                                           for t in topics}))
        player = start("playback", ["ros2", "bag", "play", str(bag), "--rate", str(args.rate),
                     "--read-ahead-queue-size", "500", "--disable-keyboard-controls", "--qos-profile-overrides-path", str(qos_path),
                     "--topics", *topics])
        status("mapping")
        last_status = time.monotonic()
        playback_begin = last_odom_wall = time.monotonic()
        limited = False
        while player.poll() is None:
            spin(.02)
            if time.monotonic()-last_odom_wall > 40:
                raise RuntimeError("No LIO-SAM odometry for 40 wall seconds")
            if (args.duration and first_odom and
                    (final_odom["stamp_ns"]-first_odom["stamp_ns"])*1e-9 >= args.duration):
                limited = True
                stop("playback")
                break
            if time.monotonic()-playback_begin > 985/args.rate+120:
                raise RuntimeError("Playback duration guard exceeded")
            if time.monotonic()-last_status > 10:
                status("mapping")
                last_status = time.monotonic()
        if not limited and player.returncode != 0:
            raise RuntimeError(f"Bag playback failed: {player.returncode}")
        spin(8.)
        if obs["odometry"] < 10:
            raise RuntimeError("Too few poses to export a valid test")
        request = SaveMap.Request()
        request.resolution, request.destination = .1, str(out/"map")
        future = save.call_async(request)
        deadline = time.monotonic()+60
        while not future.done() and time.monotonic() < deadline:
            spin(.05)
        if not future.done() or not future.result().success:
            raise RuntimeError("Map export failed/timed out")
        if not (out/"map/GlobalMap.pcd").is_file():
            raise RuntimeError("Save service did not produce GlobalMap.pcd")
        spin(2.)
        stop("adapter")
        if not limited:
            input_validation = validate_full_input(bag, json.loads((out/"adapter.json").read_text()), obs, args.lidar_mode)
            manifest["input_validation"] = input_validation
            write_json(out/"manifest.json", manifest)
            if not input_validation["passed"]:
                raise RuntimeError("Full bag input completeness gate failed; diagnostic map kept, Web publication refused")
        complete = True
        manifest["completion"] = {"status": "completed bounded segment" if limited else "completed full bag",
                                  "map": str(out/"map/GlobalMap.pcd"), "observed": obs,
                                  "sensor_duration_s": (final_odom["stamp_ns"]-first_odom["stamp_ns"])*1e-9,
                                  "estimated_endpoint_displacement_m": (np.array(final_odom["xyz"])-first_odom["xyz"]).tolist(),
                                  "not_accuracy_ground_truth": True}
        write_json(out/"manifest.json", manifest)
        for name in ("adapter", "mapOptimization", "featureExtraction", "imageProjection", "imuPreintegration"):
            stop(name)
        if args.publish_web:
            command = ["/usr/bin/python3", "-m", "backend.mapping.lio_sam_artifacts", "--run", str(out),
                       "--maps-root", str(WS/"maps"), "--sensor-mode", args.lidar_mode]
            result = subprocess.run(command, cwd=ROBOT/"map_manager", capture_output=True, text=True, timeout=60)
            if result.returncode:
                manifest["web_import_error"] = result.stderr[-3000:]
                print(json.dumps({"web_import_error": manifest["web_import_error"], "mapping_saved": True}), flush=True)
            else:
                manifest["web_artifact"] = json.loads(result.stdout)
                print(json.dumps({"web_artifact": manifest["web_artifact"]}), flush=True)
            write_json(out/"manifest.json", manifest)
        status("complete")
        if args.hold and not args.no_rviz and latest_map is not None:
            retained = node.create_publisher(PointCloud2, "/lio_sam/saved_map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
            retained.publish(latest_map)
            print(json.dumps({"visualization_only": True, "stop": "Ctrl+C or terminate this supervisor", "output": str(out)}), flush=True)
            while not cancelled and children["rviz"].poll() is None:
                rclpy.spin_once(node, timeout_sec=.2)
    except BaseException as exc:
        manifest["error"] = repr(exc)
        write_json(out/"manifest.json", manifest)
        status("cancelled" if isinstance(exc, KeyboardInterrupt) else "failed")
        if not isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        for name in ("playback", "adapter", "mapOptimization", "featureExtraction", "imageProjection", "imuPreintegration", "rviz"):
            stop(name)
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        stop("router")
        trajectory.close()
        for log in logs.values():
            log.close()
        write_json(out/"cleanup.json", {"complete": complete, "owned_processes": {k: {"pid": p.pid, "returncode": p.poll()} for k, p in children.items()}})


if __name__ == "__main__":
    main()
