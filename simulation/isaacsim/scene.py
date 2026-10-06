#!/usr/bin/env python3
"""Isaac Sim 6 indoor fixture: physical differential wheels and two real LiDARs.

Run with Isaac's python.sh. ROS runs in a separate Humble/Zenoh process; the
loopback wire protocol carries only measured state, ray hits and applied motion.
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import signal
import socket
import sys
import time
import traceback
import uuid
from pathlib import Path

import numpy as np

from protocol import COMMAND_PORT, STATE_PORT, decode, encode, finite_vector, imu_packet, ray_packets, state_packet

HERE = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--render-fps", type=float, default=0.,
                        help="GUI recording: render-only updates at this source cadence; 0 keeps the original loop")
    parser.add_argument("--scene-file", type=Path, default=HERE / "assets/scene_config.json")
    parser.add_argument("--scene-sha256", help="Require the sealed session's exact scene specification")
    parser.add_argument("--static-prior-geometry-sha256",
                        help="Require this static collider attestation before each measured state")
    parser.add_argument("--result-dir", type=Path, default=HERE / "runs/latest")
    parser.add_argument("--control-file", type=Path, help='Optional JSON {"paused":bool,"stop":bool}')
    parser.add_argument("--epoch", default=None)
    parser.add_argument("--physics-hz", type=float, default=None, help="Physics source frequency; default from sealed scene specification")
    parser.add_argument("--state-port", type=int, default=STATE_PORT)
    parser.add_argument("--command-port", type=int, default=COMMAND_PORT)
    parser.add_argument("--test-frames", type=int, default=0)
    parser.add_argument("--test-linear-speed", type=float, default=0.0,
                        help="Bounded physics calibration only; requires --test-frames")
    parser.add_argument("--test-angular-speed", type=float, default=0.0)
    parser.add_argument("--screenshot-path", type=Path)
    parser.add_argument("--screenshot-frame", type=int, default=90)
    parser.add_argument("--export-scene", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.render_fps) or not 0 <= args.render_fps <= 60:
        parser.error("render-fps must be finite and between 0 and 60")
    if args.headless and args.render_fps:
        parser.error("render-fps requires a visible GUI; omit --headless")
    if (args.physics_hz is not None and args.physics_hz <= 0) or args.test_frames < 0:
        parser.error("physics-hz must be positive and test-frames nonnegative")
    if (args.test_linear_speed or args.test_angular_speed) and not args.test_frames:
        parser.error("calibration motion requires a bounded --test-frames run")
    if args.static_prior_geometry_sha256 and (len(args.static_prior_geometry_sha256) != 64
            or any(c not in "0123456789abcdef" for c in args.static_prior_geometry_sha256)):
        parser.error("static-prior-geometry-sha256 must be a lower-case SHA256")
    return args


ARGS = parse_args()
SCENE_BYTES = ARGS.scene_file.read_bytes()
if ARGS.scene_sha256 and hashlib.sha256(SCENE_BYTES).hexdigest() != ARGS.scene_sha256:
    raise SystemExit("Scene specification does not match the sealed navigation session")
CONFIG = json.loads(SCENE_BYTES)
ARGS.physics_hz = ARGS.physics_hz or float(CONFIG.get("physics", {}).get("frequency_hz", 120.0))
if ARGS.physics_hz < CONFIG["imu"]["frequency_hz"]:
    raise SystemExit("Physics frequency must cover the configured native IMU sampling frequency")
if ARGS.render_fps > ARGS.physics_hz / 2:
    raise SystemExit("render-fps must not exceed the real body-state source frequency")
EPOCH = ARGS.epoch or str(uuid.uuid4())
RESULT_DIR = ARGS.result_dir.expanduser().resolve()
RESULT_DIR.mkdir(parents=True, exist_ok=True)
sys.argv = [sys.argv[0]]  # Keep standalone options out of Kit's own parser.

from isaacsim import SimulationApp

APP = SimulationApp({"headless": ARGS.headless, "width": 1280, "height": 900,
                     "renderer": "RaytracedLighting",
                     "disable_viewport_updates": bool(ARGS.headless and ARGS.screenshot_path is None)})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.core.experimental.materials import RigidBodyMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.robot.experimental.wheeled_robots.robots import WheeledRobot
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def array(value):
    return value.numpy() if hasattr(value, "numpy") else np.asarray(value)


def robot_asset():
    source = HERE / "assets/wheel_fixture.urdf"
    # Generated import products belong to this run, never to the sealed
    # candidate. Reimport its sealed URDF for every scene process.
    cache = RESULT_DIR / "robot_cache"
    digest = hashlib.sha256(source.read_bytes() + b"isaac6-drive80-v1").hexdigest()
    stamp = cache / "source.sha256"
    cache.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(source), usd_path=str(cache), fix_base=False,
        merge_fixed_joints=False, collision_from_visuals=False,
        joint_drive_type="acceleration", joint_target_type="velocity",
        override_joint_stiffness=0.0, override_joint_damping=80.0,
        run_asset_transformer=False, run_multi_physics_conversion=True)
    output = URDFImporter(config).import_urdf()
    if not output:
        raise RuntimeError("URDF importer returned no fixture USD")
    stamp.write_text(digest + "\n")
    return Path(output)


def add_box(name, center, size, color):
    path = "/World/Indoor/" + name
    Cube(path, positions=[center], scales=[(np.asarray(size) * 0.5).tolist()], colors=[color])
    GeomPrim(path, apply_collision_apis=True)
    return path


def build_room():
    GroundPlane("/World/GroundPlane", sizes=14.0, colors=[[0.34, 0.38, 0.43]], templates=None)
    for box in CONFIG["static_boxes"]:
        color = [0.68, 0.74, 0.83] if box["kind"] == "wall" else [0.82, 0.48, 0.20]
        add_box(box["name"], box["center"], box["size"], color)
    ceiling, room = CONFIG["ceiling"], CONFIG["room"]
    path = add_box("ceiling", [0, 0, ceiling["z"] + ceiling["thickness"] / 2],
                   [room["width"], room["depth"], ceiling["thickness"]], [0.8, 0.8, 0.8])
    if not ceiling["visible"]:
        # Invisible in the overview, still a real PhysX collision surface.
        UsdGeom.Imageable(omni.usd.get_context().get_stage().GetPrimAtPath(path)).MakeInvisible()
    DomeLight("/World/DomeLight").set_intensities([850.0])
    stage = omni.usd.get_context().get_stage()
    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
    # USD camera apertures are tenths of the stage's metre unit.
    camera.CreateHorizontalApertureAttr(152.0)
    camera.CreateVerticalApertureAttr(112.0)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 14))
    try:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = str(camera.GetPath())
    except ImportError:
        pass


def find_link(name):
    root = omni.usd.get_context().get_stage().GetPrimAtPath("/World/WheelFixture")
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    raise RuntimeError(f"Fixture missing link {name}")


def assign_contact_materials():
    tire = RigidBodyMaterial("/World/Materials/Tire", static_frictions=1.0, dynamic_frictions=0.9,
                             restitutions=0.0)
    caster = RigidBodyMaterial("/World/Materials/Caster", static_frictions=0.0, dynamic_frictions=0.0,
                               restitutions=0.0)
    # Friction combine=min ensures the caster slides while the drive tires grip.
    from pxr import PhysxSchema
    PhysxSchema.PhysxMaterialAPI.Apply(caster.materials[0].GetPrim()).CreateFrictionCombineModeAttr("min")
    for link, material in [("left_wheel_link", tire), ("right_wheel_link", tire),
                           ("front_caster_link", caster), ("rear_caster_link", caster)]:
        parent = omni.usd.get_context().get_stage().GetPrimAtPath(find_link(link))
        colliders = [str(p.GetPath()) for p in Usd.PrimRange(parent) if p.HasAPI(UsdPhysics.CollisionAPI)]
        if colliders:
            geometry = GeomPrim(colliders)
            geometry.apply_physics_materials(material)
            geometry.set_offsets(contact_offsets=0.002, rest_offsets=0.0)


def external_hit_mask(points):
    """Exclude only this fixture's authored collision solids, in body coordinates.

    A self hit is dropped; its occluded ray never becomes an invented free ray.
    """
    robot = CONFIG["robot"]
    half = np.asarray(robot["body_size"]) / 2 + 0.005
    inside = np.all(np.abs(points) <= half, axis=1)
    for side in (-1, 1):
        inside |= ((np.abs(points[:, 1] - side * robot["wheel_base"] / 2)
                    <= robot["wheel_width"] / 2 + 0.005)
                   & (points[:, 0] ** 2 + (points[:, 2] - robot["wheel_axle_z"]) ** 2
                      <= (robot["wheel_radius"] + 0.005) ** 2))
    for center in robot["caster_origins"]:
        inside |= np.linalg.norm(points - np.asarray(center), axis=1) <= robot["caster_radius"] + 0.005
    return ~inside


class WireInterface:
    def __init__(self, robot, controller):
        self.robot, self.controller = robot, controller
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(("127.0.0.1", ARGS.command_port))
        self.receiver.setblocking(False)
        self.command = np.zeros(2)
        self.command_until = 0.0
        self.command_source_deadline_ns = 0
        self.last_command_sequence = -1
        self.commands_received = self.commands_rejected = 0
        self.state_sequence = 0
        self.scan_sequence = 0
        self.scan_counts = [0, 0]
        self.hit_counts = [0, 0]
        self.first_hits = [None, None]
        self.last_scan_data = [None, None]
        self.sim_time_ns = 0
        self.imu_count = 0
        self.imu_invalid = 0
        self.imu_duplicate = 0
        self.imu_rate_skips = 0
        self.imu_native_count = 0
        self.imu_native_first_ns = None
        self.imu_native_last_ns = -1
        self.imu_next_due_ns = 0
        self.imu_first_ns = None
        self.imu_last_ns = -1
        self.imu_acc_sum = np.zeros(3)
        self.imu_ang_sum = np.zeros(3)
        self.imu_acc_min = np.full(3, np.inf)
        self.imu_acc_max = np.full(3, -np.inf)
        self.imu_ang_min = np.full(3, np.inf)
        self.imu_ang_max = np.full(3, -np.inf)
        self.imu_first = self.imu_last = None
        self.imu_stream = (RESULT_DIR / "imu_readings.jsonl").open("w")
        self.state_first_wall = self.state_last_wall = None
        self.state_first_source_ns = None
        self.state_max_wall_gap = 0.0
        self.ray_last_wall = [None, None]
        self.ray_max_wall_gaps = [0.0, 0.0]
        self.ray_phase_timings = {}
        self.geometry_audit_count = self.geometry_audit_faults = 0
        self.geometry_audit_total_wall_s = self.geometry_audit_max_wall_s = 0.0
        self.geometry_last_sha256 = self.geometry_last_fault = ""
        self.geometry_fault_latched = False

    def ray_phase(self, name, started):
        elapsed = time.monotonic() - started
        item = self.ray_phase_timings.setdefault(name, dict(count=0, total_wall_s=0.0, max_wall_s=0.0))
        item["count"] += 1
        item["total_wall_s"] += elapsed
        item["max_wall_s"] = max(item["max_wall_s"], elapsed)

    def receive_command(self, paused=False):
        for _ in range(128):
            try:
                packet, address = self.receiver.recvfrom(60001)
            except BlockingIOError:
                break
            try:
                value = decode(packet)
                if (address[0] != "127.0.0.1" or value["type"] != "command" or value["epoch"] != EPOCH
                        or type(value.get("sequence")) is not int
                        or value["sequence"] <= self.last_command_sequence):
                    raise ValueError("wrong_command_identity")
                command = finite_vector([value["vx"], value["wz"]], 2)
                lifetime = float(value["valid_for_s"])
                if not math.isfinite(lifetime) or not 0 < lifetime <= .25:
                    raise ValueError("invalid_lifetime")
                source_age_ns = self.sim_time_ns - value["sim_time_ns"]
                if not -round(2_000_000_000 / ARGS.physics_hz) <= source_age_ns < round(lifetime * 1e9):
                    raise ValueError("stale_command_source_time")
                self.command[:] = command
                self.command_source_deadline_ns = value["sim_time_ns"] + round(lifetime * 1e9)
                self.command_until = time.monotonic() + lifetime - max(0, source_age_ns) * 1e-9
                self.last_command_sequence = value["sequence"]
                self.commands_received += 1
            except (KeyError, ValueError, TypeError, json.JSONDecodeError):
                self.commands_rejected += 1
        if paused or self.geometry_fault_latched or time.monotonic() >= self.command_until or self.sim_time_ns >= self.command_source_deadline_ns:
            self.command[:] = 0
        if not paused and not self.geometry_fault_latched and ARGS.test_frames and (ARGS.test_linear_speed or ARGS.test_angular_speed):
            self.command[:] = [ARGS.test_linear_speed, ARGS.test_angular_speed]
        self.robot.apply_wheel_actions(self.controller.forward(self.command))

    def guard_command_expiry(self, step_dt, source_start_time):
        if ARGS.test_frames and (ARGS.test_linear_speed or ARGS.test_angular_speed):
            return
        next_step_ns = round((SimulationManager.get_simulation_time() - source_start_time + step_dt) * 1e9)
        if (time.monotonic() >= self.command_until or next_step_ns >= self.command_source_deadline_ns) and np.any(self.command):
            self.command[:] = 0
            self.robot.apply_wheel_actions(self.controller.forward(self.command))

    def send(self, packet):
        self.sender.sendto(packet, ("127.0.0.1", ARGS.state_port))

    def status(self, playing):
        # Pausing must revoke the graph promptly, including a short GUI pause.
        # This does not advance /clock or republish a measured state timestamp.
        self.send(encode(dict(schema=1, type="status", epoch=EPOCH,
                              sim_time_ns=self.sim_time_ns, playing=bool(playing))))

    def state(self, sim_time_ns):
        self.sim_time_ns = sim_time_ns
        positions, orientations = [array(v)[0] for v in self.robot.get_world_poses()]
        linear, angular = [array(v)[0] for v in self.robot.get_velocities()]
        w, x, y, z = orientations.tolist()
        pose = positions.tolist() + [x, y, z, w]
        metadata = None
        if ARGS.static_prior_geometry_sha256:
            # This is a geometry-domain attestation, not a lidar observation or
            # localization measurement. Preserve the actual body's source stamp.
            from truth_map import verify_stage_geometry
            started = time.monotonic()
            actual_hash, fault = "", ""
            try:
                actual_hash = verify_stage_geometry(omni.usd.get_context().get_stage(), CONFIG)
                if actual_hash != ARGS.static_prior_geometry_sha256:
                    fault = "static_geometry_hash_mismatch"
            except ValueError as error:
                fault = str(error)[:512]
            elapsed = time.monotonic() - started
            self.geometry_audit_count += 1
            self.geometry_audit_total_wall_s += elapsed
            self.geometry_audit_max_wall_s = max(self.geometry_audit_max_wall_s, elapsed)
            self.geometry_last_sha256 = actual_hash
            if fault:
                self.geometry_audit_faults += 1
                self.geometry_fault_latched = True
                self.geometry_last_fault = fault
            if self.geometry_fault_latched:
                self.command[:] = 0
                self.robot.apply_wheel_actions(self.controller.forward(self.command))
            metadata = dict(static_prior_geometry_sha256=actual_hash,
                static_prior_geometry_valid=not self.geometry_fault_latched,
                static_prior_geometry_checked_sim_time_ns=sim_time_ns,
                static_prior_geometry_fault=self.geometry_last_fault)
        self.send(state_packet(EPOCH, self.state_sequence, sim_time_ns, pose, linear.tolist(), angular.tolist(), metadata=metadata))
        now_wall = time.monotonic()
        if self.state_first_wall is None:
            self.state_first_wall = now_wall
            self.state_first_source_ns = sim_time_ns
        if self.state_last_wall is not None:
            self.state_max_wall_gap = max(self.state_max_wall_gap, now_wall - self.state_last_wall)
        self.state_last_wall = now_wall
        self.state_sequence += 1
        return pose, linear.tolist(), angular.tolist()

    def timing_summary(self):
        wall = self.state_last_wall - self.state_first_wall if self.state_first_wall is not None else 0.0
        simulated = (self.sim_time_ns - self.state_first_source_ns) * 1e-9 if self.state_first_source_ns is not None else 0.0
        return dict(measured_state_samples=self.state_sequence, measured_state_wall_seconds=wall,
                    measured_state_sim_seconds=simulated, wall_seconds_per_sim_second=wall / simulated if simulated else None,
                    realtime_factor=simulated / wall if wall else None,
                    state_wall_hz=(self.state_sequence - 1) / wall if wall else None,
                    longest_state_wall_gap_s=self.state_max_wall_gap,
                    longest_rays_wall_gap_s=self.ray_max_wall_gaps)

    def imu(self, imu_sensor, source_start_time):
        reading = imu_sensor.get_sensor_reading(read_gravity=CONFIG["imu"]["include_gravity"])
        if not reading.is_valid:
            self.imu_invalid += 1
            return
        # Both plant and IMU use the same epoch offset; only the sensor's own
        # source timestamp decides whether this is a new reading.
        sim_time_ns = round((float(reading.time) - source_start_time) * 1e9)
        if sim_time_ns < 0 or sim_time_ns <= self.imu_native_last_ns:
            self.imu_duplicate += 1
            return
        self.imu_native_count += 1
        if self.imu_native_first_ns is None:
            self.imu_native_first_ns = sim_time_ns
        self.imu_native_last_ns = sim_time_ns
        # Isaac 6's C++ IMU reports one real measurement per physical substep.
        # Select the configured output rate from those samples while retaining
        # their measured timestamps; there is no interpolation or restamping.
        if sim_time_ns + 500 < self.imu_next_due_ns:
            self.imu_rate_skips += 1
            return
        period_ns = round(1e9 / CONFIG["imu"]["frequency_hz"])
        while self.imu_next_due_ns <= sim_time_ns + 500:
            self.imu_next_due_ns += period_ns
        acceleration = np.array([reading.linear_acceleration_x, reading.linear_acceleration_y,
                                 reading.linear_acceleration_z])
        angular = np.array([reading.angular_velocity_x, reading.angular_velocity_y,
                            reading.angular_velocity_z])
        w, x, y, z = reading.orientation_w, reading.orientation_x, reading.orientation_y, reading.orientation_z
        orientation = [x, y, z, w]
        self.send(imu_packet(EPOCH, self.imu_count, sim_time_ns,
                             orientation, angular.tolist(), acceleration.tolist()))
        reading = dict(sequence=self.imu_count, sim_time_ns=sim_time_ns,
                       absolute_sensor_time=float(reading.time), orientation_xyzw=orientation,
                       angular_velocity=angular.tolist(), linear_acceleration=acceleration.tolist())
        self.imu_stream.write(json.dumps(reading, separators=(",", ":"), allow_nan=False) + "\n")
        self.imu_count += 1
        if self.imu_first_ns is None:
            self.imu_first_ns = sim_time_ns
            self.imu_first = reading
        self.imu_last_ns, self.imu_last = sim_time_ns, reading
        self.imu_acc_sum += acceleration
        self.imu_ang_sum += angular
        self.imu_acc_min = np.minimum(self.imu_acc_min, acceleration)
        self.imu_acc_max = np.maximum(self.imu_acc_max, acceleration)
        self.imu_ang_min = np.minimum(self.imu_ang_min, angular)
        self.imu_ang_max = np.maximum(self.imu_ang_max, angular)

    def imu_summary(self):
        count = self.imu_count
        span = (self.imu_last_ns - self.imu_first_ns) * 1e-9 if count > 1 else 0.0
        native_span = (self.imu_native_last_ns - self.imu_native_first_ns) * 1e-9 if self.imu_native_count > 1 else 0
        return dict(model="Isaac 6 PhysX experimental IMUSensor", configured_hz=CONFIG["imu"]["frequency_hz"],
                    physics_hz=ARGS.physics_hz, native_samples=self.imu_native_count,
                    native_measured_source_hz=(self.imu_native_count - 1) / native_span if native_span > 0 else None,
                    output_sampling="select real native readings; retain their measured timestamps",
                    measured_source_hz=(count - 1) / span if span > 0 else None,
                    samples=count, invalid_reads=self.imu_invalid, duplicate_reads_skipped=self.imu_duplicate,
                    native_rate_samples_skipped=self.imu_rate_skips,
                    source_start_time=self.imu_first_ns, source_end_time=self.imu_last_ns,
                    include_gravity=CONFIG["imu"]["include_gravity"],
                    first_reading=self.imu_first, last_reading=self.imu_last,
                    acceleration_mean=(self.imu_acc_sum / count).tolist() if count else None,
                    acceleration_min=self.imu_acc_min.tolist() if count else None,
                    acceleration_max=self.imu_acc_max.tolist() if count else None,
                    angular_velocity_mean=(self.imu_ang_sum / count).tolist() if count else None,
                    angular_velocity_min=self.imu_ang_min.tolist() if count else None,
                    angular_velocity_max=self.imu_ang_max.tolist() if count else None)

    def rays(self, lidars, sim_time_ns):
        for sensor_id, (lidar, origin) in enumerate(lidars):
            phase_start = time.monotonic()
            frame = lidar.get_current_frame()
            cloud, depth = frame.get("point_cloud"), frame.get("linear_depth")
            if cloud is None or depth is None:
                continue
            measured_scan_ns = round(float(frame["time"]) * 1e9)
            expected_step = round(sim_time_ns * ARGS.physics_hz / 1e9)
            # The native sensor sums its float step sizes; preserve that clock
            # rather than restamping it to the body's integer step clock.
            if (frame["physics_step"] != expected_step
                    or abs(measured_scan_ns - sim_time_ns) > round(1e8 / ARGS.physics_hz)):
                raise RuntimeError(f"LiDAR snapshot is not aligned with measured body state: {measured_scan_ns}/{sim_time_ns}")
            raw_shape = list(array(cloud).shape)
            depth_shape = list(array(depth).shape)
            native_rows, native_cols = lidar.get_num_rows(), lidar.get_num_cols()
            if raw_shape != [native_cols, native_rows, 3] or depth_shape != [native_cols, native_rows]:
                raise RuntimeError("LiDAR native column/vertical-row layout changed; refuse invented ring labels")
            rings = np.tile(np.arange(native_rows, dtype=np.uint16), native_cols)
            cloud = array(cloud).reshape(-1, 3)
            depth = array(depth).reshape(-1)
            self.ray_phase("native_frame_and_array", phase_start)
            if len(cloud) != len(depth):
                raise RuntimeError("LiDAR point/depth acquisition shapes disagree")
            phase_start = time.monotonic()
            valid = (np.all(np.isfinite(cloud), axis=1) & np.isfinite(depth)
                     & (depth >= CONFIG["lidar"]["range_min"])
                     & (depth < CONFIG["lidar"]["range_max"] - .001))
            points = cloud[valid] + origin
            rings = rings[valid]
            external = external_hit_mask(points)
            points, rings = points[external], rings[external]
            self.ray_phase("exact_self_and_depth_filter", phase_start)
            if not len(points):
                continue
            # Preserve the native XYZ precision used by ROS PointCloud2.
            points = np.ascontiguousarray(points, dtype=np.float32)
            rings = np.ascontiguousarray(rings, dtype=np.uint16)
            phase_start = time.monotonic()
            for packet in ray_packets(EPOCH, self.scan_sequence, measured_scan_ns, sensor_id,
                                      origin.tolist(), points, rings=rings):
                self.send(packet)
            self.ray_phase("binary_pack_and_udp", phase_start)
            now_wall = time.monotonic()
            if self.ray_last_wall[sensor_id] is not None:
                self.ray_max_wall_gaps[sensor_id] = max(self.ray_max_wall_gaps[sensor_id],
                                                       now_wall - self.ray_last_wall[sensor_id])
            self.ray_last_wall[sensor_id] = now_wall
            self.scan_counts[sensor_id] += 1
            self.hit_counts[sensor_id] = len(points)
            self.last_scan_data[sensor_id] = dict(xyz=points, origin=origin, ring=rings,
                pose=np.asarray(self.last_pose), sim_time_ns=measured_scan_ns)
            if self.first_hits[sensor_id] is None:
                self.first_hits[sensor_id] = {"count": len(points), "body_min": points.min(axis=0).tolist(),
                                               "body_max": points.max(axis=0).tolist(),
                                               "raw_cloud_shape": raw_shape, "raw_depth_shape": depth_shape,
                                               "native_rows": native_rows, "native_cols": native_cols,
                                               "native_scan_time_ns": measured_scan_ns,
                                               "native_scan_physics_step": frame["physics_step"],
                                               "zenith": array(frame["zenith"]).tolist()}
                phase_start = time.monotonic()
                np.savez(RESULT_DIR / f"first_scan_{sensor_id}.npz", xyz=points, origin=origin, ring=rings,
                                    pose=np.asarray(self.last_pose), sim_time_ns=measured_scan_ns)
                self.ray_phase("first_scan_evidence_save", phase_start)
        self.scan_sequence += 1

    def close(self):
        self.imu_stream.close()
        self.robot.apply_wheel_actions(self.controller.forward(np.zeros(2)))
        self.sender.close()
        self.receiver.close()
        for sensor_id, capture in enumerate(self.last_scan_data):
            if capture is not None:
                np.savez(RESULT_DIR / f"last_scan_{sensor_id}.npz", **capture)


def main():
    for extension in ("isaacsim.asset.importer.urdf", "isaacsim.robot.wheeled_robots", "isaacsim.sensors.physx", "isaacsim.sensors.experimental.physics"):
        app_utils.enable_extension(extension)
    APP.update()
    APP.update()
    asset = robot_asset()
    generated_robot_usd_sha256 = hashlib.sha256(asset.read_bytes()).hexdigest()
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    build_room()
    if not ARGS.headless and ARGS.render_fps:
        # Give the actual viewport room in the two-window recording layout.
        import omni.ui
        for name in ("Stage", "Property", "Content", "Console"):
            window = omni.ui.Workspace.get_window(name)
            if window is not None:
                window.visible = False
    robot_config = CONFIG["robot"]
    x, y, z, yaw = robot_config["initial_pose"]
    robot = WheeledRobot("/World/WheelFixture", wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                         usd_path=str(asset), positions=[[x, y, z]],
                         orientations=[[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]])
    APP.update()
    assign_contact_materials()
    from isaacsim.sensors.physx import RotatingLidarPhysX
    lidars = []
    lidar_config = CONFIG["lidar"]
    for index, xyz in enumerate(lidar_config["origins"]):
        origin = np.asarray(xyz, dtype=float)
        lidar = RotatingLidarPhysX(
            prim_path=find_link("base_link") + f"/Lidar_{index}", name=f"fixture_lidar_{index}",
            translation=origin, rotation_frequency=0.0,
            fov=(lidar_config["horizontal_fov_deg"], lidar_config["vertical_fov_deg"]),
            resolution=(lidar_config["horizontal_resolution_deg"], lidar_config["vertical_resolution_deg"]),
            valid_range=(lidar_config["range_min"], lidar_config["range_max"]))
        lidar.add_linear_depth_data_to_frame()
        lidar.add_point_cloud_data_to_frame()
        lidar.add_zenith_data_to_frame()
        lidars.append((lidar, origin))
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor
    imu_config = CONFIG["imu"]
    imu_sensor = IMUSensor(IMU.create(find_link("base_link") + "/BodyImu",
                           translations=[imu_config["origin"]], orientations=[imu_config["orientation_wxyz"]],
                           linear_acceleration_filter_size=imu_config["linear_acceleration_filter_size"],
                           angular_velocity_filter_size=imu_config["angular_velocity_filter_size"],
                           orientation_filter_size=imu_config["orientation_filter_size"]))
    dt = 1 / ARGS.physics_hz
    SimulationManager.setup_simulation(dt=dt, device="cpu")
    SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
    recording_render_mode = not ARGS.headless and ARGS.render_fps > 0
    if recording_render_mode:
        app_utils.enable_extension("isaacsim.core.rendering_manager")
        from isaacsim.core.rendering_manager import RenderingManager
    controller = DifferentialController(wheel_radius=robot_config["wheel_radius"], wheel_base=robot_config["wheel_base"],
                                        max_linear_speed=robot_config["max_linear_speed"],
                                        max_angular_speed=robot_config["max_angular_speed"], max_wheel_speed=12.0)
    app_utils.play()
    settle_start = SimulationManager.get_num_physics_steps()
    while SimulationManager.get_num_physics_steps() - settle_start < int(2 * ARGS.physics_hz):
        APP.update()
    for lidar, _ in lidars:
        lidar.initialize()
    wire = WireInterface(robot, controller)
    initial_positions, initial_orientations = [array(v)[0].tolist() for v in robot.get_world_poses()]
    source_steps = SimulationManager.get_num_physics_steps()
    source_start_time = SimulationManager.get_simulation_time()
    # The application can render at 60 Hz while physics advances at 120 Hz.
    # Read the native IMU after each physical step so its 100 Hz source does
    # not get undersampled by viewport rendering; state is still emitted once
    # per application update (normally 60 Hz).
    import omni.physics.core
    imu_subscription = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=False, order=2,
        on_update=lambda _dt, _context: wire.imu(imu_sensor, source_start_time))
    command_expiry_subscription = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=True, order=0,
        on_update=lambda _dt, _context: wire.guard_command_expiry(_dt, source_start_time))
    next_lidar_capture_ns = round(2 * dt * 1e9)
    lidar_capture_count = 0
    lidar_native_enabled = True

    def gate_lidar_capture(step_dt, _context):
        nonlocal next_lidar_capture_ns, lidar_capture_count, lidar_native_enabled
        measurement_ns = round((SimulationManager.get_simulation_time() - source_start_time + step_dt) * 1e9)
        due = measurement_ns + 500 >= next_lidar_capture_ns
        for lidar, _ in lidars:
            if due != lidar_native_enabled:
                # The documented RangeSensor 'enabled' switch gates the real
                # native raycast, not just the Python copy of an old scan.
                lidar.prim.GetAttribute("enabled").Set(due)
            lidar.resume() if due else lidar.pause()
        lidar_native_enabled = due
        if due:
            lidar_capture_count += 1
            while next_lidar_capture_ns <= measurement_ns + 500:
                next_lidar_capture_ns += round(1e9 / lidar_config["frequency_hz"])

    lidar_gate_subscription = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=True, order=1, on_update=gate_lidar_capture)
    previous_steps = source_steps
    next_scan_ns = 0
    stop_requested = False
    pause_requested = False
    screenshot_task = None
    viewport_disabled = bool(ARGS.headless and ARGS.screenshot_path is None)
    viewport_disabled_at_frame = 0 if viewport_disabled else None
    frame = 0
    last_state = None
    next_control_check = 0
    last_playing = True
    next_pause_status = 0.0
    direct_step_wall_anchor = None
    direct_step_source_anchor = None
    render_period_ns = round(1e9 / ARGS.render_fps) if recording_render_mode else 0
    next_render_ns = render_period_ns
    next_paused_render_wall = 0.0
    render_calls = 0
    render_failed = False
    render_advanced_physics_frames = 0
    render_max_wall_s = 0.0
    render_uses_fabric = SimulationManager.is_fabric_enabled() if recording_render_mode else False
    phase_timings = {}
    trajectory = (RESULT_DIR / "trajectory.jsonl").open("w")

    def record_phase(name, started):
        elapsed = time.monotonic() - started
        item = phase_timings.setdefault(name, dict(count=0, total_wall_s=0.0, max_wall_s=0.0))
        item["count"] += 1
        item["total_wall_s"] += elapsed
        item["max_wall_s"] = max(item["max_wall_s"], elapsed)

    def render_without_physics():
        nonlocal render_calls, render_failed, render_advanced_physics_frames, render_max_wall_s
        # Isaac 6 RenderingManager.render (impl/rendering_manager.py:99-118)
        # temporarily disables /app/player/playSimulations around Kit update.
        # Restore the original setting even if that API raises, then assert
        # that neither physical steps nor their measured source clock changed.
        setting = "/app/player/playSimulations"
        settings = carb.settings.get_settings()
        original_play_simulations = settings.get_as_bool(setting)
        before_steps = SimulationManager.get_num_physics_steps()
        before_time = SimulationManager.get_simulation_time()
        started = time.monotonic()
        render_failed = True
        try:
            if not render_uses_fabric:
                # Native PhysX API only publishes already simulated transforms
                # to USD (_physx.pyi:1911); preserve the existing USD backend.
                import omni.physx
                omni.physx.get_physx_interface().update_transformations(False, True, False)
            RenderingManager.render()
        finally:
            settings.set_bool(setting, original_play_simulations)
            after_steps = SimulationManager.get_num_physics_steps()
            render_advanced_physics_frames += abs(after_steps - before_steps)
            render_max_wall_s = max(render_max_wall_s, time.monotonic() - started)
            if after_steps != before_steps or SimulationManager.get_simulation_time() != before_time:
                raise RuntimeError("Render-only update advanced physics; terminate this simulation epoch")
        render_calls += 1
        render_failed = False

    def request_stop(signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    if ARGS.export_scene:
        ARGS.export_scene.parent.mkdir(parents=True, exist_ok=True)
        omni.usd.get_context().get_stage().Flatten().Export(str(ARGS.export_scene.resolve()))
    (RESULT_DIR / "ready.json").write_text(json.dumps({"epoch": EPOCH, "pid": __import__("os").getpid(),
        "initial_position": initial_positions, "schema": 1, "sensor_origins": lidar_config["origins"],
        "generated_robot_usd": str(asset), "generated_robot_usd_sha256": generated_robot_usd_sha256}, indent=2))
    carb.log_info(f"Wheel fixture READY epoch={EPOCH}; physical wheel drives, two PhysX LiDARs; UDP {ARGS.state_port}/{ARGS.command_port}")
    try:
        while not stop_requested and (ARGS.headless or APP.is_running()):
            now = time.monotonic()
            if ARGS.control_file and now >= next_control_check:
                next_control_check = now + .1
                try:
                    control = json.loads(ARGS.control_file.read_text())
                    stop_requested = bool(control.get("stop", False))
                    requested = bool(control.get("paused", False))
                    if requested != pause_requested:
                        pause_requested = requested
                        wire.command[:] = 0
                        wire.command_until = 0
                        wire.receive_command(paused=True)
                        app_utils.pause() if requested else app_utils.play()
                except (OSError, json.JSONDecodeError):
                    pass
            playing = app_utils.is_playing() and not pause_requested
            if not playing and (last_playing or now >= next_pause_status):
                wire.status(False)
                next_pause_status = now + .1
            last_playing = playing
            phase_start = time.monotonic()
            wire.receive_command(paused=not playing)
            record_phase("receive_command", phase_start)
            if playing and ((ARGS.headless and viewport_disabled) or recording_render_mode):
                # The documented physics-only step still invokes the native
                # PhysX LiDAR and IMU post-step callbacks. Skip Kit rendering
                # and keep the real source clock at most wall-clock speed.
                if direct_step_wall_anchor is None:
                    direct_step_wall_anchor = time.monotonic()
                    direct_step_source_anchor = SimulationManager.get_simulation_time()
                phase_start = time.monotonic()
                SimulationManager.step(steps=2, update_fabric=render_uses_fabric if recording_render_mode else False)
                record_phase("physics_only_two_steps", phase_start)
                source_elapsed = SimulationManager.get_simulation_time() - direct_step_source_anchor
                remaining = direct_step_wall_anchor + source_elapsed - time.monotonic()
                if remaining > 0:
                    time.sleep(min(remaining, 2 * dt))
            else:
                direct_step_wall_anchor = direct_step_source_anchor = None
                phase_start = time.monotonic()
                if recording_render_mode:
                    if now >= next_paused_render_wall:
                        render_without_physics()
                        next_paused_render_wall = time.monotonic() + 1 / ARGS.render_fps
                        record_phase("paused_render_only_update", phase_start)
                else:
                    APP.update()
                    record_phase("application_update", phase_start)
            if ARGS.headless and not viewport_disabled and screenshot_task is not None and screenshot_task.done():
                from omni.kit.viewport.utility import get_active_viewport
                viewport = get_active_viewport()
                if viewport:
                    viewport.updates_enabled = False
                    viewport_disabled = True
                    viewport_disabled_at_frame = SimulationManager.get_num_physics_steps() - source_steps
            steps = SimulationManager.get_num_physics_steps()
            if steps == previous_steps:
                if pause_requested or (recording_render_mode and not playing):
                    time.sleep(.005)
                continue
            if steps < previous_steps:
                raise RuntimeError("Physics time reset: terminate and start a new simulation epoch")
            previous_steps = steps
            frame = steps - source_steps
            sim_time_ns = round(frame * 1_000_000_000 / ARGS.physics_hz)
            phase_start = time.monotonic()
            last_state = wire.state(sim_time_ns)
            record_phase("measured_state_and_udp", phase_start)
            wire.last_pose = last_state[0]
            trajectory.write(json.dumps({"sim_time_ns": sim_time_ns, "pose": last_state[0],
                             "command": wire.command.tolist()}, separators=(",", ":"), allow_nan=False) + "\n")
            if frame % 60 == 0:
                trajectory.flush()
            if sim_time_ns >= next_scan_ns:
                phase_start = time.monotonic()
                wire.rays(lidars, sim_time_ns)
                record_phase("dual_cloud_filter_pack_and_udp", phase_start)
                next_scan_ns = sim_time_ns + round(1_000_000_000 / lidar_config["frequency_hz"])
            if ARGS.screenshot_path and screenshot_task is None and frame >= ARGS.screenshot_frame:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                ARGS.screenshot_path.parent.mkdir(parents=True, exist_ok=True)
                viewport = get_active_viewport()
                if viewport:
                    capture = capture_viewport_to_file(viewport, file_path=str(ARGS.screenshot_path.resolve()), is_hdr=False)
                    screenshot_task = asyncio.ensure_future(capture.wait_for_result())
            if recording_render_mode and sim_time_ns >= next_render_ns:
                # Publish each real state/cloud before spending time drawing.
                # The display cadence never changes the 120 Hz physical steps,
                # native sensor stamps, command TTL, or 60 Hz body-state source.
                phase_start = time.monotonic()
                render_without_physics()
                record_phase("render_only_update", phase_start)
                while next_render_ns <= sim_time_ns:
                    next_render_ns += render_period_ns
            if ARGS.test_frames and frame >= ARGS.test_frames:
                break
    finally:
        imu_subscription = None
        command_expiry_subscription = None
        lidar_gate_subscription = None
        trajectory.close()
        wire.close()
        if screenshot_task is not None and not screenshot_task.done() and not render_failed:
            for _ in range(30):
                render_without_physics() if recording_render_mode else APP.update()
                if screenshot_task.done():
                    break
        summary = {"epoch": EPOCH, "physics_frames": frame, "physics_hz": ARGS.physics_hz,
                   "initial_position": initial_positions, "initial_orientation_wxyz": initial_orientations,
                   "final_pose": last_state[0] if last_state else None,
                   "final_linear_velocity_world": last_state[1] if last_state else None,
                   "final_angular_velocity_world": last_state[2] if last_state else None,
                   "wheel_dof_velocities": array(robot.get_dof_velocities())[0].tolist(),
                   "scans_per_sensor": wire.scan_counts, "last_hits_per_sensor": wire.hit_counts,
                   "first_hits": wire.first_hits, "commands_received": wire.commands_received,
                   "imu": wire.imu_summary(),
                   "timing": wire.timing_summary(),
                   "phase_timings": phase_timings,
                   "ray_phase_timings": wire.ray_phase_timings,
                   "static_geometry_attestation": dict(expected_sha256=ARGS.static_prior_geometry_sha256,
                       cadence="every actual body state; source timestamp unchanged",
                       count=wire.geometry_audit_count, faults=wire.geometry_audit_faults,
                       fault_latched=wire.geometry_fault_latched, last_sha256=wire.geometry_last_sha256,
                       last_fault=wire.geometry_last_fault, total_wall_s=wire.geometry_audit_total_wall_s,
                       max_wall_s=wire.geometry_audit_max_wall_s),
                   "native_lidar_captures_per_sensor": lidar_capture_count,
                   "lidar_acquisition_mode": "full native snapshot on each real 10 Hz acquisition step",
                   "headless_viewport_disabled": viewport_disabled,
                   "viewport_disabled_at_physics_frame": viewport_disabled_at_frame,
                   "headless_step_mode": "native physics-only, paced at most 1x after viewport capture",
                   "recording_render": dict(enabled=recording_render_mode, requested_fps=ARGS.render_fps,
                       cadence="playing: real source clock; paused: wall clock; render calls are not completed GPU-frame counts",
                       render_calls=render_calls, max_render_wall_s=render_max_wall_s,
                       render_failed=render_failed,
                       render_advanced_physics_frames=render_advanced_physics_frames,
                       transform_sync=("existing PhysX Fabric" if render_uses_fabric else "native PhysX to USD")
                           if recording_render_mode else "inactive"),
                   "commands_rejected": wire.commands_rejected, "urdf_usd": str(asset),
                   "generated_robot_usd_sha256": generated_robot_usd_sha256,
                   "calibration_command": [ARGS.test_linear_speed, ARGS.test_angular_speed]}
        (RESULT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        carb.log_info("Wheel fixture summary: " + json.dumps(summary))
        app_utils.stop()


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # Fast Kit teardown can terminate Python before its normal exception
        # printer runs. Preserve diagnostics before closing the application.
        failure = traceback.format_exc()
        (RESULT_DIR / "failure.txt").write_text(failure)
        print(failure, file=sys.stderr, flush=True)
        raise
    finally:
        APP.close()
