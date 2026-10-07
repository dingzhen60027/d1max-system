#!/usr/bin/env python3
"""Isaac Sim 6 navigation plant: physical quadruped or legacy wheel regression.

Run with Isaac's python.sh. ROS runs in a separate Humble/Zenoh process; the
loopback wire protocol carries only measured state, ray hits and applied motion.
"""
from __future__ import annotations

import argparse
import asyncio
from collections import Counter
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

from protocol import COMMAND_PORT, STATE_PORT, decode, dynamic_packet, encode, finite_vector, imu_packet, ray_packets, state_packet

HERE = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--camera", choices=("overview", "follow"), default="follow")
    parser.add_argument("--render-fps", type=float, default=0.,
                        help="GUI recording: render-only updates at this source cadence; 0 keeps the original loop")
    parser.add_argument("--scene-file", type=Path, default=HERE / "assets/scene_config.json")
    parser.add_argument("--scene-sha256", help="Require the sealed session's exact scene specification")
    parser.add_argument("--static-prior-geometry-sha256",
                        help="Require this static collider attestation before each measured state")
    parser.add_argument('--body-envelope-json', help='Sealed full articulated-body collision query envelope')
    parser.add_argument('--session-id', help='Original navigation session identity for simulation audits')
    parser.add_argument('--clock-anchor-ns', type=int, default=0, help='Original fixed session source-clock anchor')
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
    parser.add_argument("--profile-physics-callbacks", action="store_true",
                        help="Read-only per-callback wall timing for a bounded component probe")
    parser.add_argument("--audit-native-hit-identities", action="store_true",
                        help="Bounded component probe of original native hit prim strings")
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
    if args.profile_physics_callbacks and not args.test_frames:
        parser.error("callback profiling requires a bounded --test-frames run")
    if args.audit_native_hit_identities and not args.test_frames:
        parser.error("hit identity audit requires a bounded --test-frames run")
    if args.static_prior_geometry_sha256 and (len(args.static_prior_geometry_sha256) != 64
            or any(c not in "0123456789abcdef" for c in args.static_prior_geometry_sha256)):
        parser.error("static-prior-geometry-sha256 must be a lower-case SHA256")
    return args


ARGS = parse_args()
SCENE_BYTES = ARGS.scene_file.read_bytes()
if ARGS.scene_sha256 and hashlib.sha256(SCENE_BYTES).hexdigest() != ARGS.scene_sha256:
    raise SystemExit("Scene specification does not match the sealed navigation session")
CONFIG = json.loads(SCENE_BYTES)
IS_QUADRUPED = CONFIG['robot'].get('kind') in ('quadruped', 'official_go2_physx', 'official_spot_physx')
PLANT = None
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
import omni.physx.bindings._physx as physx_bindings
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# Native worker scheduling changes throughput, not dt, solver settings,
# geometry, or policy. It is an explicit sealed fixture option. Restore the
# process's previous persistent setting before Kit teardown.
PHYSICS_CPU_THREADS_ORIGINAL = carb.settings.get_settings().get(physx_bindings.SETTING_NUM_THREADS)
PHYSICS_CPU_THREADS = CONFIG.get('physics', {}).get('cpu_threads')
if PHYSICS_CPU_THREADS is not None:
    if type(PHYSICS_CPU_THREADS) is not int or not 0 <= PHYSICS_CPU_THREADS <= 64:
        raise ValueError('invalid_sealed_physics_cpu_threads')
    carb.settings.get_settings().set(physx_bindings.SETTING_NUM_THREADS, PHYSICS_CPU_THREADS)
PHYSICS_CPU_THREADS_READBACK = carb.settings.get_settings().get(physx_bindings.SETTING_NUM_THREADS)
PHYSICS_USD_VELOCITY_WRITEBACK_ORIGINAL = carb.settings.get_settings().get(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)
PHYSICS_USD_VELOCITY_WRITEBACK = CONFIG.get('physics', {}).get('usd_velocity_writeback')
if PHYSICS_USD_VELOCITY_WRITEBACK is not None and type(PHYSICS_USD_VELOCITY_WRITEBACK) is not bool:
    raise ValueError('invalid_sealed_physics_usd_velocity_writeback')


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
    if IS_QUADRUPED:
        from world_builder import author_world
        author_world(omni.usd.get_context().get_stage(), CONFIG)
        return
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


def external_hit_mask(points, capture=None):
    """Exclude only this fixture's authored collision solids, in body coordinates.

    A self hit is dropped; its occluded ray never becomes an invented free ray.
    """
    if IS_QUADRUPED:
        pose = capture['pose']
        return PLANT.external_hit_mask(points, measured_snapshot=capture['robot_snapshot'],
            body_position=pose[:3], body_orientation_wxyz=[pose[6], *pose[3:6]])
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
    def __init__(self, robot, controller, dynamic_view=None):
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
        self.stage_geometry_verifier = None
        if ARGS.static_prior_geometry_sha256:
            from truth_map import StageGeometryVerifier
            self.stage_geometry_verifier = StageGeometryVerifier(omni.usd.get_context().get_stage(), CONFIG,
                profile_notices=ARGS.profile_physics_callbacks)
        from dynamic_collision import actor_registry, registry_digest
        self.dynamic_registry = actor_registry(CONFIG)
        self.dynamic_registry_sha256 = registry_digest(self.dynamic_registry)
        self.dynamic_samples = 0
        self.dynamic_stream = (RESULT_DIR/'dynamic_actor_readings.jsonl').open('w') if IS_QUADRUPED else None
        self.native_hit_identity_stream = ((RESULT_DIR/'native_hit_identity_audit.jsonl').open('w')
            if IS_QUADRUPED and ARGS.audit_native_hit_identities else None)
        self.last_body_certificate = None
        self.collision_audit = None
        if IS_QUADRUPED and CONFIG.get('robot_collision_registry'):
            from collision_audit import Audit
            self.collision_audit = Audit(CONFIG, ARGS.session_id or EPOCH,
                hashlib.sha256(SCENE_BYTES).hexdigest(), ARGS.clock_anchor_ns)
        self.dynamic_view = dynamic_view
        self.dynamic_index = {}
        if dynamic_view is not None:
            if not dynamic_view.is_physics_tensor_entity_valid():
                raise RuntimeError('dynamic_actor_actual_physics_view_invalid')
            native_paths = [str(path) for path in dynamic_view._physics_rigid_body_view.prim_paths]
            expected_paths = [actor['path'] for actor in self.dynamic_registry]
            if len(native_paths) != len(expected_paths) or set(native_paths) != set(expected_paths):
                raise RuntimeError('dynamic_actor_native_identity_mismatch')
            self.dynamic_index = {path: index for index, path in enumerate(native_paths)}
        self.body_envelope = json.loads(ARGS.body_envelope_json) if ARGS.body_envelope_json else None
        self.body_audit_count = self.body_audit_faults = 0
        self.floor_hit_audits = self.floor_hit_samples = 0
        self.floor_endpoint_max_abs_error_m = 0.
        self.floor_hit_stream = (RESULT_DIR/'floor_hit_audit.jsonl').open('w') if IS_QUADRUPED else None
        self.link_snapshot_stream = (RESULT_DIR/'robot_link_readings.jsonl').open('w') if IS_QUADRUPED else None
        self.lidar_capture = None

    def apply_motion(self, command):
        # The quadruped policy consumes this command only on a physics tick.
        # Wheels remain solely for reproducible historical regression.
        if not IS_QUADRUPED:
            self.robot.apply_wheel_actions(self.controller.forward(command))

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
        self.apply_motion(self.command)

    def guard_command_expiry(self, step_dt, source_start_time):
        if ARGS.test_frames and (ARGS.test_linear_speed or ARGS.test_angular_speed):
            return
        next_step_ns = round((SimulationManager.get_simulation_time() - source_start_time + step_dt) * 1e9)
        if (time.monotonic() >= self.command_until or next_step_ns >= self.command_source_deadline_ns) and np.any(self.command):
            self.command[:] = 0
            self.apply_motion(self.command)

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
        robot_snapshot = self.robot.collider_snapshot() if IS_QUADRUPED else None
        self.last_robot_snapshot = robot_snapshot
        if self.link_snapshot_stream is not None:
            # Preserve the exact same measured bundle used by full-body proof
            # and laser self-filtering. This is independent gait evidence;
            # no extra pose reads, interpolation or policy-derived state.
            self.link_snapshot_stream.write(json.dumps(dict(sim_time_ns=sim_time_ns,
                session_id=ARGS.session_id, clock_anchor_ns=ARGS.clock_anchor_ns,
                pose=pose, colliders=robot_snapshot), separators=(',', ':'), allow_nan=False)+'\n')
        metadata = None
        if ARGS.static_prior_geometry_sha256:
            # This is a geometry-domain attestation, not a lidar observation or
            # localization measurement. Preserve the actual body's source stamp.
            started = time.monotonic()
            actual_hash, fault = "", ""
            try:
                actual_hash = self.stage_geometry_verifier.verify()
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
                self.apply_motion(self.command)
            metadata = dict(static_prior_geometry_sha256=actual_hash,
                static_prior_geometry_valid=not self.geometry_fault_latched,
                static_prior_geometry_checked_sim_time_ns=sim_time_ns,
                static_prior_geometry_fault=self.geometry_last_fault)
            if self.body_envelope is not None:
                from runtime_geometry import body_certificate
                self.body_audit_count += 1
                try:
                    metadata.update(body_certificate(robot_snapshot, pose,
                        CONFIG['robot_collision_registry'], self.body_envelope, sim_time_ns))
                except (ValueError, RuntimeError) as error:
                    from dynamic_collision import canonical
                    self.body_audit_faults += 1
                    self.geometry_fault_latched = True
                    self.geometry_last_fault = str(error)[:512]
                    metadata.update(body_envelope_valid=False,
                        body_envelope_checked_sim_time_ns=sim_time_ns,
                        body_envelope_registry_sha256=hashlib.sha256(canonical(CONFIG['robot_collision_registry'])).hexdigest(),
                        body_envelope_fault=self.geometry_last_fault, static_prior_geometry_valid=False,
                        static_prior_geometry_fault=self.geometry_last_fault)
                    self.command[:] = 0
                    self.apply_motion(self.command)
                self.last_body_certificate = {k:v for k,v in metadata.items() if k.startswith('body_envelope_')}
        self.send(state_packet(EPOCH, self.state_sequence, sim_time_ns, pose, linear.tolist(), angular.tolist(), metadata=metadata))
        now_wall = time.monotonic()
        if self.state_first_wall is None:
            self.state_first_wall = now_wall
            self.state_first_source_ns = sim_time_ns
        if self.state_last_wall is not None:
            self.state_max_wall_gap = max(self.state_max_wall_gap, now_wall - self.state_last_wall)
        self.state_last_wall = now_wall
        self.state_sequence += 1
        if IS_QUADRUPED:
            samples = {}
            if self.dynamic_view is not None:
                from isaacsim.core.experimental.utils import backend
                if not self.dynamic_view.is_physics_tensor_entity_valid():
                    raise RuntimeError('dynamic_actor_actual_physics_view_invalid')
                with backend.use_backend('tensor', raise_on_unsupported=True, raise_on_fallback=True):
                    positions, orientations = [array(v) for v in self.dynamic_view.get_world_poses()]
                    dynamic_linear, _dynamic_angular = [array(v) for v in self.dynamic_view.get_velocities()]
                for actor in self.dynamic_registry:
                    index = self.dynamic_index[actor['path']]
                    w, x, y, z = orientations[index].tolist()
                    samples[actor['id']] = dict(present=True, position=positions[index].tolist(),
                        orientation_xyzw=[x, y, z, w], linear_velocity=dynamic_linear[index].tolist())
            self.send(dynamic_packet(EPOCH, self.state_sequence, sim_time_ns,
                self.dynamic_registry_sha256, samples))
            self.dynamic_stream.write(json.dumps(dict(sim_time_ns=sim_time_ns,
                registry_sha256=self.dynamic_registry_sha256, samples=samples), separators=(',', ':'))+'\n')
            if self.dynamic_samples % 50 == 0:
                self.dynamic_stream.flush()
            self.dynamic_samples += 1
            if self.collision_audit is not None:
                self.collision_audit.sample(sim_time_ns, robot_snapshot, samples)
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
            capture = self.lidar_capture if IS_QUADRUPED else None
            source_scan_ns = capture['sim_time_ns'] if capture else measured_scan_ns
            capture_pose = capture['pose'] if capture else self.last_pose
            if IS_QUADRUPED and (capture is None or capture['physics_step'] + 1 != frame['physics_step']):
                self.geometry_fault_latched = True
                self.command[:] = 0
                raise RuntimeError('native_lidar_prephysics_witness_missing_or_mismatched')
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
            if IS_QUADRUPED:
                from runtime_geometry import floor_endpoint_certificate
                hit_paths = np.asarray(lidar._lidar_sensor_interface.get_prim_data(lidar.prim_path), dtype=object)
                if hit_paths.shape != (len(cloud),):
                    self.geometry_fault_latched = True
                    self.command[:] = 0
                    raise RuntimeError('native_lidar_hit_identity_layout_changed')
                try:
                    floor_evidence = floor_endpoint_certificate(
                        np.ascontiguousarray(cloud[valid] + origin, dtype=np.float32),
                        capture_pose, hit_paths[valid], CONFIG['flat_support_contact'])
                except ValueError:
                    self.geometry_fault_latched = True
                    self.command[:] = 0
                    raise
                self.floor_hit_audits += 1
                self.floor_hit_samples += floor_evidence['native_floor_hits']
                self.floor_endpoint_max_abs_error_m = max(self.floor_endpoint_max_abs_error_m,
                    floor_evidence['floor_endpoint_max_abs_error_m'])
                self.floor_hit_stream.write(json.dumps(dict(sensor_id=sensor_id,
                    native_scan_time_ns=measured_scan_ns, scan_sequence=self.scan_sequence,
                    acquisition_begin_ns=source_scan_ns,
                    floor_path=CONFIG['flat_support_contact']['floor_path'],
                    sealed_error_bound_m=CONFIG['flat_support_contact']['floor_endpoint_error_bound_m'],
                    **floor_evidence), separators=(',', ':'))+'\n')
            points = cloud[valid] + origin
            rings = rings[valid]
            actor_ids = None
            if IS_QUADRUPED and self.dynamic_registry:
                from dynamic_collision import native_hit_actor_ids
                actor_ids = np.asarray(native_hit_actor_ids(hit_paths[valid], self.dynamic_registry), dtype=np.uint16)
            external = external_hit_mask(points, capture)
            points, rings = points[external], rings[external]
            if actor_ids is not None:
                actor_ids = np.ascontiguousarray(actor_ids[external], dtype=np.uint16)
            audited_hit_paths = None
            if self.native_hit_identity_stream is not None:
                audited_hit_paths = np.asarray([str(path) for path in hit_paths[valid][external]], dtype=np.str_)
                counts = Counter(audited_hit_paths.tolist())
                self.native_hit_identity_stream.write(json.dumps(dict(sensor_id=sensor_id,
                    acquisition_begin_ns=source_scan_ns, native_frame_time_ns=measured_scan_ns,
                    native_frame_physics_step=frame['physics_step'], capture_physics_step=capture['physics_step'],
                    registry_sha256=self.dynamic_registry_sha256, point_count=len(points),
                    unique_path_count=len(counts), original_hit_prim_counts=counts.most_common(128),
                    actor_id_counts=Counter(actor_ids.tolist()).most_common() if actor_ids is not None else []),
                    separators=(',', ':'))+'\n')
            self.ray_phase("exact_self_and_depth_filter", phase_start)
            if not len(points):
                continue
            # Preserve the native XYZ precision used by ROS PointCloud2.
            points = np.ascontiguousarray(points, dtype=np.float32)
            rings = np.ascontiguousarray(rings, dtype=np.uint16)
            phase_start = time.monotonic()
            phase_metadata = dict(native_frame_time_ns=measured_scan_ns,
                native_frame_time_s=float(frame['time']), phase='physx_prephysics_capture_v1',
                native_frame_physics_step=int(frame['physics_step']),
                capture_physics_step=capture['physics_step']) if capture else {}
            for packet in ray_packets(EPOCH, self.scan_sequence, source_scan_ns, sensor_id,
                                      origin.tolist(), points, rings=rings,
                                      actor_ids=actor_ids,
                                      actor_registry_sha256=self.dynamic_registry_sha256 if actor_ids is not None else None,
                                      **phase_metadata):
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
                pose=np.asarray(capture_pose), sim_time_ns=source_scan_ns,
                native_frame_time_ns=measured_scan_ns)
            if actor_ids is not None:
                self.last_scan_data[sensor_id].update(isaac_actor_id=actor_ids,
                    actor_registry_sha256=self.dynamic_registry_sha256)
            if audited_hit_paths is not None:
                self.last_scan_data[sensor_id]['native_hit_prim'] = audited_hit_paths
            if self.first_hits[sensor_id] is None:
                self.first_hits[sensor_id] = {"count": len(points), "body_min": points.min(axis=0).tolist(),
                                               "body_max": points.max(axis=0).tolist(),
                                               "raw_cloud_shape": raw_shape, "raw_depth_shape": depth_shape,
                                               "native_rows": native_rows, "native_cols": native_cols,
                                               "native_scan_time_ns": measured_scan_ns,
                                               "native_scan_physics_step": frame["physics_step"],
                                               "zenith": array(frame["zenith"]).tolist()}
                phase_start = time.monotonic()
                np.savez(RESULT_DIR / f"first_scan_{sensor_id}.npz", **self.last_scan_data[sensor_id])
                self.ray_phase("first_scan_evidence_save", phase_start)
        self.scan_sequence += 1

    def close(self):
        self.imu_stream.close()
        if self.dynamic_stream is not None:
            self.dynamic_stream.close()
        if self.native_hit_identity_stream is not None:
            self.native_hit_identity_stream.close()
        if self.floor_hit_stream is not None:
            self.floor_hit_stream.close()
        if self.link_snapshot_stream is not None:
            self.link_snapshot_stream.close()
        self.command[:] = 0
        self.apply_motion(self.command)
        self.sender.close()
        self.receiver.close()
        for sensor_id, capture in enumerate(self.last_scan_data):
            if capture is not None:
                np.savez(RESULT_DIR / f"last_scan_{sensor_id}.npz", **capture)


def main():
    global PLANT
    for extension in ("isaacsim.asset.importer.urdf", "isaacsim.robot.wheeled_robots", "isaacsim.sensors.physx", "isaacsim.sensors.experimental.physics"):
        app_utils.enable_extension(extension)
    APP.update()
    APP.update()
    asset = None if IS_QUADRUPED else robot_asset()
    generated_robot_usd_sha256 = hashlib.sha256(asset.read_bytes()).hexdigest() if asset else None
    stage_utils.create_new_stage()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    build_room()
    if not ARGS.headless and ARGS.render_fps:
        # Give the actual viewport room in the two-window recording layout.
        from omni import ui as omni_ui
        for name in ("Stage", "Property", "Content", "Console"):
            window = omni_ui.Workspace.get_window(name)
            if window is not None:
                window.visible = False
    robot_config = CONFIG["robot"]
    x, y, z, yaw = robot_config["initial_pose"]
    if IS_QUADRUPED:
        app_utils.enable_extension('isaacsim.robot.policy.examples')
        from quadruped import QuadrupedPlant
        quad_config = dict(robot_config, initial_position=[x, y, z], initial_yaw=yaw)
        robot = QuadrupedPlant(omni.usd.get_context().get_stage(), quad_config, RESULT_DIR)
        PLANT = robot
        body_path = robot.body_path
        asset = robot.usd_path
        generated_robot_usd_sha256 = hashlib.sha256(asset.read_bytes()).hexdigest()
    else:
        robot = WheeledRobot("/World/WheelFixture", wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                             usd_path=str(asset), positions=[[x, y, z]],
                             orientations=[[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]])
        body_path = find_link('base_link')
    APP.update()
    if not IS_QUADRUPED:
        assign_contact_materials()
    from isaacsim.sensors.physx import RotatingLidarPhysX
    lidars = []
    lidar_config = CONFIG["lidar"]
    for index, xyz in enumerate(lidar_config["origins"]):
        origin = np.asarray(xyz, dtype=float)
        lidar = RotatingLidarPhysX(
            prim_path=body_path + f"/Lidar_{index}", name=f"fixture_lidar_{index}",
            translation=origin, rotation_frequency=0.0,
            fov=(lidar_config["horizontal_fov_deg"], lidar_config["vertical_fov_deg"]),
            resolution=(lidar_config["horizontal_resolution_deg"], lidar_config["vertical_resolution_deg"]),
            valid_range=(lidar_config["range_min"], lidar_config["range_max"]))
        lidar.add_linear_depth_data_to_frame()
        lidar.add_point_cloud_data_to_frame()
        lidar.add_zenith_data_to_frame()
        if IS_QUADRUPED:
            lidar.enable_semantics()  # Native exact hit-prim identity, never synthetic labels.
        # Enroll the native capture switch before sealing the stage audit.
        # Runtime value changes are measurement gating; new schema/attributes
        # after enrollment still require a full geometry audit.
        lidar.prim.CreateAttribute("enabled", Sdf.ValueTypeNames.Bool).Set(True)
        lidars.append((lidar, origin))
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor
    imu_config = CONFIG["imu"]
    imu_sensor = IMUSensor(IMU.create(body_path + "/BodyImu",
                           translations=[imu_config["origin"]], orientations=[imu_config["orientation_wxyz"]],
                           linear_acceleration_filter_size=imu_config["linear_acceleration_filter_size"],
                           angular_velocity_filter_size=imu_config["angular_velocity_filter_size"],
                           orientation_filter_size=imu_config["orientation_filter_size"]))
    dt = 1 / ARGS.physics_hz
    SimulationManager.setup_simulation(dt=dt, device="cpu")
    SimulationManager.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
    # At 500 Hz Kit updates can advance several physics steps. Step the
    # quadruped explicitly so a 10 Hz native scan and its body pose coincide.
    recording_render_mode = IS_QUADRUPED or (not ARGS.headless and ARGS.render_fps > 0)
    effective_render_fps = ARGS.render_fps or (20. if IS_QUADRUPED else 0.)
    if recording_render_mode:
        app_utils.enable_extension("isaacsim.core.rendering_manager")
        from isaacsim.core.rendering_manager import RenderingManager
    controller = None if IS_QUADRUPED else DifferentialController(wheel_radius=robot_config["wheel_radius"], wheel_base=robot_config["wheel_base"],
                                        max_linear_speed=robot_config["max_linear_speed"],
                                        max_angular_speed=robot_config["max_angular_speed"], max_wheel_speed=12.0)
    from dynamic_collision import actor_registry
    dynamic_view = None
    if actor_registry(CONFIG):
        from isaacsim.core.experimental.prims import RigidPrim
        dynamic_view = RigidPrim([actor['path'] for actor in actor_registry(CONFIG)], reset_xform_op_properties=False)
    app_utils.play()
    from omni.physics import core as physics_core
    callback_timings = {}

    def profile_callback(name, callback):
        if not ARGS.profile_physics_callbacks:
            return callback
        def timed(*args):
            started = time.monotonic()
            try:
                return callback(*args)
            finally:
                elapsed = time.monotonic() - started
                item = callback_timings.setdefault(name, dict(count=0, total_wall_s=0., max_wall_s=0.))
                item['count'] += 1
                item['total_wall_s'] += elapsed
                item['max_wall_s'] = max(item['max_wall_s'], elapsed)
        return timed

    policy_subscription = None
    dynamic_subscription = None
    wire_holder = {'wire': None}
    if IS_QUADRUPED:
        APP.update()  # Establish the native tensor view before policy initialization.
        robot.initialize()
        policy_subscription = physics_core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=True, order=3, on_update=profile_callback('official_policy',
                lambda step_dt, _context: robot.step(step_dt,
                *(wire_holder['wire'].command if wire_holder['wire'] is not None else (0., 0.)))))
        from world_builder import update_actors, update_follow_camera
        stage = omni.usd.get_context().get_stage()
        if not ARGS.headless or ARGS.screenshot_path:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport:
                viewport.camera_path = CONFIG['cameras'][ARGS.camera]['path']
    settle_start = SimulationManager.get_num_physics_steps()
    while SimulationManager.get_num_physics_steps() - settle_start < int(2 * ARGS.physics_hz):
        SimulationManager.step(steps=1, update_fabric=False) if IS_QUADRUPED else APP.update()
    for lidar, _ in lidars:
        lidar.initialize()
    wire = WireInterface(robot, controller, dynamic_view)
    if PHYSICS_USD_VELOCITY_WRITEBACK is not None:
        # Enroll the real native properties after the original settle. Pose
        # updates stay native at 500 Hz for LiDAR/IMU. Navigation and sensor
        # velocity readers use actual tensors, without duplicate USD output.
        carb.settings.get_settings().set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, PHYSICS_USD_VELOCITY_WRITEBACK)
    wire_holder['wire'] = wire
    initial_positions, initial_orientations = [array(v)[0].tolist() for v in robot.get_world_poses()]
    source_steps = SimulationManager.get_num_physics_steps()
    source_start_time = SimulationManager.get_simulation_time()
    state_period_ns = round(1e9/CONFIG.get('physics', {}).get('state_frequency_hz', 50. if IS_QUADRUPED else ARGS.physics_hz/2))
    next_state_ns = state_period_ns if IS_QUADRUPED else 0
    if IS_QUADRUPED:
        dynamic_subscription = physics_core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=True, order=-1, on_update=profile_callback('dynamic_actor_updates',
                lambda step_dt, _context: update_actors(stage, CONFIG,
                SimulationManager.get_simulation_time()-source_start_time+step_dt)))
    # The application can render at 60 Hz while physics advances at 120 Hz.
    # Read the native IMU after each physical step so its 100 Hz source does
    # not get undersampled by viewport rendering; state is still emitted once
    # per application update (normally 60 Hz).
    imu_subscription = physics_core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=False, order=2,
        on_update=profile_callback('native_imu', lambda _dt, _context: wire.imu(imu_sensor, source_start_time)))
    command_expiry_subscription = physics_core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=True, order=0,
        on_update=profile_callback('command_expiry', lambda _dt, _context: wire.guard_command_expiry(_dt, source_start_time)))
    next_lidar_capture_ns = state_period_ns if IS_QUADRUPED else round(2 * dt * 1e9)
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
            if IS_QUADRUPED:
                # The native sensor raycasts at physics BEGIN. Its wrapper
                # increments the frame counter to END before exposing data.
                # Read actual tensors here, retain both source times, and send
                # this extra real body witness for exact ray projection.
                capture_step = SimulationManager.get_num_physics_steps() - source_steps
                capture_ns = round(capture_step * 1e9 / ARGS.physics_hz)
                captured_state = emit_state(capture_ns)
                wire.lidar_capture = dict(sim_time_ns=capture_ns,
                    physics_step=capture_step, pose=captured_state[0],
                    robot_snapshot=wire.last_robot_snapshot)
            lidar_capture_count += 1
            while next_lidar_capture_ns <= measurement_ns + 500:
                next_lidar_capture_ns += round(1e9 / lidar_config["frequency_hz"])

    lidar_gate_subscription = physics_core.get_physics_simulation_interface().subscribe_physics_on_step_events(
        pre_step=True, order=1, on_update=profile_callback('native_lidar_gate_and_begin_witness', gate_lidar_capture))
    previous_steps = source_steps
    next_scan_ns = state_period_ns if IS_QUADRUPED else 0
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
    render_period_ns = round(1e9 / effective_render_fps) if recording_render_mode else 0
    next_render_ns = render_period_ns
    next_paused_render_wall = 0.0
    render_calls = 0
    render_failed = False
    render_advanced_physics_frames = 0
    render_max_wall_s = 0.0
    render_uses_fabric = SimulationManager.is_fabric_enabled() if recording_render_mode else False
    phase_timings = {}
    trajectory = (RESULT_DIR / "trajectory.jsonl").open("w")

    def emit_state(source_ns):
        state = wire.state(source_ns)
        trajectory.write(json.dumps(dict(sim_time_ns=source_ns, pose=state[0],
            command=wire.command.tolist(), linear_velocity_world=state[1],
            angular_velocity_world=state[2], body_envelope_certificate=wire.last_body_certificate,
            policy_input_velocity=wire.robot._command.tolist() if IS_QUADRUPED else None,
            velocity_feedback_integral=wire.robot.velocity_feedback.integral.tolist() if IS_QUADRUPED else None,
            policy_joint_reading=wire.robot.policy_joint_reading(source_ns) if IS_QUADRUPED else None),
            separators=(',', ':'), allow_nan=False)+'\n')
        return state

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
        "generated_robot_usd": str(asset), "generated_robot_usd_sha256": generated_robot_usd_sha256,
        "robot_kind": robot_config.get('kind', 'wheel_fixture'),
        "robot_asset": robot.asset_metadata() if IS_QUADRUPED else None,
        "dynamic_actor_registry_sha256": wire.dynamic_registry_sha256,
        "dynamic_actor_count": len(wire.dynamic_registry)}, indent=2))
    carb.log_info(f"Navigation plant READY epoch={EPOCH}; physical joints, two PhysX LiDARs; UDP {ARGS.state_port}/{ARGS.command_port}")
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
                prior_lidar_capture_count = lidar_capture_count
                SimulationManager.step(steps=2, update_fabric=render_uses_fabric if recording_render_mode else False)
                record_phase("physics_only_two_steps", phase_start)
                if ARGS.profile_physics_callbacks:
                    record_phase('physics_two_steps_with_native_capture' if lidar_capture_count != prior_lidar_capture_count
                        else 'physics_two_steps_without_native_capture', phase_start)
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
                        next_paused_render_wall = time.monotonic() + 1 / effective_render_fps
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
            if sim_time_ns < next_state_ns:
                continue
            while next_state_ns <= sim_time_ns:
                next_state_ns += state_period_ns
            phase_start = time.monotonic()
            last_state = emit_state(sim_time_ns)
            record_phase("measured_state_and_udp", phase_start)
            wire.last_pose = last_state[0]
            if IS_QUADRUPED and ARGS.camera == 'follow':
                px, py, pz, qx, qy, qz, qw = last_state[0]
                camera_yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                update_follow_camera(stage, CONFIG, [px, py, pz, camera_yaw])
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
            if recording_render_mode and sim_time_ns >= next_render_ns and not (ARGS.headless and viewport_disabled):
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
        dynamic_subscription = None
        policy_subscription = None
        trajectory.close()
        if wire.collision_audit is not None:
            audit = wire.collision_audit.finish(RESULT_DIR/'trajectory.jsonl', completed=not wire.geometry_fault_latched)
            (RESULT_DIR/'collision_audit.json').write_text(json.dumps(audit, indent=2, allow_nan=False)+'\n')
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
                   "joint_velocities": array(robot.get_dof_velocities())[0].tolist(),
                   "robot_kind": robot_config.get('kind', 'wheel_fixture'),
                   "robot_asset": robot.asset_metadata() if IS_QUADRUPED else None,
                   "dynamic_actor_count": len(wire.dynamic_registry),
                   "dynamic_measurement_samples": wire.dynamic_samples,
                   "native_floor_hit_audit": dict(scans=wire.floor_hit_audits,
                       native_floor_hits=wire.floor_hit_samples,
                       max_abs_error_m=wire.floor_endpoint_max_abs_error_m,
                       sealed_error_bound_m=CONFIG.get('flat_support_contact', {}).get('floor_endpoint_error_bound_m'),
                       raw_endpoints_and_source_times_preserved=True),
                   "body_envelope_attestation": dict(required=wire.body_envelope is not None,
                       verified_samples=wire.body_audit_count-wire.body_audit_faults,
                       failures=wire.body_audit_faults, envelope=wire.body_envelope),
                   "scans_per_sensor": wire.scan_counts, "last_hits_per_sensor": wire.hit_counts,
                   "first_hits": wire.first_hits, "commands_received": wire.commands_received,
                   "imu": wire.imu_summary(),
                   "timing": wire.timing_summary(),
                   "phase_timings": phase_timings,
                   "physics_callback_timings": callback_timings,
                   "physics_cpu_threads": dict(requested=PHYSICS_CPU_THREADS,
                       original=PHYSICS_CPU_THREADS_ORIGINAL, readback=PHYSICS_CPU_THREADS_READBACK),
                   "physics_usd_state_writeback": dict(readback=carb.settings.get_settings().get(physx_bindings.SETTING_UPDATE_TO_USD)),
                   "physics_usd_velocity_writeback": dict(requested=PHYSICS_USD_VELOCITY_WRITEBACK,
                       original=PHYSICS_USD_VELOCITY_WRITEBACK_ORIGINAL,
                       readback=carb.settings.get_settings().get(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD)),
                   "ray_phase_timings": wire.ray_phase_timings,
                   "static_geometry_attestation": dict(expected_sha256=ARGS.static_prior_geometry_sha256,
                       cadence="every actual body state; source timestamp unchanged",
                       count=wire.geometry_audit_count, faults=wire.geometry_audit_faults,
                       fault_latched=wire.geometry_fault_latched, last_sha256=wire.geometry_last_sha256,
                       last_fault=wire.geometry_last_fault, total_wall_s=wire.geometry_audit_total_wall_s,
                       max_wall_s=wire.geometry_audit_max_wall_s,
                       full_stage_audits=wire.stage_geometry_verifier.full_audits if wire.stage_geometry_verifier else None,
                       notice_timings=wire.stage_geometry_verifier.notice_timings if wire.stage_geometry_verifier else None,
                       cache_hits=wire.stage_geometry_verifier.cache_hits if wire.stage_geometry_verifier else None),
                   "native_lidar_captures_per_sensor": lidar_capture_count,
                   "lidar_acquisition_mode": "full native snapshot on each real 10 Hz acquisition step",
                   "lidar_pose_phase": ('actual physics BEGIN witness; original native wrapper END retained'
                       if IS_QUADRUPED else 'legacy wheel snapshot'),
                   "body_state_cadence": ('50 Hz plus real 10 Hz acquisition BEGIN witnesses'
                       if IS_QUADRUPED else 'legacy wheel cadence'),
                   "headless_viewport_disabled": viewport_disabled,
                   "viewport_disabled_at_physics_frame": viewport_disabled_at_frame,
                   "headless_step_mode": "native physics-only, paced at most 1x after viewport capture",
                   "recording_render": dict(enabled=recording_render_mode, requested_fps=ARGS.render_fps,
                       effective_fps=effective_render_fps,
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
        carb.log_info("Navigation plant summary: " + json.dumps(summary))
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
        if PHYSICS_CPU_THREADS is not None:
            carb.settings.get_settings().set(physx_bindings.SETTING_NUM_THREADS, PHYSICS_CPU_THREADS_ORIGINAL)
        if PHYSICS_USD_VELOCITY_WRITEBACK is not None:
            carb.settings.get_settings().set(physx_bindings.SETTING_UPDATE_VELOCITIES_TO_USD, PHYSICS_USD_VELOCITY_WRITEBACK_ORIGINAL)
        APP.close()
