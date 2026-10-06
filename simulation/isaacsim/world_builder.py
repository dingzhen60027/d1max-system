#!/usr/bin/env python3
"""Bounded campus geometry and deterministic actors, independent of navigation.

No Isaac, ROS or NumPy dependency is imported for offline geometry checks.
Authoring imports OpenUSD lazily and creates real static and kinematic colliders.
The route helper is a conservative *offline* connectivity check, never a robot
controller, free-space certificate, or replacement for the navigation framework.
"""
from __future__ import annotations

import argparse
import copy
import heapq
import json
import math
from pathlib import Path
import re

DEFAULT_WORLD = Path(__file__).resolve().parent / "assets/large_quadruped_scene.json"
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*\Z")


def _number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("invalid_" + name)
    if positive and value <= 0:
        raise ValueError("invalid_" + name)
    return float(value)


def _vector(value, name, length=3):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("invalid_" + name)
    return tuple(_number(v, name) for v in value)


def _angle_delta(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def _box(name, center, size, kind="wall", color=None):
    result = dict(name=name, kind=kind, center=center, size=size)
    if color is not None:
        result["color"] = color
    return result


def create_large_world(seed=60027):
    """Regenerate the checked-in compact geometry description without voxels."""
    height, thickness = 3.4, .2
    boxes = [
        _box("south_wall", [0, -40.1, height / 2], [100.4, thickness, height]),
        _box("north_wall", [0, 40.1, height / 2], [100.4, thickness, height]),
        _box("west_wall", [-50.1, 0, height / 2], [thickness, 80, height]),
        _box("east_wall", [50.1, 0, height / 2], [thickness, 80, height]),
    ]
    doors = []

    # Bind the segment writer per wall; every resulting collider is a full box.
    def add_wall(name, axis, fixed, lo, hi, openings=()):
        cursor = lo
        def segment(suffix, left, right):
            center = [(left + right) / 2, fixed, height / 2] if axis == "x" else [fixed, (left + right) / 2, height / 2]
            size = [right - left, thickness, height] if axis == "x" else [thickness, right - left, height]
            boxes.append(_box(name + suffix, center, size))
        for index, (center, width) in enumerate(sorted(openings)):
            left, right = center - width / 2, center + width / 2
            if left < cursor or right > hi:
                raise ValueError("overlapping_wall_openings")
            if left > cursor:
                segment("_segment_" + str(index), cursor, left)
            xyz = [center, fixed, 2.85] if axis == "x" else [fixed, center, 2.85]
            size = [width, thickness, 1.1] if axis == "x" else [thickness, width, 1.1]
            boxes.append(_box(name + "_lintel_" + str(index), xyz, size))
            doors.append(dict(name=name + "_door_" + str(index), center=xyz[:2], width=width,
                              clear_height=2.3, opening_axis=axis))
            cursor = right
        if cursor < hi:
            segment("_segment_end", cursor, hi)

    add_wall("office_west", "y", -48, 6, 38)
    add_wall("office_north", "x", 38, -48, -14)
    add_wall("office_south", "x", 6, -48, -14, [(-31, 4)])
    add_wall("office_east", "y", -14, 6, 38, [(20, 4)])
    room_edges = [-48, -39.5, -31, -22.5, -14]
    room_centers = [-43.75, -35.25, -26.75, -18.25]
    add_wall("office_corridor_north", "x", 22, -48, -14, [(x, 1.8) for x in room_centers])
    add_wall("office_corridor_south", "x", 18, -48, -14, [(x, 1.8) for x in room_centers])
    for index, x in enumerate(room_edges[1:-1]):
        add_wall("office_north_partition_" + str(index), "y", x, 22, 38)
        add_wall("office_south_partition_" + str(index), "y", x, 6, 18)
    # The south entrance reaches the corridor through the room boundary door.
    # x=-31 lies on a partition, so its inner doorway is deliberately at -35.25.
    for index, x in enumerate(room_centers):
        boxes.append(_box("office_north_desk_" + str(index), [x + 1.7, 33, .38], [2, 1.2, .76], "desk", [.56, .37, .23]))
        boxes.append(_box("office_south_desk_" + str(index), [x + 1.7, 10, .38], [2, 1.2, .76], "desk", [.56, .37, .23]))

    add_wall("warehouse_west", "y", 14, 6, 38, [(20, 5)])
    add_wall("warehouse_east", "y", 48, 6, 38)
    add_wall("warehouse_north", "x", 38, 14, 48)
    add_wall("warehouse_south", "x", 6, 14, 48, [(32, 5)])
    for index, x in enumerate((20, 28, 36, 44)):
        for row, (y, length) in enumerate(((14.5, 7), (29, 8))):
            # Full closed rack envelopes: no unmodelled hollow support claims.
            boxes.append(_box("warehouse_rack_%s_%s" % (index, row), [x, y, 1.3], [1.4, length, 2.6], "rack", [.2, .36, .65]))
    add_wall("narrow_service_gate", "y", 12, -38, -14, [(-25, 1.4)])
    for ix, x in enumerate((-43, -36, -29, -22)):
        for iy, y in enumerate((-32, -25, -18)):
            boxes.append(_box("column_%s_%s" % (ix, iy), [x, y, height / 2], [.8, .8, height], "column", [.56, .59, .63]))
    for index, (x, y, top) in enumerate(((-45, -11, .18), (-35, -11, .3), (-25, -11, .45), (40, -32, .22), (34, -32, .4))):
        boxes.append(_box("low_obstacle_" + str(index), [x, y, top / 2], [1.2, 1.2, top], "low_obstacle", [.85, .42, .12]))
    boxes.extend([
        _box("low_clearance_beam", [33, -8, .85], [6, 3, .3], "overhead", [.95, .66, .12]),
        _box("low_clearance_post_a", [29.85, -8, .85], [.3, 3, 1.7], "column"),
        _box("low_clearance_post_b", [36.15, -8, .85], [.3, 3, 1.7], "column"),
        _box("plaza_bench_a", [-10, 31, .35], [3, .8, .7], "bench"),
        _box("plaza_bench_b", [10, 31, .35], [3, .8, .7], "bench"),
    ])
    regions = [
        dict(id="office", label="OFFICE", bounds=[-48, -14, 6, 38], color=[.33, .43, .56]),
        dict(id="office_corridor", label="CORRIDOR", bounds=[-48, -14, 18.1, 21.9], color=[.48, .59, .68]),
        dict(id="warehouse", label="WAREHOUSE", bounds=[14, 48, 6, 38], color=[.43, .40, .30]),
        dict(id="plaza", label="PLAZA", bounds=[-13, 13, -13, 37], color=[.35, .48, .39]),
        dict(id="pillar_field", label="PILLARS", bounds=[-47, -18, -36, -10], color=[.43, .36, .47]),
        dict(id="service", label="SERVICE", bounds=[14, 47, -37, 4], color=[.44, .43, .40]),
        dict(id="south_route", label="SOUTH ROUTE", bounds=[-48, 48, -39, -36.5], color=[.27, .36, .43]),
        dict(id="east_west_route", label="CONNECTOR", bounds=[-48, 48, -5, 4], color=[.35, .39, .43]),
    ]

    def actor(identifier, kind, points, shapes, enabled=True, start=0, loop=True, color=None):
        waypoints = [dict(time_s=p[0], position=list(p[1:4]), yaw=p[4]) for p in points]
        speed, acceleration, yaw_rate = 0., 0., 0.
        for a, b in zip(waypoints, waypoints[1:]):
            dt = b["time_s"] - a["time_s"]
            distance = math.dist(a["position"], b["position"])
            speed = max(speed, 1.5 * distance / dt)
            acceleration = max(acceleration, 6 * distance / (dt * dt))
            yaw_rate = max(yaw_rate, 1.5 * abs(_angle_delta(a["yaw"], b["yaw"])) / dt)
        return dict(id=identifier, kind=kind, enabled=enabled, seed=seed,
                    start_time_s=start, end_time_s=start + waypoints[-1]["time_s"] * (20 if loop else 1),
                    presence="persistent_when_enabled", trajectory=dict(mode="loop" if loop else "once", interpolation="c1_smoothstep", waypoints=waypoints),
                    analytic_max_linear_speed_mps=speed,
                    native_physics_velocity_margin_mps=.02,
                    max_linear_speed_mps=speed + .02, max_linear_acceleration_mps2=acceleration,
                    velocity_bound_scope="analytic_C1_bound_plus_explicit_0.02mps_native_f32_position_500Hz_quantization_margin_for_100x80m_campus; retain_all_measured_samples",
                    max_yaw_rate_radps=yaw_rate, collision_shapes=shapes, color=color or [.8, .35, .2])
    person = [dict(id="body", type="cylinder", center=[0, 0, .8], radius=.28, height=1.6, collision=True),
              dict(id="head", type="cylinder", center=[0, 0, 1.7], radius=.17, height=.2, collision=True)]
    cart = [dict(id="load", type="box", center=[0, 0, .5], size=[.9, .7, 1.0], collision=True),
            dict(id="wheel_left", type="cylinder", center=[-.25, -.27, .12], radius=.1, height=.24, collision=True),
            dict(id="wheel_right", type="cylinder", center=[-.25, .27, .12], radius=.1, height=.24, collision=True)]
    forklift = [dict(id="chassis", type="box", center=[0, 0, .45], size=[1.8, 1.15, .9], collision=True),
                dict(id="cab", type="box", center=[-.3, 0, 1.3], size=[1, 1.1, 1.6], collision=True),
                dict(id="fork_left", type="box", center=[1.2, -.38, .14], size=[1.0, .12, .12], collision=True),
                dict(id="fork_right", type="box", center=[1.2, .38, .14], size=[1.0, .12, .12], collision=True)]
    actors = [
        actor("plaza_person", "person", [(0, 0, -15, 0, math.pi / 2), (40, 0, 12, 0, math.pi / 2), (80, 0, -15, 0, -math.pi / 2)], person),
        actor("office_person", "person", [(0, -44, 20, 0, 0), (55, -16, 20, 0, 0), (110, -44, 20, 0, math.pi)], person, color=[.6, .3, .7]),
        actor("plaza_cart", "cart", [(0, 5, -10, 0, math.pi / 2), (35, 5, 15, 0, math.pi / 2), (70, 5, -10, 0, -math.pi / 2)], cart, color=[.2, .65, .72]),
        actor("warehouse_forklift", "forklift", [(0, 32, 9, 0, math.pi / 2), (65, 32, 35, 0, math.pi / 2), (130, 32, 9, 0, -math.pi / 2)], forklift, color=[.95, .65, .1]),
        actor("oncoming_cart", "cart", [(0, 22, -25, 0, math.pi), (40, 2, -25, 0, math.pi)], cart, enabled=False, start=5, loop=False),
        # Fixed scene-time schedule for the .15m/s demand bound. Earliest
        # straight-line arrival at the gate is ~47s, before planning delays.
        # A real encounter remains unproven if navigation instead takes a detour.
        actor("door_blocking_cart", "cart", [(0, 18, -25, 0, math.pi), (25, 12, -25, 0, math.pi), (50, 12, -25, 0, math.pi), (75, 18, -25, 0, 0)], cart, enabled=False, start=20, loop=False),
    ]
    # A loop has exactly the same pose at its seam; smoothstep stops there.
    for item in actors:
        if item["trajectory"]["mode"] == "loop":
            item["trajectory"]["waypoints"][-1]["yaw"] = item["trajectory"]["waypoints"][0]["yaw"]
            item["max_yaw_rate_radps"] = max(1.5 * abs(_angle_delta(a["yaw"], b["yaw"])) / (b["time_s"] - a["time_s"]) for a, b in zip(item["trajectory"]["waypoints"], item["trajectory"]["waypoints"][1:]))
    scenarios = [
        dict(id="long_distance_multi_goal", start=[-44, -4, .52, 0], goals=[[-43.75, 29, .52], [42, 20, .52], [24, -25, .52], [-34, -28, .52], [0, 24, .52]], actor_ids=["plaza_person", "office_person", "plaza_cart", "warehouse_forklift"], timeout_source_s=3600, checks=["all_goals_reached", "no_static_or_actor_collision", "bounded_progress", "stop_at_final_goal"]),
        dict(id="crossing_blocker", start=[-8, -5, .52, 0], goals=[[8, -5, .52]], actor_ids=["plaza_person"], timeout_source_s=360, checks=["yield_or_verified_detour", "actor_not_teleported", "goal_reached_after_crossing"]),
        dict(id="narrow_head_on", start=[5, -25, .52, 0], goals=[[20, -25, .52]], actor_ids=["oncoming_cart"], timeout_source_s=600, checks=["yield_or_alternate_route", "no_two_body_overlap_in_gate", "goal_reached"]),
        dict(id="temporary_door_block", start=[5, -25, .52, 0], goals=[[20, -25, .52]], actor_ids=["door_blocking_cart"], timeout_source_s=600, checks=["blocked_door_not_free", "wait_or_verified_detour", "goal_reached_after_release"]),
        dict(id="cancel_and_park", start=[-8, -4, .52, 0], goals=[[10, -4, .52]], actor_ids=["plaza_cart"], events=[dict(source_time_s=12, action="cancel_current_task")], timeout_source_s=60, checks=["cancel_terminal_identity", "writer_stop_ack", "measured_stationary_for_2s"]),
        dict(id="resume_after_block", start=[5, -25, .52, 0], goals=[[20, -25, .52]], actor_ids=["door_blocking_cart"], events=[dict(source_time_s=65, action="cancel_current_task"), dict(source_time_s=110, action="submit_new_task", goal=[20, -25, .52])], timeout_source_s=600, checks=["retired_task_never_resumes", "fresh_task_identity", "goal_reached", "measured_final_stop"]),
    ]
    for item in actors:
        item["scenario_ids"] = [case["id"] for case in scenarios if item["id"] in case["actor_ids"]]
    return dict(schema=1, name="large_quadruped_campus", units="m", seed=seed,
        scope=dict(ground="flat_single_level", floor_id="floor1", stairs_supported=False, cross_level_supported=False, dynamic_free_space="requires_fresh_registered_collision_oracle"),
        physics=dict(frequency_hz=500., state_frequency_hz=50.),
        room=dict(x_min=-50., x_max=50., y_min=-40., y_max=40., width=100., depth=80., height=height, wall_thickness=thickness),
        floor=dict(z=0., bounds=[-50., 50., -40., 40.],
            physics_material=dict(schema=1, path="/World/PhysicsMaterials/CampusFloor",
                static_friction=1., dynamic_friction=1., restitution=0., binding_purpose="physics")),
        ceiling=dict(z=height, thickness=.15, visible=False),
        flat_support_contact=dict(enabled=True, floor_path="/World/GroundPlane/collisionPlane", floor_z=0., penetration_allowance_m=.02,
                                  scope="closed_authorized_flat_plane_only; actual_full_leg_registry_and_bounded_dynamic_oracle_required"),
        static_boxes=boxes, doorways=doors,
        robot=dict(name="official_spot", kind="official_spot_physx", body_size=[1.1, .6, .25], body_reference_height=.52,
                   initial_pose=[-44., -4., .80, 0.], initial_spawn_height_m=.80, goal=[0., 24., .52], max_linear_speed=.15, max_angular_speed=.30,
                   offline_collision_envelope=dict(radius_m=.65, bottom_z_offset_m=-.52, top_z_offset_m=.55),
                   navigation_envelope=dict(length_m=1.25, width_m=.90, above_body_m=.55),
                   velocity_feedback_mode="spot_nonlinear_measured_v1",
                   model_limits=dict(max_linear_speed_mps=.6, max_angular_speed_radps=.8),
                   speed_limit_scope="navigation_command_0.15mps_0.30radps; isolated_PhysX_reachable_bounds_0.60mps_0.80radps; live_transport_unchanged",
                   body_size_scope="nominal navigation body dimensions; exact loaded official collision registry is authoritative",
                   stance_status="reference_height_from_official_Spot_PhysX_standing_test; offline_envelope_is_conservative_design_only",
                   observed_standing_body_z_range_m=[.478, .544],
                   terrain_contract="flat_ground_contacts_require_actual_link_contact_certificate; stairs_and_cross_level_not_supported",
                   model_binding="official_Spot_PhysX_asset; exact_runtime_collision_registry_and_contact_evidence_required",
                   spawn_clearance_basis="authored_zero_joint_foot_bottom_minus_0.692m_relative_to_body; 0.80m_spawn_leaves_about_0.108m_clearance"),
        lidar=dict(origins=[[.2, 0., .2], [-.2, 0., .2]], horizontal_fov_deg=360., vertical_fov_deg=160., horizontal_resolution_deg=1., vertical_resolution_deg=5., range_min=.06, range_max=35., frequency_hz=10., self_hit_filter="actual registered robot geometry; no synthesized ray returns"),
        imu=dict(model="Isaac 6 PhysX experimental IMUSensor", native_frequency_hz=500, frequency_hz=100, origin=[0., 0., 0.], orientation_wxyz=[1., 0., 0., 0.], include_gravity=True, linear_acceleration_filter_size=1, angular_velocity_filter_size=1, orientation_filter_size=1, output_sampling="real physics-step readings retain native source time"),
        map_generation=dict(surface_spacing_m=.10, pct_resolution_m=.10, pct_max_total_cells=10000000, pct_max_working_bytes=2000000000, static_prior_resolution_m=.05),
        regions=regions,
        signs=[dict(id=r["id"] + "_sign", text=r["label"], center=[(r["bounds"][0] + r["bounds"][1]) / 2, (r["bounds"][2] + r["bounds"][3]) / 2, .008], letter_height_m=.9) for r in regions if r["id"] != "office_corridor"] + [dict(id="gate_sign", text="GATE 1.4 M", center=[8, -22, .008], letter_height_m=.6), dict(id="clearance_sign", text="LOW CLEARANCE", center=[33, -5, .008], letter_height_m=.55)],
        lighting=dict(dome_intensity=300., sunlight_intensity=600., sunlight_rotation_xyz_deg=[-35., 20., -25.]),
        cameras=dict(overview=dict(path="/World/OverviewCamera", position=[0., 0., 120.], projection="orthographic", rotation_z_deg=180., horizontal_aperture=1440., vertical_aperture=1040., framing_basis="1280x900 viewport; horizontal conform shows about144x101.25m with >=8m margin on each side"), follow=dict(path="/World/FollowCamera", offset_robot=[-4., -6., 3.0], look_at_robot=[0., 0., .3], focal_length=22.)),
        dynamic_actors=actors, scenarios=scenarios)


def load_world(path=DEFAULT_WORLD):
    spec = json.loads(Path(path).read_text())
    validate_world(spec)
    return spec


def static_primitives(spec):
    """Exact static collider registry; ceiling stays physical when invisible."""
    result = [dict(path="/World/Indoor/" + b["name"], type="box", center=list(b["center"]), size=list(b["size"])) for b in spec["static_boxes"]]
    room, ceiling = spec["room"], spec["ceiling"]
    result.append(dict(path="/World/Indoor/ceiling", type="box", center=[0., 0., ceiling["z"] + ceiling["thickness"] / 2], size=[room["width"], room["depth"], ceiling["thickness"]]))
    return result


def actor_pose(actor, source_time_s):
    """C1 pose at original simulation time, with analytic bounded derivatives.

    Enabled actors persist before and after motion: no hiding, spawning, jumping
    to a robot-relative location, repeated integration, or wall-clock dependence.
    Positions are collider-root positions, not their bounding-box centres.
    """
    t = _number(source_time_s, "source_time")
    points = actor["trajectory"]["waypoints"]
    elapsed = t - actor["start_time_s"]
    duration = points[-1]["time_s"]
    moving = actor["enabled"] and actor["start_time_s"] <= t < actor["end_time_s"]
    if elapsed <= 0:
        point = points[0]
    elif t >= actor["end_time_s"]:
        point = points[0] if actor["trajectory"]["mode"] == "loop" else points[-1]
    else:
        if actor["trajectory"]["mode"] == "loop":
            elapsed %= duration
        if elapsed >= duration:
            point = points[-1]
        else:
            for a, b in zip(points, points[1:]):
                if a["time_s"] <= elapsed <= b["time_s"]:
                    dt = b["time_s"] - a["time_s"]
                    u = (elapsed - a["time_s"]) / dt
                    s, derivative, second = u * u * (3 - 2 * u), 6 * u * (1 - u) / dt, (6 - 12 * u) / dt**2
                    delta = [y - x for x, y in zip(a["position"], b["position"])]
                    dyaw = _angle_delta(a["yaw"], b["yaw"])
                    return dict(position=[x + s * d for x, d in zip(a["position"], delta)], yaw=a["yaw"] + s * dyaw,
                                linear_velocity=[derivative * d for d in delta], linear_acceleration=[second * d for d in delta],
                                yaw_rate=derivative * dyaw, active=bool(actor["enabled"]), motion_active=bool(moving))
            raise ValueError("invalid_trajectory_time_domain")
    return dict(position=list(point["position"]), yaw=point["yaw"], linear_velocity=[0., 0., 0.], linear_acceleration=[0., 0., 0.], yaw_rate=0., active=bool(actor["enabled"]), motion_active=False)


def actor_active_during(actor, t_begin, t_end):
    if _number(t_end, "time_end") < _number(t_begin, "time_begin"):
        raise ValueError("reversed_time_interval")
    return bool(actor["enabled"])


def actor_possible_centers_bounds(actor, t_begin, t_end):
    """Conservative root-centre bounds, suitable for an oracle UNKNOWN region.

    Smoothstep is monotone on each segment. The whole explicit path is a cheap
    conservative bound for any time interval, including loop seams and parking.
    Shape extent and bounded prediction inflation must be added by the consumer.
    """
    if not actor_active_during(actor, t_begin, t_end):
        return None
    points = [p["position"] for p in actor["trajectory"]["waypoints"]]
    return dict(min=[min(p[i] for p in points) for i in range(3)], max=[max(p[i] for p in points) for i in range(3)])


def actor_colliders(actor, source_time_s):
    pose = actor_pose(actor, source_time_s)
    if not pose["active"]:
        return []
    result, c, s = [], math.cos(pose["yaw"]), math.sin(pose["yaw"])
    for shape in actor["collision_shapes"]:
        x, y, z = shape["center"]
        item = dict(shape, path="/World/Dynamic/" + actor["id"] + "/" + shape["id"],
                    center=[pose["position"][0] + c * x - s * y, pose["position"][1] + s * x + c * y, pose["position"][2] + z], yaw=pose["yaw"])
        result.append(item)
    return result


def scenario_matrix(spec):
    return copy.deepcopy(spec["scenarios"])


def scenario_spec(spec, identifier):
    result = copy.deepcopy(spec)
    case = next((s for s in result["scenarios"] if s["id"] == identifier), None)
    if case is None:
        raise ValueError("unknown_scenario:" + identifier)
    result["robot"]["initial_pose"] = list(case["start"])
    if result["robot"]["kind"] in ("official_go2_physx", "official_spot_physx"):
        # Spawn height is an asset setting; stable body height is a measured
        # configuration field and must not be substituted for spawn clearance.
        result["robot"]["initial_pose"][2] = spec["robot"]["initial_pose"][2]
    result["robot"]["goal"] = list(case["goals"][0])
    for actor in result["dynamic_actors"]:
        actor["enabled"] = actor["id"] in case["actor_ids"]
    result["active_scenario"] = identifier
    return result


def validate_world(spec):
    if spec.get("schema") != 1 or type(spec.get("schema")) is not int or spec.get("units") != "m":
        raise ValueError("unsupported_scene_spec")
    room, floor, roof = spec["room"], spec["floor"], spec["ceiling"]
    bounds = _vector([room["x_min"], room["x_max"], room["y_min"], room["y_max"]], "room_bounds", 4)
    if not (bounds[0] < bounds[1] and bounds[2] < bounds[3]):
        raise ValueError("invalid_room_bounds")
    expected = (bounds[1] - bounds[0], bounds[3] - bounds[2], roof["z"] - floor["z"])
    if any(abs(_number(room[k], k, True) - v) > 1e-8 for k, v in zip(("width", "depth", "height"), expected)) or abs(bounds[0] + bounds[1]) > 1e-8 or abs(bounds[2] + bounds[3]) > 1e-8:
        raise ValueError("inconsistent_room_dimensions")
    if _vector(floor["bounds"], "floor_bounds", 4) != bounds:
        raise ValueError("inconsistent_floor_domain")
    material = floor.get("physics_material")
    if material is not None:
        if (material.get("schema") != 1 or material.get("path") != "/World/PhysicsMaterials/CampusFloor"
                or material.get("binding_purpose") != "physics"):
            raise ValueError("invalid_floor_physics_material")
        for key in ("static_friction", "dynamic_friction", "restitution"):
            value = _number(material[key], "floor_" + key)
            if value < 0 or (key == "restitution" and value > 1):
                raise ValueError("invalid_floor_physics_material_value")
    _number(roof["thickness"], "ceiling_thickness", True)
    names = set()
    for box in spec["static_boxes"]:
        if not _NAME.fullmatch(box["name"]) or box["name"] in names or box["name"] == "ceiling":
            raise ValueError("invalid_or_duplicate_static_name")
        names.add(box["name"])
        _vector(box["center"], "box_center")
        if any(x <= 0 for x in _vector(box["size"], "box_size")):
            raise ValueError("invalid_box_size")
    by_name = {b["name"]: b for b in spec["static_boxes"]}
    for name, axis, side in (("west_wall", 0, 0), ("east_wall", 0, 1), ("south_wall", 1, 0), ("north_wall", 1, 1)):
        if name not in by_name:
            raise ValueError("missing_room_enclosure")
        box = by_name[name]
        lo = [c - s / 2 for c, s in zip(box["center"], box["size"])]
        hi = [c + s / 2 for c, s in zip(box["center"], box["size"])]
        domain_lo, domain_hi = [bounds[0], bounds[2], floor["z"]], [bounds[1], bounds[3], roof["z"]]
        if abs((hi if side == 0 else lo)[axis] - (domain_lo if side == 0 else domain_hi)[axis]) > 1e-8 or any(lo[j] > domain_lo[j] + 1e-8 or hi[j] < domain_hi[j] - 1e-8 for j in set(range(3)) - {axis}):
            raise ValueError("incomplete_room_enclosure")
    actors = set()
    for actor in spec.get("dynamic_actors", []):
        if not _NAME.fullmatch(actor["id"]) or actor["id"] in actors or type(actor["enabled"]) is not bool:
            raise ValueError("invalid_actor_identity")
        actors.add(actor["id"])
        if actor.get("presence") != "persistent_when_enabled" or "lifetime" in actor:
            raise ValueError("unsupported_actor_lifecycle")
        trajectory = actor["trajectory"]
        if trajectory["mode"] not in ("loop", "once") or trajectory["interpolation"] != "c1_smoothstep":
            raise ValueError("unsupported_actor_trajectory")
        points = trajectory["waypoints"]
        if len(points) < 2 or points[0]["time_s"] != 0:
            raise ValueError("invalid_actor_waypoints")
        start, end = _number(actor["start_time_s"], "actor_start"), _number(actor["end_time_s"], "actor_end")
        if end - start < points[-1]["time_s"]:
            raise ValueError("invalid_actor_motion_window")
        speed = acceleration = yaw_rate = 0.
        for point in points:
            _vector(point["position"], "actor_position")
            _number(point["yaw"], "actor_yaw")
            _number(point["time_s"], "actor_waypoint_time")
        for a, b in zip(points, points[1:]):
            dt = b["time_s"] - a["time_s"]
            if dt <= 0:
                raise ValueError("nonmonotonic_actor_waypoint_time")
            distance = math.dist(a["position"], b["position"])
            speed, acceleration, yaw_rate = max(speed, 1.5 * distance / dt), max(acceleration, 6 * distance / dt**2), max(yaw_rate, 1.5 * abs(_angle_delta(a["yaw"], b["yaw"])) / dt)
        if trajectory["mode"] == "loop" and (math.dist(points[0]["position"], points[-1]["position"]) > 1e-8 or abs(_angle_delta(points[0]["yaw"], points[-1]["yaw"])) > 1e-8 or abs((end - start) / points[-1]["time_s"] - round((end - start) / points[-1]["time_s"])) > 1e-8):
            raise ValueError("discontinuous_actor_loop")
        for field, required in (("max_linear_speed_mps", speed), ("max_linear_acceleration_mps2", acceleration), ("max_yaw_rate_radps", yaw_rate)):
            if _number(actor[field], field) + 1e-9 < required:
                raise ValueError("understated_actor_motion_bound:" + field)
        if "native_physics_velocity_margin_mps" in actor:
            margin = _number(actor["native_physics_velocity_margin_mps"], "native_physics_velocity_margin_mps")
            analytic = _number(actor["analytic_max_linear_speed_mps"], "analytic_max_linear_speed_mps")
            if margin < 0 or abs(analytic-speed) > 1e-9 or actor["max_linear_speed_mps"] + 1e-9 < speed + margin:
                raise ValueError("invalid_native_actor_velocity_margin")
        shape_ids = set()
        for shape in actor["collision_shapes"]:
            if not _NAME.fullmatch(shape["id"]) or shape["id"] in shape_ids or shape.get("collision") is not True:
                raise ValueError("invalid_actor_shape_identity")
            shape_ids.add(shape["id"])
            _vector(shape["center"], "actor_shape_center")
            if shape["type"] == "box":
                if any(x <= 0 for x in _vector(shape["size"], "actor_shape_size")):
                    raise ValueError("invalid_actor_shape_size")
            elif shape["type"] == "cylinder":
                _number(shape["radius"], "actor_shape_radius", True)
                _number(shape["height"], "actor_shape_height", True)
            else:
                raise ValueError("unsupported_actor_shape")
        if not shape_ids:
            raise ValueError("empty_actor_collision_registry")
    cases = set()
    for case in spec.get("scenarios", []):
        if case["id"] in cases or not _NAME.fullmatch(case["id"]):
            raise ValueError("invalid_scenario_identity")
        cases.add(case["id"])
        _vector(case["start"], "scenario_start", 4)
        for goal in case["goals"]:
            _vector(goal, "scenario_goal")
        if not case["goals"] or not set(case["actor_ids"]).issubset(actors):
            raise ValueError("invalid_scenario_actors_or_goals")
    return dict(static_colliders=len(spec["static_boxes"]) + 2, actors=len(actors), scenarios=len(cases), flat_single_level_only=True)


def voxel_budget(spec, resolution=.05):
    """Exact closed-cell extent count used by truth_map, without allocation."""
    resolution = _number(resolution, "voxel_resolution", True)
    solids = static_primitives(spec)
    room = spec["room"]
    lower = [room["x_min"], room["y_min"], spec["floor"]["z"]]
    upper = [room["x_max"], room["y_max"], spec["ceiling"]["z"]]
    for box in solids:
        for i in range(3):
            lower[i] = min(lower[i], box["center"][i] - box["size"][i] / 2)
            upper[i] = max(upper[i], box["center"][i] + box["size"][i] / 2)
    shape = [math.floor((hi + 1e-9) / resolution) - math.ceil((lo - 1e-9) / resolution) + 2 for lo, hi in zip(lower, upper)]
    cells = math.prod(shape)
    return dict(resolution_m=resolution, shape=shape, total_cells=cells, raw_uint8_bytes=cells,
                raw_uint8_mib=cells / 2**20, one_bool_mask_bytes=cells, one_float32_volume_bytes=4 * cells,
                estimated_uint8_plus_bool_plus_float32_bytes=6 * cells,
                fits_256_mib_raw=cells <= 256 * 2**20, allocation_performed=False,
                note="Temporary-array values are explicit hypothetical dtypes, not a measured process RSS; surface/PCT grids have separate limits.")


def _shape_bounds(shape):
    if shape["type"] == "cylinder":
        half = [shape["radius"], shape["radius"], shape["height"] / 2]
    else:
        x, y, z = [s / 2 for s in shape["size"]]
        c, s = abs(math.cos(shape.get("yaw", 0))), abs(math.sin(shape.get("yaw", 0)))
        half = [c * x + s * y, s * x + c * y, z]
    return [c - h for c, h in zip(shape["center"], half)], [c + h for c, h in zip(shape["center"], half)]


def collision_names(spec, position, radius=None, bottom=None, top=None, actor_time=None):
    """Conservative cylindrical whole-body offline check, including overheads."""
    x, y, z = _vector(position, "query_position")
    envelope = spec["robot"]["offline_collision_envelope"]
    radius = envelope["radius_m"] if radius is None else _number(radius, "body_radius", True)
    bottom = z + envelope["bottom_z_offset_m"] if bottom is None else bottom
    top = z + envelope["top_z_offset_m"] if top is None else top
    collisions, room = [], spec["room"]
    if x - radius <= room["x_min"] or x + radius >= room["x_max"] or y - radius <= room["y_min"] or y + radius >= room["y_max"]:
        collisions.append("world_boundary")
    if bottom < spec["floor"]["z"] - 1e-8 or top >= spec["ceiling"]["z"]:
        collisions.append("floor_or_ceiling")
    shapes = static_primitives(spec)
    if actor_time is not None:
        shapes += [shape for actor in spec.get("dynamic_actors", []) for shape in actor_colliders(actor, actor_time)]
    for shape in shapes:
        lo, hi = _shape_bounds(shape)
        if top < lo[2] or bottom > hi[2]:
            continue
        dx, dy = max(lo[0] - x, 0., x - hi[0]), max(lo[1] - y, 0., y - hi[1])
        if dx * dx + dy * dy <= radius * radius:
            collisions.append(shape["path"])
    return collisions


def find_route(spec, start, goal, resolution=.25, radius=None):
    """Bounded conservative 8-neighbour A* used only for offline connectivity."""
    resolution = _number(resolution, "route_resolution", True)
    radius = spec["robot"]["offline_collision_envelope"]["radius_m"] if radius is None else _number(radius, "body_radius", True)
    start, goal = tuple(start[:3]), tuple(goal[:3])
    if collision_names(spec, start, radius) or collision_names(spec, goal, radius):
        raise ValueError("route_endpoint_collision")
    if abs(start[2] - goal[2]) > 1e-8:
        raise ValueError("offline_route_is_single_level")
    room = spec["room"]
    nx, ny = math.floor(room["width"] / resolution) + 1, math.floor(room["depth"] / resolution) + 1
    if nx * ny > 500000:
        raise ValueError("offline_route_grid_too_large")
    blocked = bytearray(nx * ny)
    z, envelope = start[2], spec["robot"]["offline_collision_envelope"]
    bottom, top = z + envelope["bottom_z_offset_m"], z + envelope["top_z_offset_m"]
    def index(point):
        return (round((point[0] - room["x_min"]) / resolution), round((point[1] - room["y_min"]) / resolution))
    def point(cell):
        return [room["x_min"] + cell[0] * resolution, room["y_min"] + cell[1] * resolution, z]
    # Inflate each AABB axis by half a cell in addition to the body radius.
    # This bounds every point of the cell; diagonal corner cutting is forbidden.
    inflation = radius + resolution / 2
    for box in static_primitives(spec):
        lo, hi = _shape_bounds(box)
        if top < lo[2] or bottom > hi[2]:
            continue
        ia = max(0, math.ceil((lo[0] - inflation - room["x_min"]) / resolution))
        ib = min(nx - 1, math.floor((hi[0] + inflation - room["x_min"]) / resolution))
        ja = max(0, math.ceil((lo[1] - inflation - room["y_min"]) / resolution))
        jb = min(ny - 1, math.floor((hi[1] + inflation - room["y_min"]) / resolution))
        for i in range(ia, ib + 1):
            for j in range(ja, jb + 1):
                blocked[i * ny + j] = 1
    for i in range(nx):
        for j in range(ny):
            px, py, _ = point((i, j))
            if px - inflation <= room["x_min"] or px + inflation >= room["x_max"] or py - inflation <= room["y_min"] or py + inflation >= room["y_max"]:
                blocked[i * ny + j] = 1
    origin, target = index(start), index(goal)
    if blocked[origin[0] * ny + origin[1]] or blocked[target[0] * ny + target[1]]:
        raise ValueError("route_grid_endpoint_collision")
    queue, previous, cost = [(math.dist(origin, target), 0., origin)], {}, {origin: 0.}
    while queue:
        _, distance, cell = heapq.heappop(queue)
        if distance != cost[cell]:
            continue
        if cell == target:
            route = [cell]
            while route[-1] != origin:
                route.append(previous[route[-1]])
            return [list(start)] + [point(c) for c in reversed(route)] + [list(goal)]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            neighbor = cell[0] + di, cell[1] + dj
            if not (0 <= neighbor[0] < nx and 0 <= neighbor[1] < ny) or blocked[neighbor[0] * ny + neighbor[1]]:
                continue
            if di and dj and (blocked[(cell[0] + di) * ny + cell[1]] or blocked[cell[0] * ny + cell[1] + dj]):
                continue
            candidate = distance + math.hypot(di, dj)
            if candidate < cost.get(neighbor, math.inf):
                cost[neighbor], previous[neighbor] = candidate, cell
                heapq.heappush(queue, (candidate + math.dist(neighbor, target), candidate, neighbor))
    raise ValueError("no_offline_route")


def actor_static_collisions(spec, actor, source_time_s):
    """Closed AABB overlap check; deliberately conservative for rotated boxes."""
    result = []
    for shape in actor_colliders(dict(actor, enabled=True), source_time_s):
        lo, hi = _shape_bounds(shape)
        if lo[2] < spec["floor"]["z"] - 1e-8:
            result.append((shape["path"], "floor_penetration"))
        for solid in static_primitives(spec):
            a, b = _shape_bounds(solid)
            if all(lo[i] <= b[i] and hi[i] >= a[i] for i in range(3)):
                result.append((shape["path"], solid["path"]))
    return result


def audit_world(spec):
    """Small offline report; no USD, voxel allocation, robot or ROS process."""
    report = validate_world(spec)
    collisions = []
    for actor in spec["dynamic_actors"]:
        duration = actor["trajectory"]["waypoints"][-1]["time_s"]
        samples = math.ceil(duration / .25)
        for i in range(samples + 1):
            t = actor["start_time_s"] + duration * i / samples
            for pair in actor_static_collisions(spec, actor, t):
                collisions.append(dict(actor=actor["id"], source_time_s=t, pair=pair))
                if len(collisions) >= 20:
                    break
            if len(collisions) >= 20:
                break
    report.update(actor_static_collisions=collisions, voxel_budgets=[voxel_budget(spec, r) for r in (.05, .1)], route_checks=[])
    for case in spec["scenarios"]:
        start, lengths = case["start"][:3], []
        for goal in case["goals"]:
            route = find_route(spec, start, goal)
            lengths.append(sum(math.dist(a, b) for a, b in zip(route, route[1:])))
            start = goal
        report["route_checks"].append(dict(scenario=case["id"], all_static_goals_reachable=True, segment_lengths_m=lengths))
    return report


# Pixel-vector lettering is a real visual mesh, not a GUI overlay or collider.
_FONT = {
    "A": ["01110","10001","10001","11111","10001","10001","10001"], "B": ["11110","10001","10001","11110","10001","10001","11110"],
    "C": ["01111","10000","10000","10000","10000","10000","01111"], "D": ["11110","10001","10001","10001","10001","10001","11110"],
    "E": ["11111","10000","10000","11110","10000","10000","11111"], "F": ["11111","10000","10000","11110","10000","10000","10000"],
    "G": ["01111","10000","10000","10111","10001","10001","01111"], "H": ["10001","10001","10001","11111","10001","10001","10001"],
    "I": ["11111","00100","00100","00100","00100","00100","11111"], "L": ["10000","10000","10000","10000","10000","10000","11111"],
    "M": ["10001","11011","10101","10101","10001","10001","10001"], "N": ["10001","11001","10101","10011","10001","10001","10001"],
    "O": ["01110","10001","10001","10001","10001","10001","01110"], "P": ["11110","10001","10001","11110","10000","10000","10000"],
    "R": ["11110","10001","10001","11110","10100","10010","10001"], "S": ["01111","10000","10000","01110","00001","00001","11110"],
    "T": ["11111","00100","00100","00100","00100","00100","00100"], "U": ["10001","10001","10001","10001","10001","10001","01110"],
    "W": ["10001","10001","10001","10101","10101","10101","01010"], "Y": ["10001","10001","01010","00100","00100","00100","00100"],
    "V": ["10001","10001","10001","10001","10001","01010","00100"], "Z": ["11111","00001","00010","00100","01000","10000","11111"],
    "1": ["00100","01100","00100","00100","00100","00100","01110"], "4": ["00010","00110","01010","10010","11111","00010","00010"],
    ".": ["00000","00000","00000","00000","00000","00100","00100"], " ": ["00000"] * 7,
}


def author_world(stage, spec, include_floor=True):
    """Author complete geometry into an existing stage; return exact registry.

    Cube size=1 and scale=full size. Cylinder axis=Z with authored radius/height.
    Each actor root is a kinematic rigid body with named CollisionAPI children.
    Decorations, floor paint, text, lights and cameras carry no CollisionAPI.
    Call update_actors on *every physics source-time step*, not render time.
    """
    validate_world(spec)
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, UsdShade
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    world = UsdGeom.Xform.Define(stage, "/World")
    if not stage.GetDefaultPrim():
        stage.SetDefaultPrim(world.GetPrim())
    def cube(path, center, size, color, collision=False):
        item = UsdGeom.Cube.Define(stage, path)
        item.CreateSizeAttr(1.)
        item.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        transform = UsdGeom.Xformable(item.GetPrim())
        transform.AddTranslateOp().Set(Gf.Vec3d(*center))
        # Large dimensions must retain spec accuracy for the static-domain seal.
        transform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*size))
        if collision:
            UsdPhysics.CollisionAPI.Apply(item.GetPrim()).CreateCollisionEnabledAttr(True)
        return item
    if include_floor:
        if not hasattr(UsdGeom, "Plane"):
            raise RuntimeError("OpenUSD Plane support required; caller may author its Isaac GroundPlane and pass include_floor=False")
        floor = UsdGeom.Plane.Define(stage, "/World/GroundPlane/collisionPlane")
        floor.CreateAxisAttr(UsdGeom.Tokens.z)
        floor.CreateWidthAttr(spec["room"]["width"])
        floor.CreateLengthAttr(spec["room"]["depth"])
        floor.CreateDisplayColorAttr([Gf.Vec3f(.29, .33, .36)])
        UsdGeom.Xformable(floor.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, spec["floor"]["z"]))
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim()).CreateCollisionEnabledAttr(True)
    floor_material = spec["floor"].get("physics_material")
    if floor_material is not None:
        floor_prim = stage.GetPrimAtPath("/World/GroundPlane/collisionPlane")
        if not floor_prim:
            raise ValueError("floor_must_exist_before_binding_physics_material")
        material = UsdShade.Material.Define(stage, floor_material["path"])
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(floor_material["static_friction"])
        physics_material.CreateDynamicFrictionAttr(floor_material["dynamic_friction"])
        physics_material.CreateRestitutionAttr(floor_material["restitution"])
        UsdShade.MaterialBindingAPI.Apply(floor_prim).Bind(material, materialPurpose="physics")
    registry = []
    colors = {"wall": [.68, .72, .78], "column": [.57, .6, .64], "rack": [.2, .36, .65]}
    for box in spec["static_boxes"]:
        path = "/World/Indoor/" + box["name"]
        cube(path, box["center"], box["size"], box.get("color", colors.get(box["kind"], [.8, .48, .21])), True)
        registry.append(path)
    ceiling = static_primitives(spec)[-1]
    roof = cube(ceiling["path"], ceiling["center"], ceiling["size"], [.7, .73, .77], True)
    if not spec["ceiling"]["visible"]:
        roof.MakeInvisible()
    registry.append(ceiling["path"])
    for index, region in enumerate(spec.get("regions", [])):
        x0, x1, y0, y1 = region["bounds"]
        cube("/World/Decorations/Regions/" + region["id"], [(x0 + x1) / 2, (y0 + y1) / 2, .001 + index * .0001], [x1 - x0, y1 - y0, .0001], region["color"])
    for sign in spec.get("signs", []):
        pixel = sign["letter_height_m"] / 7
        text = sign["text"]
        width = (len(text) * 6 - 1) * pixel
        x0, y0, z = sign["center"][0] - width / 2, sign["center"][1] - 3.5 * pixel, sign["center"][2]
        vertices, indices, counts = [], [], []
        for column, letter in enumerate(text):
            if letter not in _FONT:
                raise ValueError("unsupported_sign_glyph:" + letter)
            for row, bits in enumerate(_FONT[letter]):
                for cell, bit in enumerate(bits):
                    if bit != "1":
                        continue
                    x, y = x0 + (column * 6 + cell) * pixel, y0 + (6 - row) * pixel
                    address = len(vertices)
                    vertices += [Gf.Vec3f(x, y, z), Gf.Vec3f(x + pixel * .88, y, z), Gf.Vec3f(x + pixel * .88, y + pixel * .88, z), Gf.Vec3f(x, y + pixel * .88, z)]
                    indices += [address, address + 1, address + 2, address + 3]
                    counts.append(4)
        mesh = UsdGeom.Mesh.Define(stage, "/World/Decorations/Signs/" + sign["id"])
        mesh.CreatePointsAttr(vertices)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.95, .95, .85)])
        mesh.CreateDoubleSidedAttr(True)
    for actor in spec.get("dynamic_actors", []):
        if not actor["enabled"]:
            continue
        root = UsdGeom.Xform.Define(stage, "/World/Dynamic/" + actor["id"])
        transform = UsdGeom.Xformable(root.GetPrim())
        transform.AddTranslateOp().Set(Gf.Vec3d(*actor_pose(actor, 0.)["position"]))
        transform.AddRotateZOp().Set(math.degrees(actor_pose(actor, 0.)["yaw"]))
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateKinematicEnabledAttr(True)
        for shape in actor["collision_shapes"]:
            path = str(root.GetPath()) + "/" + shape["id"]
            if shape["type"] == "box":
                cube(path, shape["center"], shape["size"], actor["color"], True)
            else:
                item = UsdGeom.Cylinder.Define(stage, path)
                item.CreateRadiusAttr(shape["radius"])
                item.CreateHeightAttr(shape["height"])
                item.CreateAxisAttr(UsdGeom.Tokens.z)
                item.CreateDisplayColorAttr([Gf.Vec3f(*actor["color"])])
                UsdGeom.Xformable(item.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*shape["center"]))
                UsdPhysics.CollisionAPI.Apply(item.GetPrim()).CreateCollisionEnabledAttr(True)
            registry.append(path)
    lighting = spec["lighting"]
    UsdLux.DomeLight.Define(stage, "/World/CampusDomeLight").CreateIntensityAttr(lighting["dome_intensity"])
    sun = UsdLux.DistantLight.Define(stage, "/World/CampusSunLight")
    sun.CreateIntensityAttr(lighting["sunlight_intensity"])
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(*lighting["sunlight_rotation_xyz_deg"]))
    overview = spec["cameras"]["overview"]
    camera = UsdGeom.Camera.Define(stage, overview["path"])
    camera.CreateProjectionAttr(UsdGeom.Tokens.orthographic)
    camera.CreateHorizontalApertureAttr(overview["horizontal_aperture"])
    camera.CreateVerticalApertureAttr(overview["vertical_aperture"])
    camera.CreateClippingRangeAttr(Gf.Vec2f(.1, 300))
    overview_xform = UsdGeom.Xformable(camera.GetPrim())
    overview_xform.AddTranslateOp().Set(Gf.Vec3d(*overview["position"]))
    overview_xform.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(overview.get("rotation_z_deg", 0.))
    follow = UsdGeom.Camera.Define(stage, spec["cameras"]["follow"]["path"])
    follow.CreateFocalLengthAttr(spec["cameras"]["follow"]["focal_length"])
    follow.CreateClippingRangeAttr(Gf.Vec2f(.1, 200))
    UsdGeom.Xformable(follow.GetPrim()).AddTransformOp()
    update_follow_camera(stage, spec, spec["robot"]["initial_pose"])
    return dict(static_paths=[p["path"] for p in static_primitives(spec)], dynamic_paths=[path for path in registry if path.startswith("/World/Dynamic/")], floor_path="/World/GroundPlane/collisionPlane", camera_paths=[overview["path"], spec["cameras"]["follow"]["path"]])


def update_actors(stage, spec, source_time_s):
    from pxr import Gf, UsdGeom
    for actor in spec.get("dynamic_actors", []):
        if not actor["enabled"]:
            continue
        pose = actor_pose(actor, source_time_s)
        prim = stage.GetPrimAtPath("/World/Dynamic/" + actor["id"])
        if not prim:
            raise ValueError("missing_registered_actor:" + actor["id"])
        ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
        ops["xformOp:translate"].Set(Gf.Vec3d(*pose["position"]))
        ops["xformOp:rotateZ"].Set(math.degrees(pose["yaw"]))


def update_follow_camera(stage, spec, robot_pose):
    from pxr import Gf, UsdGeom
    x, y, z, yaw = _vector(robot_pose, "robot_camera_pose", 4)
    camera = spec["cameras"]["follow"]
    def world(local):
        a, b, c = local
        return Gf.Vec3d(x + a * math.cos(yaw) - b * math.sin(yaw), y + a * math.sin(yaw) + b * math.cos(yaw), z + c)
    matrix = Gf.Matrix4d().SetLookAt(world(camera["offset_robot"]), world(camera["look_at_robot"]), Gf.Vec3d(0, 0, 1)).GetInverse()
    prim = stage.GetPrimAtPath(camera["path"])
    if not prim:
        raise ValueError("missing_follow_camera")
    UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(matrix)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--write-default", action="store_true", help="Write compact reproducible JSON; never voxels")
    parser.add_argument("--audit", action="store_true", help="Check actor/static collisions and scenario connectivity offline")
    args = parser.parse_args()
    if args.write_default:
        if args.spec.exists():
            raise SystemExit("Refusing to overwrite existing world: " + str(args.spec))
        args.spec.parent.mkdir(parents=True, exist_ok=True)
        args.spec.write_text(json.dumps(create_large_world(), indent=2) + "\n")
    spec = load_world(args.spec)
    result = audit_world(spec) if args.audit else dict(validation=validate_world(spec), voxel_budgets=[voxel_budget(spec, r) for r in (.05, .1)])
    print(json.dumps(result, indent=2))
    if result.get("actor_static_collisions"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
