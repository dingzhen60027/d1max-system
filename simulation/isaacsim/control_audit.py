#!/usr/bin/env python3
"""Read-only, source-indexed control evidence; never navigation acceptance.

Stream smoke's JSONL, retaining original wire geometry and source stamps.
De Boor/derivatives follow fixed_bspline_sampler.hpp and UniformBspline.
Missing historical controls, exact bodies or writer facts remain pending.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, OrderedDict
import hashlib
import json
import math
from pathlib import Path


VERSION_FIELDS = ("session_id", "task_id", "route_id", "route_hash", "map_version_id",
                  "localization_epoch", "localization_seed_id", "reference_generation",
                  "segment_id", "anchor_id", "anchor_revision", "context_sequence",
                  "map_geometry_revision")


def stamp_ns(value):
    if isinstance(value, dict) and type(value.get("sec")) is int and type(value.get("nanosec")) is int:
        return value["sec"] * 1_000_000_000 + value["nanosec"]
    return None


def vector(value):
    result = [value[k] for k in ("x", "y", "z")] if isinstance(value, dict) else list(value)
    if len(result) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in result):
        raise ValueError("nonfinite_or_incomplete_xyz")
    return result


def norm(v):
    return math.sqrt(math.fsum(x*x for x in v))


def difference(a, b):
    return [x-y for x, y in zip(a, b)]


def yaw(orientation):
    # Match tracker extraction from the received quaternion; never normalize it.
    if isinstance(orientation, dict):
        x, y, z, w = [orientation[k] for k in ("x", "y", "z", "w")]
    else:
        x, y, z, w = orientation
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def identity(value, trajectory_id=None, tagged=False):
    version = dict(value)
    if tagged:
        version["reference_generation"] = value.get("generation")
        trajectory_id = value.get("trajectory", {}).get("traj_id")
    return tuple(version.get(k) for k in VERSION_FIELDS) + (trajectory_id,)


def identity_value(key):
    return dict(zip(VERSION_FIELDS+("trajectory_id",), key))


def observed_identity_matches(observed, complete):
    return complete is not None and observed[-1] is not None and all(
        a is None or a == b for a, b in zip(observed, complete))


class ExactSpline:
    """Same strictly increasing native knot contract, with full XYZ controls."""
    def __init__(self, points, knots, degree):
        self.points = [vector(p) for p in points]
        self.knots = list(knots)
        self.degree = degree
        self._derivative = None
        self._arc_table = None
        if (type(degree) is not int or not 0 <= degree <= 3 or len(self.points) <= degree
                or len(self.points) > 10000 or len(knots) != len(points)+degree+1
                or any(type(k) not in (int, float) or not math.isfinite(k) for k in knots)
                or any(b-a <= 1e-9 for a, b in zip(knots, knots[1:]))):
            raise ValueError("not_native_strict_bspline")
        self.duration = knots[-degree-1]-knots[degree]

    def evaluate(self, time):
        if not math.isfinite(time):
            raise ValueError("nonfinite_curve_time")
        p, knots = self.degree, self.knots
        u = min(max(knots[p], time+knots[p]), knots[-p-1])
        k = bisect_left(knots, u, p+1, len(self.points)+1)-1
        values = [list(v) for v in self.points[k-p:k+1]]
        for r in range(1, p+1):
            for i in range(p, r-1, -1):
                alpha = (u-knots[i+k-p])/(knots[i+1+k-r]-knots[i+k-p])
                values[i] = [(1-alpha)*a+alpha*b for a, b in zip(values[i-1], values[i])]
        return values[p]

    def derivative(self):
        if self._derivative is not None:
            return self._derivative
        p, knots = self.degree, self.knots
        if p == 0:
            raise ValueError("constant_curve_has_no_recorded_derivative")
        points = [[p*(b[d]-a[d])/(knots[i+p+1]-knots[i+1]) for d in range(3)]
                  for i, (a, b) in enumerate(zip(self.points, self.points[1:]))]
        self._derivative = ExactSpline(points, knots[1:-1], p-1)
        return self._derivative

    def samples_at(self, time):
        velocity = self.derivative()
        v = velocity.evaluate(time)
        return dict(position_xyz=self.evaluate(time), velocity_xyz_mps=v,
                    full_xyz_speed_mps=norm(v))

    def control_lookahead_time(self, time, speed, horizon, spatial):
        if not spatial:
            return min(self.duration,time+horizon)
        if self._arc_table is None:
            n=max(40,min(6000,math.ceil(self.duration/.02)))
            times=[self.duration*i/n for i in range(n+1)]
            points=[self.evaluate(t) for t in times]
            arcs=[0.]
            for a,b in zip(points,points[1:]):
                arcs.append(arcs[-1]+norm(difference(b,a)))
            self._arc_table=times,arcs
        times,arcs=self._arc_table
        i=bisect_left(times,time)
        current=arcs[0] if i==0 else arcs[-1] if i==len(times) else (
            arcs[i-1]+(arcs[i]-arcs[i-1])*(time-times[i-1])/(times[i]-times[i-1]))
        target=min(arcs[-1],current+speed*horizon)
        if target>=arcs[-1]:
            return self.duration
        j=bisect_left(arcs,target)
        if j==0:
            return times[0]
        if j==len(arcs):
            return self.duration
        span=arcs[j]-arcs[j-1]
        return times[j-1]+(times[j]-times[j-1])*(target-arcs[j-1])/span if span>0 else times[j]


class ControlAudit:
    def __init__(self, emit, *, kp_position=.8, heading_threshold=.6, lookahead=.8,
                 spatial_control_lookahead=False,
                 command_speed_limit=None, command_yaw_limit=None):
        self.emit = emit
        self.kp, self.heading_threshold = kp_position, heading_threshold
        self.lookahead = lookahead
        self.spatial_control_lookahead=spatial_control_lookahead
        self.command_speed_limit, self.command_yaw_limit = command_speed_limit, command_yaw_limit
        self.curves, self.ambiguous, self.grants = {}, set(), {}
        self.bodies, self.pending = OrderedDict(), OrderedDict()
        self.ambiguous_bodies = set()
        self.installed, self.writer_active = None, None
        self.last_installation, self.commands, self.credit_counts = None, {}, {}
        self.route, self.route_segments = None, []
        self.counts, self.pending_counts = Counter(), Counter()
        self.line = 0

    def event(self, kind, record, **data):
        self.counts[kind] += 1
        self.emit(dict(kind=kind, trace_line=self.line,
                       source_clock_ns=record.get("source_clock_ns"),
                       receipt_monotonic_ns=record.get("receipt_monotonic_ns"), **data))

    def body(self, source, position, orientation, frame, velocity=None):
        if source is None:
            return
        old = self.bodies.get(source)
        current = dict(position=vector(position), yaw=yaw(orientation),
            frame=frame if frame is not None else old["frame"] if old else None,
            velocity=vector(velocity) if velocity is not None else
            old["velocity"] if old else None)
        if old and (any(old[k] != current[k] for k in ("position", "yaw")) or
                    old["frame"] is not None and current["frame"] is not None and old["frame"] != current["frame"]):
            self.ambiguous_bodies.add(source)
        if old and velocity is not None and old["velocity"] is not None and old["velocity"] != current["velocity"]:
            self.ambiguous_bodies.add(source)
        self.bodies[source] = current
        self.bodies.move_to_end(source)
        while len(self.bodies) > 4096:
            self.bodies.popitem(last=False)
        for pending_id, (record, data, line) in list(self.pending.items()):
            if data.get("control_diagnostic", {}).get("body_source_stamp_ns") == source:
                previous = self.line
                self.line = line
                self.control(record, data, defer=False, resolved=True)
                self.line = previous
                del self.pending[pending_id]

    def curve(self, record, data):
        wire = data.get("trajectory", {})
        key = identity(data, tagged=True)
        try:
            curve = ExactSpline(wire["pos_pts"], wire["knots"], wire["order"])
            if curve.degree < 1:
                raise ValueError("position_curve_degree_zero")
        except (KeyError, ValueError, TypeError) as error:
            self.event("curve_evidence_pending", record, identity=identity_value(key), reason=str(error))
            return
        geometry = dict(order=curve.degree, knots=curve.knots, points_xyz=curve.points,
                        frame_id=data.get("frame_id"), point_reference=data.get("point_reference"),
                        curve_start_source_ns=stamp_ns(wire.get("start_time")))
        digest = hashlib.sha256(json.dumps(geometry, sort_keys=True, allow_nan=False).encode()).hexdigest()
        previous = self.curves.get(key)
        if previous and previous["sha256"] != digest:
            self.ambiguous.add(key)
            self.event("conflicting_curve_identity", record, identity=identity_value(key),
                       previous_sha256=previous["sha256"], received_sha256=digest)
            return
        if not previous or record.get("kind") not in previous["wire_by_lane"]:
            if not previous:
                self.curves[key] = dict(curve=curve, wire=data, sha256=digest, wire_by_lane={})
            self.curves[key]["wire_by_lane"][record.get("kind")] = data
            if record.get("kind") == "execution_spline":
                self.curves[key]["wire"] = data
            velocity = curve.derivative()
            entry = data.get("valid_start_time")
            facts = dict(identity=identity_value(key), geometry_sha256=digest,
                         original_full_geometry=geometry,
                         observed_curve_lane=record.get("kind"),
                         curve_start_source_ns=stamp_ns(wire.get("start_time")),
                         join_source_stamp_ns=stamp_ns(data.get("join_source_stamp")),
                         duration_s=curve.duration, control_points=len(curve.points),
                         velocity_control_hull_upper_bound_mps=max(map(norm, velocity.points)))
            if type(entry) in (int, float) and math.isfinite(entry):
                facts["original_entry"] = curve.samples_at(entry)
                facts["original_entry"]["curve_time"] = entry
                if data.get("join_pose") and data.get("join_twist"):
                    facts["original_entry"]["recorded_join_position_xyz"] = vector(data["join_pose"]["position"])
                    facts["original_entry"]["recorded_join_velocity_xyz_mps"] = vector(data["join_twist"]["linear"])
            self.event("complete_curve_observed", record, **facts)

    def matching_curve(self, trajectory_id, start_ns=None, key=None, partial=False):
        if key is not None:
            if not partial:
                return None if key in self.ambiguous else self.curves.get(key)
            matches = [value for k, value in self.curves.items() if k not in self.ambiguous
                       and observed_identity_matches(key, k)]
            return matches[0] if len(matches) == 1 else None
        matches = [value for k, value in self.curves.items() if k not in self.ambiguous
                   and k[-1] == trajectory_id and (start_ns is None or
                   stamp_ns(value["wire"]["trajectory"].get("start_time")) == start_ns)]
        return matches[0] if len(matches) == 1 else None

    def control(self, record, data, *, defer=True, resolved=False):
        diag = data.get("control_diagnostic", {})
        if not diag.get("geometry_available"):
            self.event("tracker_state", record, reason=data.get("reason"),
                       prepare_reason=data.get("prepare_reason"), geometry_available=False)
            return
        curve = self.matching_curve(data.get("trajectory_id"), diag.get("installed_trajectory_start_ns"))
        body_ns = diag.get("body_source_stamp_ns")
        body = None if body_ns in self.ambiguous_bodies else self.bodies.get(body_ns)
        facts = dict(reason=data.get("reason"), prepare_reason=data.get("prepare_reason"),
                     trajectory_id=data.get("trajectory_id"), body_source_stamp_ns=body_ns,
                     step_source_ns=diag.get("step_source_ns"),
                     projected_curve_time=diag.get("projected_curve_time"),
                     lookahead_curve_time=diag.get("lookahead_curve_time"),
                     recorded_lookahead_position_xyz=diag.get("lookahead_position"),
                     recorded_lookahead_velocity_xyz_mps=diag.get("lookahead_velocity"),
                     planar_speed_limit_mps=diag.get("planar_speed_limit_mps"),
                     planar_acceleration_limit_mps2=diag.get("planar_acceleration_limit_mps2"),
                     spatial_control_lookahead=diag.get("spatial_control_lookahead",False),
                     lookahead_from_xyz_arc_m=diag.get("lookahead_from_xyz_arc_m"),
                     lookahead_target_xyz_arc_m=diag.get("lookahead_target_xyz_arc_m"),
                     lookahead_requested_distance_m=diag.get("lookahead_requested_distance_m"),
                     resolved_from_later_receipt_same_source=resolved)
        missing = []
        if curve is None:
            missing.append("complete_unambiguous_curve")
        else:
            for prefix, field in (("projected", "projected_curve_time"), ("lookahead", "lookahead_curve_time")):
                value = diag.get(field)
                if type(value) in (int, float) and math.isfinite(value):
                    facts[prefix] = curve["curve"].samples_at(value)
            if "lookahead" in facts:
                for field, result in (("lookahead_position", "position_xyz"), ("lookahead_velocity", "velocity_xyz_mps")):
                    if diag.get(field) is not None:
                        facts[field+"_evaluation_residual_xyz"] = difference(facts["lookahead"][result], vector(diag[field]))
        if body is None:
            missing.append("conflicting_same_source_bodies" if body_ns in self.ambiguous_bodies else "exact_original_body_source")
            if defer and body_ns is not None:
                self.pending[self.line] = (record, data, self.line)
                if len(self.pending) > 1024:
                    self.pending.popitem(last=False)
                    self.pending_counts["bounded_pending_body_queue_eviction"] += 1
        else:
            facts["measured_body_position_xyz"] = body["position"]
            facts["measured_body_velocity_xyz_mps"] = body["velocity"]
            facts["measured_body_yaw_rad"] = body["yaw"]
            look_position, look_velocity = diag.get("lookahead_position"), diag.get("lookahead_velocity")
            if curve and body["frame"] != curve["wire"].get("frame_id"):
                missing.append("matching_body_and_curve_frame")
            elif look_position is not None and look_velocity is not None:
                # Controller uses XY; full XYZ geometry/measurements stay above.
                look_position, look_velocity = vector(look_position), vector(look_velocity)
                at_endpoint = diag.get("lookahead_curve_time") == diag.get("curve_duration")
                world = [self.kp*(look_position[i]-body["position"][i])+
                         (0. if at_endpoint else look_velocity[i]) for i in range(2)]
                desired = math.atan2(world[1], world[0]) if math.hypot(*world) > 1e-5 else body["yaw"]
                error = (desired-body["yaw"]+math.pi)%(2*math.pi)-math.pi
                facts.update(controller_world_xy_mps=world, controller_yaw_error_rad=error,
                             original_heading_gate="outside" if abs(error)>self.heading_threshold else "inside")
        facts["pending_evidence"] = missing
        self.event("tracker_curve_control", record, **facts)

    def route_progress(self, record, data):
        source = data.get("body_source_stamp_ns")
        position, orientation = data.get("position"), data.get("orientation_xyzw")
        if position is not None and orientation is not None:
            # RouteProgress.frame_id names the fixed ground route, not its
            # odom_body_pose. Older flattened traces omit that pose's frame.
            self.body(source, position, orientation, data.get("odom_body_frame_id"))
        route_key = tuple(data.get(k) for k in ("frame_id", "reference_generation", "segment_id", "anchor_id",
                         "anchor_revision", "context_sequence", "map_geometry_revision"))
        transform = (tuple(data.get("map_from_odom_position", [])), tuple(data.get("map_from_odom_orientation_xyzw", [])))
        key = (route_key, transform)
        if self.route is None or self.route["key"] != key:
            self.route = dict(key=key, first_source_ns=source, last_source_ns=source,
                              first_xyz=position, last_xyz=position,
                              first_confirmed_arc_m=data.get("confirmed_arc_m"),
                              last_confirmed_arc_m=data.get("confirmed_arc_m"),
                              first_measured_arc_m=data.get("measured_arc_m"),
                              last_measured_arc_m=data.get("measured_arc_m"),
                              confirmed_arc_net_m=0., measured_arc_net_m=0.,
                              displacement_scope="original_odom_body_pose_with_fixed_route_identity_and_anchor",
                              cumulative_xy_m=0., cumulative_xyz_m=0., net_xy_m=0., net_xyz_m=0.)
            self.route_segments.append(self.route)
            self.event("fixed_route_evidence_domain", record, route_identity=list(route_key),
                       body_source_stamp_ns=source, reason="new_observed_identity_or_transform")
        elif source is not None and source > self.route["last_source_ns"] and position is not None:
            delta = difference(position, self.route["last_xyz"])
            net = difference(position, self.route["first_xyz"])
            self.route.update(last_source_ns=source, last_xyz=position,
                              last_confirmed_arc_m=data.get("confirmed_arc_m"),
                              last_measured_arc_m=data.get("measured_arc_m"),
                              cumulative_xy_m=self.route["cumulative_xy_m"]+math.hypot(*delta[:2]),
                              cumulative_xyz_m=self.route["cumulative_xyz_m"]+norm(delta),
                              net_xy_m=math.hypot(*net[:2]), net_xyz_m=norm(net))
            for field in ("confirmed", "measured"):
                first, last = self.route[f"first_{field}_arc_m"], self.route[f"last_{field}_arc_m"]
                self.route[f"{field}_arc_net_m"] = last-first if first is not None and last is not None else None

    def feed(self, record):
        self.line += 1
        data = record.get("data", {})
        data = data.get("wire", data)
        kind = record.get("kind")
        if kind in ("native_spline", "execution_spline"):
            self.curve(record, data)
        elif kind == "geometry_receipt":
            key = identity(data.get("version", {}), data.get("trajectory_id"))
            fact = (key, data.get("installation_sequence"), data.get("installed_at"), data.get("installed"), data.get("reason"))
            if fact != self.last_installation:
                self.last_installation = fact
                if data.get("installed") is True:
                    self.installed = key
                self.event("geometry_installation_fact", record, identity=identity_value(key),
                           installation_sequence=data.get("installation_sequence"),
                           installed_at_source_ns=stamp_ns(data.get("installed_at")),
                           installed=data.get("installed"), reason=data.get("reason"))
        elif kind == "handoff_grant":
            self.grants[data.get("handoff_id")] = data
        elif kind == "commit_ack":
            key = identity(data.get("candidate_version", {}), data.get("candidate_trajectory_id"))
            complete = "candidate_version" in data and "candidate_trajectory_id" in data
            if data.get("applied") is True and complete:
                self.writer_active = key
            entry_facts = {}
            curve = self.matching_curve(data.get("candidate_trajectory_id"), key=key) if complete else None
            if curve and type(data.get("curve_time")) in (int, float):
                entry_facts["candidate_at_writer_curve_time"] = curve["curve"].samples_at(data["curve_time"])
                can_compare_lead=not self.spatial_control_lookahead or self.command_speed_limit is not None
                lead_time=curve["curve"].control_lookahead_time(data["curve_time"],self.command_speed_limit,
                    self.lookahead,self.spatial_control_lookahead) if can_compare_lead else None
                if lead_time is not None:
                    entry_facts["candidate_lookahead"] = curve["curve"].samples_at(lead_time)
                    entry_facts["candidate_lookahead_curve_time"]=lead_time
                if data.get("measured_pose") and data.get("measured_twist"):
                    entry_facts["writer_measured_pose"] = data["measured_pose"]
                    entry_facts["writer_measured_twist"] = data["measured_twist"]
                    entry_facts["writer_body_source_stamp_ns"] = stamp_ns(data.get("body_source_stamp"))
                    entry_facts["curve_minus_writer_body_xyz"] = difference(
                        entry_facts["candidate_at_writer_curve_time"]["position_xyz"],
                        vector(data["measured_pose"]["pose"]["position"]))
                    entry_facts["curve_minus_writer_velocity_xyz_mps"] = difference(
                        entry_facts["candidate_at_writer_curve_time"]["velocity_xyz_mps"],
                        vector(data["measured_twist"]["linear"]))
                    if lead_time is not None and data["measured_pose"].get("header", {}).get("frame_id") == curve["wire"].get("frame_id"):
                        body_pose = data["measured_pose"]["pose"]
                        body_yaw = yaw(body_pose["orientation"])
                        lead = entry_facts["candidate_lookahead"]
                        body_position = vector(body_pose["position"])
                        endpoint = lead_time >= curve["curve"].duration
                        world = [self.kp*(lead["position_xyz"][i]-body_position[i])+
                            (0. if endpoint else lead["velocity_xyz_mps"][i]) for i in range(2)]
                        desired = math.atan2(world[1],world[0]) if math.hypot(*world)>1e-5 else body_yaw
                        error = (desired-body_yaw+math.pi)%(2*math.pi)-math.pi
                        entry_facts.update(candidate_controller_world_xy_mps=world,
                            candidate_controller_yaw_error_rad=error,
                            candidate_original_heading_gate="outside" if abs(error)>self.heading_threshold else "inside")
            self.event("writer_software_commit_fact", record, handoff_id=data.get("handoff_id"),
                       applied=data.get("applied"), write_submitted=data.get("write_submitted"),
                       write_acknowledged=data.get("write_acknowledged"), reason=data.get("reason"),
                       commit_sequence=data.get("commit_sequence"),
                       candidate_identity=identity_value(key) if complete else None,
                       applied_at_source_ns=stamp_ns(data.get("applied_at")) or data.get("applied_at_ns"),
                       curve_time=data.get("curve_time"), applied_velocity=data.get("applied_velocity"),
                       pending_evidence=[] if complete else ["full_writer_ack_identity"],
                       scope="irreversible_writer_software_selection_not_physical_motion_proof", **entry_facts)
        elif kind == "tracking_progress":
            source = stamp_ns(data.get("header", {}).get("stamp"))
            pose, twist = data.get("pose", {}), data.get("twist", {})
            self.body(source, pose["position"], pose["orientation"], data.get("header", {}).get("frame_id"), twist.get("linear"))
            # TrackingProgress.generation is the reference_generation on wire.
            value = dict(data, reference_generation=data.get("generation"))
            key = identity(value, data.get("trajectory_id"))
            curve = self.matching_curve(data.get("trajectory_id"), key=key, partial=True)
            facts = dict(identity=identity_value(key), body_source_stamp_ns=source,
                         curve_time=data.get("curve_time"), valid=data.get("valid"), holding=data.get("holding"),
                         reason=data.get("reason"), measured_pose_xyz=vector(pose["position"]),
                         measured_velocity_xyz_mps=vector(twist["linear"]),
                         same_installed_observed_identity=observed_identity_matches(key, self.installed),
                         same_writer_selected_observed_identity=observed_identity_matches(key, self.writer_active),
                         identity_match_scope="only_fields_present_in_original_tracking_progress")
            if curve and type(data.get("curve_time")) in (int, float):
                facts["curve"] = curve["curve"].samples_at(data["curve_time"])
                facts["curve_position_minus_body_xyz"] = difference(facts["curve"]["position_xyz"], facts["measured_pose_xyz"])
                facts["curve_velocity_minus_body_xyz_mps"] = difference(facts["curve"]["velocity_xyz_mps"], facts["measured_velocity_xyz_mps"])
            else:
                facts["pending_evidence"] = ["complete_unambiguous_curve"]
            self.event("source_measured_curve_progress", record, **facts)
        elif kind == "route_progress":
            self.route_progress(record, data)
        elif kind == "tracker":
            self.control(record, data)
        elif kind == "bt_progress":
            domain = (data.get("execution_id"), data.get("control_epoch"))
            count = data.get("execution_progress_credit_samples")
            previous = self.credit_counts.get(domain)
            if type(count) is int:
                self.credit_counts[domain] = count
            if count != previous or data.get("execution_progress_reason"):
                self.event("owner_progress_credit_fact", record, **data,
                           observed_credit_increment=count-previous if type(count) is int and previous is not None else None,
                           scope="original_owner_count_only_never_inferred_from_gait_distance")
        elif kind in ("demand", "safe_demand", "applied"):
            velocity = data.get("velocity")
            if isinstance(velocity, dict):
                forward, angular = velocity["linear"]["x"], velocity["angular"]["z"]
            elif isinstance(velocity, list) and len(velocity) == 2:
                forward, angular = velocity
            else:
                self.event("command_evidence_pending", record, lane=kind, reason="missing_recorded_command")
                return
            previous = self.commands.get(kind)
            self.commands[kind] = (forward, angular)
            flags = []
            if previous and previous[0] > 0 and forward == 0:
                flags.append("positive_forward_to_exact_zero")
            if forward < 0:
                flags.append("negative_forward_command")
            for name, actual, limit in (("configured_forward_limit_exceeded", abs(forward), self.command_speed_limit),
                                        ("configured_yaw_limit_exceeded", abs(angular), self.command_yaw_limit)):
                if limit is not None and actual > limit:
                    flags.append(name)
            # Existing native/SDK braking-model command ceilings; these are
            # diagnostic comparisons, not a new permit or task pass criterion.
            if abs(forward) > .3:
                flags.append("native_model_command_speed_ceiling_exceeded")
            if abs(angular) > .5:
                flags.append("native_model_command_yaw_ceiling_exceeded")
            if previous != (forward, angular) or flags:
                self.event("recorded_command_change", record, lane=kind, forward_mps=forward, yaw_radps=angular,
                           previous_command=previous, original_velocity=velocity, reason=data.get("reason"),
                           hold=data.get("hold"), source_stamp_ns=stamp_ns(data.get("source_stamp")) or data.get("source_stamp_ns"),
                           body_source_stamp_ns=stamp_ns(data.get("body_source_stamp")) or data.get("body_source_stamp_ns"), flags=flags)
        elif kind in ("motion_validation", "trajectory_validation", "permit", "safety"):
            self.event("original_gate_reason", record, lane=kind,
                       reason=data.get("reason"), allowed=data.get("allowed"), valid=data.get("valid"),
                       revoked=data.get("revoked"), source_stamp_ns=stamp_ns(data.get("source_stamp")) or data.get("source_stamp_ns"))

    def summary(self):
        routes = [{k:v for k,v in r.items() if k != "key"} for r in self.route_segments]
        return dict(schema=1, scope="read_only_causal_evidence_not_acceptance", records=self.line,
                    events=dict(self.counts), complete_curve_identities=len(self.curves),
                    conflicting_curve_identities=len(self.ambiguous),
                    conflicting_original_body_source_count=len(self.ambiguous_bodies),
                    unresolved_original_body_diagnostics=len(self.pending), pending_counts=dict(self.pending_counts),
                    recorded_owner_credit_counts=[dict(execution_id=k[0], control_epoch=k[1], count=v)
                                                   for k,v in self.credit_counts.items()],
                    fixed_route_domains=routes, configured_command_limits=dict(speed_mps=self.command_speed_limit,
                                                                                yaw_radps=self.command_yaw_limit),
                    native_model_command_ceilings=dict(speed_mps=.3, yaw_radps=.5,
                        provenance="motion_sweep.hpp BrakingModel.valid command ceiling; diagnostic only"),
                    controller_comparison=dict(kp_position=self.kp, heading_threshold=self.heading_threshold,
                        lookahead_s=self.lookahead,
                        spatial_control_lookahead=self.spatial_control_lookahead,
                        small_vector_convention_mps=1e-5,
                        provenance="original tracker configuration/defaults; no new motion authority"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--events-output", type=Path, required=True)
    parser.add_argument("--summary-output", type=Path, required=True)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--tracker-config", type=Path)
    parser.add_argument("--max-records", type=int, help="Optional bounded prefix observation; never a task duration change")
    args = parser.parse_args()
    if args.max_records is not None and args.max_records <= 0:
        parser.error("max-records must be positive")
    if args.events_output.exists() or args.summary_output.exists():
        parser.error("audit output already exists")
    session = args.session or args.trace.parent/"session.json"
    model = json.loads(session.read_text()) if session.exists() else {}
    config = {}
    if args.tracker_config:
        import yaml
        config = yaml.safe_load(args.tracker_config.read_text())
        config = next(iter(config.values()))["ros__parameters"]
    source_hash = hashlib.sha256()
    with args.events_output.open("x") as output:
        observer = ControlAudit(lambda event: output.write(json.dumps(event, allow_nan=False)+"\n"),
            kp_position=config.get("kp_position", .8), heading_threshold=config.get("heading_threshold", .6),
            lookahead=config.get("lookahead", .8),
            spatial_control_lookahead=config.get("spatial_control_lookahead",False),
            command_speed_limit=model.get("max_speed_mps"), command_yaw_limit=model.get("max_yaw_radps"))
        with args.trace.open("rb") as source:
            for line in source:
                source_hash.update(line)
                observer.feed(json.loads(line))
                if args.max_records is not None and observer.line >= args.max_records:
                    break
        summary = observer.summary()
        summary.update(trace_path=str(args.trace.resolve()), trace_sha256=source_hash.hexdigest(),
                       events_path=str(args.events_output.resolve()),
                       trace_hash_scope="complete_stream_read" if args.max_records is None else "bounded_prefix_bytes_read",
                       requested_max_records=args.max_records)
    with args.summary_output.open("x") as output:
        json.dump(summary, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(summary, allow_nan=False))


if __name__ == "__main__":
    main()
