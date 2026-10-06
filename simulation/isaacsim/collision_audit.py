"""Sampled simulation geometry separation audit; no controller or map writes.

Every enabled robot primitive is transformed by its actual PhysX link matrix.
Registered actor shapes use the same-step measured actor-root pose. Strictly
disjoint enclosing AABBs prove separation at that sample. Touching/overlapping
AABBs are unresolved, not evidence of penetration. Floor contact is excluded
and must be checked by the separate support-contact certificate. This is never
a continuous trajectory collision proof or a deployable perception sensor.
Prepared cases may also seal a sampled proximity encounter rule. Its witnesses
come from these same actual snapshots and fixed primitive pairs, never desired
actor poses, goal results, or a second navigation controller.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from collections import deque

import numpy as np

from dynamic_collision import actor_registry, canonical
from quadruped import pose_matrix, primitive_aabb

SCOPE = "sampled_actual_link_geometry_static_and_actor"
NUMERICAL_OUTWARD_MARGIN_M = 1e-6
ENCOUNTER_RULE = "measured_primitive_center_approach_near_depart_v1"
ENCOUNTER_SOURCE = "actual_registered_link_and_actor_geometry"
ENCOUNTER_SCOPE = "sampled_same_step_primitive_center_proximity_only"


def encounter_contract(actor_ids):
    """Explicit preparation-time rule; changing it requires a new candidate."""
    return dict(schema=1, rule=ENCOUNTER_RULE, required_actor_ids=sorted(actor_ids),
        near_center_distance_m=2., minimum_near_source_s=.5,
        minimum_distance_change_m=.5, minimum_leg_source_s=1.,
        minimum_mean_distance_rate_mps=.05, maximum_leg_source_s=30.,
        maximum_source_sampling_gap_s=.12, maximum_history_samples_per_pair=2048,
        maximum_witnesses_per_actor=128,
        approach_starts_outside_near=True, departure_ends_outside_near=True,
        identity_scope="same_exact_robot_path_and_actor_shape_path_for_all_three_legs",
        scope=ENCOUNTER_SCOPE)


def validate_encounter_contract(value, enabled_actor_ids):
    if not isinstance(value, dict):
        raise ValueError("invalid_actor_encounter_contract")
    ids = value.get("required_actor_ids")
    if (not isinstance(ids, list) or not all(isinstance(v, str) for v in ids)
            or ids != sorted(set(ids))
            or not set(ids).issubset(enabled_actor_ids)
            or value != encounter_contract(ids)):
        raise ValueError("invalid_actor_encounter_contract")
    return value


class _EncounterPairs:
    """Bounded histories for exact primitive pairs, using original source time.

    A primitive's local origin lies inside each supported convex shape. Its
    transformed center therefore gives an actual shape-separation upper bound;
    a small AABB separation lower bound alone cannot prove a near encounter.
    """
    def __init__(self, contract, anchor_ns):
        self.contract, self.anchor_ns = contract, anchor_ns
        self.pairs, self.witnesses, self.truncated_actor_ids = {}, {}, set()

    def observe(self, actor_id, robot_path, actor_path, sim_ns, robot_center, actor_center):
        if actor_id in self.truncated_actor_ids:
            return
        c = self.contract
        key = (actor_id, robot_path, actor_path)
        state = self.pairs.setdefault(key, dict(history=deque(), near=None, last_ns=None))
        point = dict(sim_time_ns=sim_ns, source_stamp_ns=self.anchor_ns + sim_ns,
            actor_id=actor_id, robot_path=robot_path, actor_shape_path=actor_path,
            robot_center_m=robot_center.tolist(), actor_center_m=actor_center.tolist(),
            center_distance_m=float(np.linalg.norm(robot_center-actor_center)))
        if state["last_ns"] is not None and sim_ns-state["last_ns"] > round(c["maximum_source_sampling_gap_s"]*1e9):
            state["history"].clear(); state["near"] = None
        state["last_ns"] = sim_ns
        history = state["history"]
        while history and sim_ns-history[0]["sim_time_ns"] > round(c["maximum_leg_source_s"]*1e9):
            history.popleft()
        near, distance = state["near"], point["center_distance_m"]
        if near is None and distance <= c["near_center_distance_m"]:
            for previous in reversed(history):
                dt = (sim_ns-previous["sim_time_ns"])*1e-9
                change = previous["center_distance_m"]-distance
                if (dt >= c["minimum_leg_source_s"] and change >= c["minimum_distance_change_m"]
                        and change/dt >= c["minimum_mean_distance_rate_mps"]
                        and previous["center_distance_m"] > c["near_center_distance_m"]):
                    near = state["near"] = dict(approach=previous, begin=point,
                        closest=point, end=point, complete=False, closed=False,
                        sample_count=0, maximum_distance_m=distance)
                    break
        if near is not None:
            if distance <= c["near_center_distance_m"] and not near["closed"]:
                if distance < near["closest"]["center_distance_m"]:
                    near["closest"] = point
                near["end"] = point
                near["sample_count"] += 1
                near["maximum_distance_m"] = max(near["maximum_distance_m"], distance)
                near["complete"] = sim_ns-near["begin"]["sim_time_ns"] >= round(c["minimum_near_source_s"]*1e9)
            elif not near["complete"]:
                state["near"] = None
            else:
                near["closed"] = True
            if near["complete"]:
                dt = (sim_ns-near["closest"]["sim_time_ns"])*1e-9
                change = distance-near["closest"]["center_distance_m"]
                if near["closed"] and dt > c["maximum_leg_source_s"]:
                    state["near"] = None
                if (c["minimum_leg_source_s"] <= dt <= c["maximum_leg_source_s"]
                        and distance > c["near_center_distance_m"]
                        and change >= c["minimum_distance_change_m"]
                        and change/dt >= c["minimum_mean_distance_rate_mps"]):
                    witness = dict(actor_id=actor_id, robot_path=robot_path,
                        actor_shape_path=actor_path, approach_start=near["approach"],
                        near_begin=near["begin"], near_end=near["end"],
                        closest=near["closest"], departure_end=point,
                        near_sample_count=near["sample_count"],
                        near_maximum_center_distance_m=near["maximum_distance_m"])
                    certificates = self.witnesses.setdefault(actor_id, [])
                    # Different feet can prove the same observed episode. Keep
                    # one complete exact-pair certificate for overlapping near
                    # intervals, without combining distances between pairs.
                    duplicate = any(w["near_begin"]["sim_time_ns"] <= near["end"]["sim_time_ns"]
                        and near["begin"]["sim_time_ns"] <= w["near_end"]["sim_time_ns"] for w in certificates)
                    if not duplicate:
                        if len(certificates) >= c["maximum_witnesses_per_actor"]:
                            self.truncated_actor_ids.add(actor_id)
                        else:
                            certificates.append(witness)
                    state["near"] = None
                    history.clear()
        # Only outside-near samples can anchor an approach. Keeping stationary
        # near samples here would add repeated history scans without evidence.
        if distance > c["near_center_distance_m"]:
            history.append(point)
        while len(history) > c["maximum_history_samples_per_pair"]:
            history.popleft()
            # Resource-driven loss can hide a valid approach anchor. Preserve
            # earlier certificates for diagnostics, but never turn lost
            # history into a completed no-encounter verdict. Ordinary expiry
            # beyond the sealed source-time window above is not truncation.
            self.truncated_actor_ids.add(actor_id)


def review_actual_encounter(value, spec, collision, action_begin_ns, action_end_ns):
    """Validate a producer's audit-bound certificates, never Action outcomes.

    None means incomplete evidence. False means a complete fresh audit did not
    contain the required encounter during the original case's Action interval.
    """
    enabled = [a["id"] for a in actor_registry(spec)]
    contract = validate_encounter_contract(
        spec["scenario_suite_contract"]["actor_encounter_contract"], enabled)
    if value is None or not contract["required_actor_ids"]:
        return None, []
    if not isinstance(value, dict):
        raise ValueError("invalid_actual_actor_encounter_record")
    if value.get("completed") is not True or collision.get("completed") is not True:
        return None, []
    expected = dict(schema=1, rule=ENCOUNTER_RULE, source=ENCOUNTER_SOURCE, scope=ENCOUNTER_SCOPE,
        required_actor_ids=contract["required_actor_ids"],
        contract_sha256=hashlib.sha256(canonical(contract)).hexdigest(),
        session_id=collision["session_id"], scene_sha256=collision["spec_sha256"],
        source_begin_ns=collision["source_begin_ns"], source_end_ns=collision["source_end_ns"],
        robot_collision_registry_sha256=hashlib.sha256(canonical(spec["robot_collision_registry"])).hexdigest(),
        actor_collision_registry_sha256=hashlib.sha256(canonical(actor_registry(spec))).hexdigest())
    if (any(value.get(k) != v for k, v in expected.items())
            or collision.get("robot_collision_registry_sha256") != expected["robot_collision_registry_sha256"]
            or collision.get("actor_collision_registry_sha256") != expected["actor_collision_registry_sha256"]
            or value.get("truncated_actor_ids") != []):
        raise ValueError("actor_encounter_identity_or_contract_mismatch")
    gap = value.get("maximum_source_sampling_gap_ns")
    if (type(gap) is not int or gap <= 0 or gap != collision.get("max_source_sampling_gap_ns")
            or gap > round(contract["maximum_source_sampling_gap_s"]*1e9)
            or type(value.get("source_anchor_ns")) is not int or value["source_anchor_ns"] < 0
            or type(action_begin_ns) is not int or type(action_end_ns) is not int
            or not value["source_begin_ns"] <= action_begin_ns < action_end_ns <= value["source_end_ns"]):
        raise ValueError("actor_encounter_source_coverage_or_freshness_mismatch")
    actors = value.get("actors")
    if (not isinstance(actors, list) or not all(isinstance(a, dict) for a in actors)
            or [a.get("actor_id") for a in actors] != contract["required_actor_ids"]
            or type(value.get("observed")) is not bool):
        raise ValueError("actor_encounter_actor_registry_mismatch")
    robot_paths = {p["path"] for p in spec["robot_collision_registry"]["colliders"]}
    shape_paths = {a["id"]: {s["path"] for s in a["shapes"]} for a in actor_registry(spec)}
    selected = []
    all_producer_observed = True
    for actor in actors:
        actor_id, certificates = actor["actor_id"], actor.get("witnesses")
        if (not isinstance(certificates, list) or len(certificates) > contract["maximum_witnesses_per_actor"]
                or type(actor.get("observed")) is not bool or actor["observed"] != bool(certificates)):
            raise ValueError("actor_encounter_inconsistent_observation")
        all_producer_observed = all_producer_observed and actor["observed"]
        candidate = None
        for w in certificates:
            if not isinstance(w, dict):
                raise ValueError("invalid_actual_actor_encounter_certificate")
            identity = dict(actor_id=actor_id, robot_path=w.get("robot_path"), actor_shape_path=w.get("actor_shape_path"))
            if (w.get("actor_id") != actor_id or identity["robot_path"] not in robot_paths
                    or identity["actor_shape_path"] not in shape_paths[actor_id]):
                raise ValueError("actor_encounter_unregistered_primitive_pair")
            points = [w[key] for key in ("approach_start", "near_begin", "closest", "near_end", "departure_end")]
            for p in points:
                if (not isinstance(p, dict) or any(p.get(k) != v for k, v in identity.items()) or type(p.get("sim_time_ns")) is not int
                        or p["sim_time_ns"] < 0 or type(p.get("source_stamp_ns")) is not int
                        or p["source_stamp_ns"] != value["source_anchor_ns"]+p["sim_time_ns"]
                        or not value["source_begin_ns"] <= p["source_stamp_ns"] <= value["source_end_ns"]):
                    raise ValueError("actor_encounter_pair_or_source_changed_between_legs")
                distance = p.get("center_distance_m")
                measured = float(np.linalg.norm(_vector(p["robot_center_m"], 3, "encounter_robot_center")
                            - _vector(p["actor_center_m"], 3, "encounter_actor_center")))
                if (isinstance(distance, bool) or not isinstance(distance, (int, float))
                        or not math.isfinite(distance) or distance < 0
                        or not math.isclose(distance, measured, rel_tol=0., abs_tol=1e-8)):
                    raise ValueError("actor_encounter_distance_not_bound_to_centers")
            a, b, closest, end, d = points
            if not a["sim_time_ns"] < b["sim_time_ns"] <= closest["sim_time_ns"] <= end["sim_time_ns"] < d["sim_time_ns"]:
                raise ValueError("actor_encounter_legs_not_source_ordered")
            for start, finish, change in ((a, b, a["center_distance_m"]-b["center_distance_m"]),
                    (closest, d, d["center_distance_m"]-closest["center_distance_m"])):
                dt = (finish["sim_time_ns"]-start["sim_time_ns"])*1e-9
                if (not contract["minimum_leg_source_s"] <= dt <= contract["maximum_leg_source_s"]
                        or change < contract["minimum_distance_change_m"]
                        or change/dt < contract["minimum_mean_distance_rate_mps"]):
                    raise ValueError("actor_encounter_approach_or_departure_threshold_missing")
            max_near = w.get("near_maximum_center_distance_m")
            if (a["center_distance_m"] <= contract["near_center_distance_m"]
                    or d["center_distance_m"] <= contract["near_center_distance_m"]
                    or any(p["center_distance_m"] > contract["near_center_distance_m"] for p in (b, closest, end))
                    or (end["sim_time_ns"]-b["sim_time_ns"])*1e-9 < contract["minimum_near_source_s"]
                    or type(w.get("near_sample_count")) is not int
                    or w["near_sample_count"] < math.ceil(contract["minimum_near_source_s"]/contract["maximum_source_sampling_gap_s"])+1
                    or isinstance(max_near, bool) or not isinstance(max_near, (int, float))
                    or not math.isfinite(max_near) or max_near > contract["near_center_distance_m"]
                    or max_near < max(p["center_distance_m"] for p in (b, closest, end))):
                raise ValueError("actor_encounter_near_dwell_threshold_missing")
            if action_begin_ns <= a["source_stamp_ns"] and d["source_stamp_ns"] <= action_end_ns:
                candidate = w
        if candidate is not None:
            selected.append(candidate)
    if value["observed"] != all_producer_observed:
        raise ValueError("actor_encounter_inconsistent_observation")
    return len(selected) == len(actors), selected


def _vector(value, length, label):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError("invalid_" + label)
    return array


def _shape(value):
    kind = value.get("type")
    if kind == "Cube":
        dimensions = [value["size"]]
    elif kind == "Sphere":
        dimensions = [value["radius"]]
    elif kind in ("Cylinder", "Capsule") and value.get("axis") in ("X", "Y", "Z"):
        dimensions = [value["radius"], value["height"]]
    else:
        raise ValueError("unsupported_audit_primitive")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) or v <= 0 for v in dimensions):
        raise ValueError("invalid_audit_primitive_dimensions")
    return value


def _matrix(value):
    matrix = np.asarray(value, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.array_equal(matrix[3], [0., 0., 0., 1.])):
        raise ValueError("invalid_audit_world_matrix")
    determinant = np.linalg.det(matrix[:3, :3])
    if not math.isfinite(determinant) or abs(determinant) < 1e-12:
        raise ValueError("invalid_audit_world_matrix")
    return matrix


def _bounds(shape, matrix):
    lower, upper = primitive_aabb(_shape(shape), _matrix(matrix))
    if not np.isfinite(lower).all() or not np.isfinite(upper).all() or np.any(lower > upper):
        raise ValueError("invalid_audit_primitive_bounds")
    return (lower - NUMERICAL_OUTWARD_MARGIN_M,
            upper + NUMERICAL_OUTWARD_MARGIN_M)


class Audit:
    """Caller must feed complete, same-physics-step measured snapshots.

    spec is the sealed scene JSON dictionary; scene_sha256 hashes its original
    file bytes. Actor samples are keyed by registered id and contain present,
    position and orientation_xyzw. Optional source_stamp_ns must exactly equal
    anchor_ns + sim_ns. Robot snapshots use QuadrupedPlant.collider_snapshot().
    finish returns a JSON-serializable dictionary; the caller saves it.
    """

    def __init__(self, spec, session_id, scene_sha256, anchor_ns):
        if (not isinstance(session_id, str) or not session_id
                or not isinstance(scene_sha256, str) or len(scene_sha256) != 64
                or any(c not in "0123456789abcdef" for c in scene_sha256)
                or type(anchor_ns) is not int or anchor_ns < 0):
            raise ValueError("invalid_audit_source_identity")
        # Independent deep copy fixes the authorized geometry for this audit.
        self.spec = json.loads(canonical(spec))
        self.session_id, self.spec_sha256, self.anchor_ns = session_id, scene_sha256, anchor_ns
        self.robot_kind = self.spec["robot"]["kind"]
        registry = self.spec.get("robot_collision_registry")
        if not isinstance(registry, dict) or not isinstance(registry.get("root"), str):
            raise ValueError("missing_actual_robot_collision_registry")
        colliders = registry.get("colliders")
        if not isinstance(colliders, list) or not colliders:
            raise ValueError("missing_actual_robot_collision_registry")
        self.robot = {}
        for item in colliders:
            path = item.get("path")
            if (not isinstance(path, str) or not path.startswith(registry["root"] + "/")
                    or path in self.robot):
                raise ValueError("invalid_audit_robot_registry_path")
            shape = _shape(dict(type=item["type"], **item["local_geometry"]))
            scale = _vector(item["world_scale"], 3, "registry_world_scale")
            if np.any(scale <= 0):
                raise ValueError("invalid_registry_world_scale")
            self.robot[path] = (shape, scale)
        self.registry_sha = hashlib.sha256(canonical(registry)).hexdigest()
        self.actors = actor_registry(self.spec)
        self.actor_ids = [actor["id"] for actor in self.actors]
        self.actor_registry_sha = hashlib.sha256(canonical(self.actors)).hexdigest()
        encounter = self.spec.get("scenario_suite_contract", {}).get("actor_encounter_contract")
        self.encounters = (_EncounterPairs(validate_encounter_contract(encounter, self.actor_ids), anchor_ns)
                           if encounter is not None else None)
        self._actor_centers = []
        self.static = []
        names = set()
        for box in self.spec["static_boxes"]:
            name = box["name"]
            if not isinstance(name, str) or name in names:
                raise ValueError("invalid_audit_static_name")
            names.add(name)
            center, size = _vector(box["center"], 3, "static_center"), _vector(box["size"], 3, "static_size")
            if np.any(size <= 0):
                raise ValueError("invalid_static_size")
            # Floor is only the explicit support plane; do not exempt low boxes.
            self.static.append(("/World/Indoor/" + name,
                center - size / 2 - NUMERICAL_OUTWARD_MARGIN_M,
                center + size / 2 + NUMERICAL_OUTWARD_MARGIN_M))
        room, ceiling = self.spec["room"], self.spec["ceiling"]
        lower = _vector([room["x_min"], room["y_min"], ceiling["z"]], 3, "ceiling_min")
        upper = _vector([room["x_max"], room["y_max"], ceiling["z"] + ceiling["thickness"]], 3, "ceiling_max")
        if np.any(upper <= lower) or "ceiling" in names:
            raise ValueError("invalid_audit_ceiling")
        self.static.append(("/World/Indoor/ceiling", lower - NUMERICAL_OUTWARD_MARGIN_M,
                            upper + NUMERICAL_OUTWARD_MARGIN_M))
        self.times, self.faults, self.examples = [], [], []
        self.pairs = {"static": 0, "dynamic": 0}
        self.overlaps = {"static": 0, "dynamic": 0}
        self.overlap_samples = {"static": 0, "dynamic": 0}
        self.minimum = {"static": math.inf, "dynamic": math.inf}

    def _robot_bounds(self, snapshot):
        if not isinstance(snapshot, (list, tuple)) or len(snapshot) != len(self.robot):
            raise ValueError("incomplete_actual_robot_snapshot")
        result, seen = [], set()
        for item in snapshot:
            path = item.get("path")
            if path not in self.robot or path in seen:
                raise ValueError("unregistered_or_duplicate_robot_collider")
            seen.add(path)
            shape, scale = self.robot[path]
            if canonical(item["shape"]) != canonical(shape):
                raise ValueError("actual_robot_primitive_changed:" + path)
            matrix = _matrix(item["world_matrix"])
            if not np.allclose(np.linalg.norm(matrix[:3, :3], axis=0), scale, rtol=0., atol=1e-6):
                raise ValueError("actual_robot_primitive_scale_changed:" + path)
            lower, upper = _bounds(shape, matrix)
            result.append((path, lower, upper))
        return result

    def _actor_bounds(self, sim_ns, samples):
        if not isinstance(samples, dict) or set(samples) != set(self.actor_ids):
            raise ValueError("incomplete_actual_actor_snapshot")
        result, centers = [], []
        for actor in self.actors:
            sample = samples[actor["id"]]
            if (sample.get("present") is not True or ("source_stamp_ns" in sample
                    and (type(sample["source_stamp_ns"]) is not int
                         or sample["source_stamp_ns"] != self.anchor_ns + sim_ns))):
                raise ValueError("actor_not_measured_at_same_source")
            q = _vector(sample["orientation_xyzw"], 4, "actor_orientation")
            matrix = pose_matrix(sample["position"], q[[3, 0, 1, 2]])
            for shape in actor["shapes"]:
                local = np.eye(4)
                local[:3, 3] = shape["center"]
                if shape["type"] == "box":
                    local[:3, :3] = np.diag(np.asarray(shape["size"]) / 2)
                    primitive = dict(type="Cube", size=2.)
                else:
                    primitive = dict(type="Cylinder", radius=shape["radius"], height=shape["height"], axis=shape["axis"])
                world = matrix @ local
                lower, upper = _bounds(primitive, world)
                result.append((shape["path"], lower, upper))
                if self.encounters is not None:
                    centers.append((actor["id"], shape["path"], world[:3, 3]))
        self._actor_centers = centers
        return result

    def _compare(self, robots, others, kind, sim_ns):
        if not others:
            return
        rmin, rmax = np.array([r[1] for r in robots]), np.array([r[2] for r in robots])
        omin, omax = np.array([o[1] for o in others]), np.array([o[2] for o in others])
        gaps = np.maximum(np.maximum(rmin[:, None] - omax, omin - rmax[:, None]), 0.)
        distances = np.linalg.norm(gaps, axis=2)
        intersections = distances == 0.
        count = int(intersections.sum())
        self.pairs[kind] += int(distances.size)
        self.overlaps[kind] += count
        self.overlap_samples[kind] += int(count > 0)
        self.minimum[kind] = min(self.minimum[kind], float(distances.min()))
        for ri, oi in np.argwhere(intersections):
            if len(self.examples) >= 16:
                break
            self.examples.append(dict(kind=kind, sim_time_ns=sim_ns,
                source_stamp_ns=self.anchor_ns + sim_ns,
                robot_path=robots[ri][0], obstacle_path=others[oi][0],
                aabb_intersection_min=np.maximum(rmin[ri], omin[oi]).tolist(),
                aabb_intersection_max=np.minimum(rmax[ri], omax[oi]).tolist(),
                classification="potential_overlap_unresolved"))

    def sample(self, sim_ns, robot_snapshot, actor_samples):
        try:
            if type(sim_ns) is not int or sim_ns < 0 or (self.times and sim_ns <= self.times[-1]):
                raise ValueError("nonmonotonic_or_invalid_audit_source_time")
            robots = self._robot_bounds(robot_snapshot)
            actors = self._actor_bounds(sim_ns, actor_samples)
            self._compare(robots, self.static, "static", sim_ns)
            self._compare(robots, actors, "dynamic", sim_ns)
            if self.encounters is not None:
                required = self.encounters.contract["required_actor_ids"]
                for actor_id, actor_path, center in self._actor_centers:
                    if actor_id not in required:
                        continue
                    for item in robot_snapshot:
                        self.encounters.observe(actor_id, item["path"], actor_path, sim_ns,
                            np.asarray(item["world_matrix"], dtype=np.float64)[:3, 3], center)
            self.times.append(sim_ns)
        except (ValueError, KeyError, TypeError, OverflowError, np.linalg.LinAlgError) as error:
            self.faults.append(str(error))
            raise ValueError("actual_collision_audit_incomplete:" + str(error)) from error

    def finish(self, trajectory_path, completed=True):
        path = Path(trajectory_path)
        trajectory_sha, trajectory_count, coverage = None, 0, False
        try:
            raw = path.read_bytes()
            trajectory_sha = hashlib.sha256(raw).hexdigest()
            rows = [json.loads(line) for line in raw.splitlines() if line.strip()]
            stamps = [row["sim_time_ns"] for row in rows]
            measured = set(self.times)
            trajectory_count = len(stamps)
            coverage = (bool(stamps) and all(type(t) is int and t >= 0 for t in stamps)
                and all(a < b for a, b in zip(stamps, stamps[1:]))
                and all(t in measured for t in stamps))
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.faults.append("invalid_audit_trajectory:" + str(error))
        complete = completed is True and bool(self.times) and not self.faults and coverage
        max_gap = max((b-a for a,b in zip(self.times,self.times[1:])), default=None)
        encounter = None
        if self.encounters is not None:
            contract = self.encounters.contract
            fresh = (complete and max_gap is not None and not self.encounters.truncated_actor_ids
                     and max_gap <= round(contract["maximum_source_sampling_gap_s"]*1e9))
            required = contract["required_actor_ids"]
            encounter = dict(schema=1, rule=ENCOUNTER_RULE, source=ENCOUNTER_SOURCE,
                scope=ENCOUNTER_SCOPE, completed=fresh, required_actor_ids=required,
                contract_sha256=hashlib.sha256(canonical(contract)).hexdigest(),
                observed=all(a in self.encounters.witnesses for a in required) if fresh and required else None,
                actors=[dict(actor_id=a, observed=a in self.encounters.witnesses if fresh else None,
                             witnesses=self.encounters.witnesses.get(a, [])) for a in required],
                truncated_actor_ids=sorted(self.encounters.truncated_actor_ids),
                maximum_source_sampling_gap_ns=max_gap,
                source_begin_ns=self.anchor_ns+self.times[0] if self.times else None,
                source_end_ns=self.anchor_ns+self.times[-1] if self.times else None,
                source_anchor_ns=self.anchor_ns, session_id=self.session_id,
                scene_sha256=self.spec_sha256, robot_collision_registry_sha256=self.registry_sha,
                actor_collision_registry_sha256=self.actor_registry_sha,
                observation_status="not_required" if not required else "completed" if fresh else "pending",
                limitations=["sampled_proximity_only", "not_route_blocking_or_yield_or_contact_evidence",
                             "caller_binds_actual_snapshots_to_same_physics_step"])
        return dict(schema=1, scope=SCOPE, completed=complete,
            continuous_collision_proof=False,
            separation_method="strict_closed_AABB_separation_of_actual_transformed_primitives",
            numerical_outward_margin_m=NUMERICAL_OUTWARD_MARGIN_M,
            floor_contact_scope="excluded_explicit_support_plane_requires_separate_contact_certificate",
            robot_kind=self.robot_kind, session_id=self.session_id,
            spec_sha256=self.spec_sha256, spec_canonical_sha256=hashlib.sha256(canonical(self.spec)).hexdigest(),
            trajectory_sha256=trajectory_sha, robot_collision_registry_sha256=self.registry_sha,
            actor_collision_registry_sha256=self.actor_registry_sha,
            actor_ids=self.actor_ids, sample_count=len(self.times),
            trajectory_sample_count=trajectory_count, trajectory_sources_all_audited=coverage,
            source_begin_ns=self.anchor_ns + self.times[0] if self.times else None,
            source_end_ns=self.anchor_ns + self.times[-1] if self.times else None,
            max_source_sampling_gap_ns=max_gap, actual_actor_encounter=encounter,
            non_floor_static_penetration_count=0 if complete and self.overlaps["static"] == 0 else None,
            dynamic_penetration_count=0 if complete and self.overlaps["dynamic"] == 0 else None,
            possible_overlap_counts=dict(self.overlaps), possible_overlap_sample_counts=dict(self.overlap_samples),
            checked_pair_counts=dict(self.pairs), possible_overlap_examples=list(self.examples),
            min_static_separation_lower_bound_m=self.minimum["static"] if math.isfinite(self.minimum["static"]) else None,
            min_actor_separation_lower_bound_m=self.minimum["dynamic"] if math.isfinite(self.minimum["dynamic"]) else None,
            faults=list(self.faults),
            limitations=["sampled_times_only", "AABB_overlap_is_unresolved_not_measured_penetration",
                         "encounter_is_sampled_proximity_only", "caller_binds_snapshots_to_same_physics_step"])
