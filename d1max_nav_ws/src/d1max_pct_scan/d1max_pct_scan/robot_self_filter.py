"""Entity-only ray suppression, upstream of the production rolling map.

The sensor's actual acquisition-frame origin is used, not a robot-centred
clearing box. The declared fixed standing model needs no SDK joint stream. A
discarded self return is NOT converted into a max-range or free-space ray.
"""
from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np


@dataclass(frozen=True)
class SelfFilterResult:
    mask: np.ndarray
    report: dict


class RobotSelfFilter:
    """Read-only model shared by workers; each worker gets a pose snapshot."""
    def __init__(self, model, settings):
        from .robot_self_geometry import SelfFilterConfig
        self.model = model
        self.settings = dict(settings)
        self.mode = settings.get('mode', 'filter')
        self.pose_source = settings.get('pose_source', 'source_stamped_joints')
        if self.pose_source not in ('source_stamped_joints', 'fixed_standing_cad'):
            raise ValueError('invalid_entity_pose_source')
        self.requires_joint_stream = self.pose_source == 'source_stamped_joints'
        if self.mode not in ('audit', 'filter'):
            raise ValueError('invalid_entity_self_filter_mode')
        self.geometry_config = SelfFilterConfig(
            measurement_padding_m=float(settings.get('measurement_padding_m', .005)))
        self.max_origin_error = float(settings.get('max_sensor_origin_error_m', .02))
        self.max_points = int(settings.get('max_near_points', 4096))
        self.budget_ms = float(settings.get('classification_budget_ms', 30.))
        self.roi = np.asarray(settings.get('body_roi_half_extent_m', [.7, .5, .85]), dtype=float)
        self.origins = {int(k): np.asarray(v, dtype=float)
                        for k, v in settings['sensor_origins_body_m'].items()}
        if (self.roi.shape != (3,) or not np.isfinite(self.roi).all()
                or np.any(self.roi <= 0.) or np.any(self.roi > 2.)
                or not 0. <= self.max_origin_error <= .02
                or not 1 <= self.max_points <= 8192 or not 1. <= self.budget_ms <= 50.
                or set(self.origins) != {0, 1}
                or any(v.shape != (3,) or not np.isfinite(v).all() for v in self.origins.values())):
            raise ValueError('invalid_entity_self_filter_limits')
        self.roi.setflags(write=False)
        for value in self.origins.values():
            value.setflags(write=False)
        # Bind the model bytes as well as the policy. Runtime pose samples are
        # execution evidence, not a mutable geometry/configuration identity.
        model_description = model.to_dict()
        self.applicability = dict(model_description.get('applicability', {}))
        payload = dict(settings=settings, model=model_description)
        self.digest = hashlib.sha256(json.dumps(payload, sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()

    @classmethod
    def load(cls, settings):
        from .robot_self_geometry import RobotSelfModel
        path = Path(settings['model_manifest'])
        if not path.is_file() or path.stat().st_size > 1024*1024:
            raise ValueError('invalid_entity_model_manifest')
        if settings.get('pose_source') == 'fixed_standing_cad':
            from .robot_fixed_stance import FixedStanceModel
            model = FixedStanceModel.load(path, profile=settings.get('fixed_stance'))
        else:
            model = RobotSelfModel.load(path)
        result = cls(model, settings)
        result.warmup_report = model.warmup(include_dynamic=True,
            sensor_origins=tuple(result.origins.values()))
        return result

    def classify(self, endpoints, origin, times_ns, *, sensor_id, poses=None):
        """No coverage wait: only model parts supported at the beam time apply."""
        endpoints, origin = np.asarray(endpoints), np.asarray(origin)
        times_ns = np.asarray(times_ns, dtype=np.int64)
        mask = np.zeros(len(endpoints), dtype=bool)
        report = dict(model_digest=self.digest, mode=self.mode, sensor_id=int(sensor_id),
                      matched=0, suppressed=0, missing_links=[], per_link_counts={},
                      reason='', origin_error_m=0., classification_ms=0., raw_link_match_count=0,
                      pose_source=self.pose_source, requires_joint_stream=self.requires_joint_stream,
                      physical_calibration_verified=False,
                      applicability=dict(self.applicability), near_point_count=0,
                      unmatched_near_point_count=0,
                      source_start_ns=0, source_end_ns=0)
        if (endpoints.shape != (len(endpoints), 3) or origin.shape != (3,)
                or times_ns.shape != (len(endpoints),) or sensor_id not in self.origins
                or not np.isfinite(endpoints).all() or not np.isfinite(origin).all()):
            raise ValueError('invalid_entity_ray_input')
        selected = np.flatnonzero(np.all(np.abs(endpoints) <= self.roi, axis=1))
        report.update(near_point_count=len(selected), unmatched_near_point_count=len(selected),
                      source_start_ns=int(times_ns.min()) if len(times_ns) else 0,
                      source_end_ns=int(times_ns.max()) if len(times_ns) else 0)
        origin_error = float(np.linalg.norm(origin-self.origins[sensor_id]))
        report['origin_error_m'] = origin_error
        if origin_error > self.max_origin_error and self.requires_joint_stream:
            # Preserve the real rear origin/points. A CAD inconsistency is not
            # permission to move a sensor frame or enlarge the deletion mask.
            report['reason'] = 'sensor_origin_model_mismatch_rays_preserved'
            return SelfFilterResult(mask, report)
        # The fixed stance is an explicitly approximate CAD geometry, not a
        # sensor calibration authority. Classify using the actual recorded ray
        # origin; preserve and report the CAD mismatch without moving either
        # endpoint or origin. No joint/SDK availability gate is introduced.
        report['sensor_origin_matches_cad'] = origin_error <= self.max_origin_error
        if not len(selected):
            return SelfFilterResult(mask, report)
        started = time.perf_counter()
        counts, missing, inside, ambiguous, unloaded, unsupported = {}, set(), set(), set(), set(), set()
        geometry_incomplete = False

        def collect(result, indices, *, include_missing=True):
            nonlocal geometry_incomplete
            mask[indices] |= result.matched_mask
            if include_missing:
                missing.update(result.missing_links)
            inside.update(result.origin_inside_links)
            ambiguous.update(getattr(result, 'origin_ambiguous_links', ()))
            unloaded.update(getattr(result, 'unloaded_links', ()))
            unsupported.update(getattr(result, 'unsupported_occluder_links', ()))
            for key, value in result.per_link_counts.items():
                counts[key] = counts.get(key, 0)+int(value)
            geometry_incomplete |= bool(getattr(result, 'timed_out', False)
                or getattr(result, 'candidate_limit_reached', False))

        def query_config():
            remaining = self.budget_ms-(time.perf_counter()-started)*1000.
            return replace(self.geometry_config,
                max_classification_ms=max(.000001, remaining),
                max_candidate_points=self.max_points)

        joint_arrays = (poses.positions_arrays_at(times_ns[selected],
            required_joints=self.model.joint_types) if poses is not None else {})
        has_joint_coverage = any(np.isfinite(values).any() for values in joint_arrays.values())
        # One coherent supported model: a measured leg may occlude a fixed
        # body surface. Never classify fixed parts while ignoring AVAILABLE
        # articulation. Exact source poses are batched, not quantized/held.
        # With no joint coverage fixed parts need just one independent query.
        # Dense ground in the ROI does not consume the entity candidate cap.
        if not self.requires_joint_stream:
            result = self.model.classify(endpoints[selected], origin, config=query_config())
        elif has_joint_coverage:
            result = self.model.classify_articulated(endpoints[selected], origin,
                joint_arrays, include_static=True, config=query_config())
        else:
            result = self.model.classify(endpoints[selected], origin,
                include_static=True, include_dynamic=False, config=query_config())
        collect(result, selected)
        report.update(classification_ms=(time.perf_counter()-started)*1000.,
                      missing_links=sorted(missing), origin_inside_links=sorted(inside),
                      origin_ambiguous_links=sorted(ambiguous), unloaded_links=sorted(unloaded),
                      unsupported_occluder_links=sorted(unsupported),
                      per_link_counts=counts, matched=int(mask.sum()),
                      raw_link_match_count=sum(counts.values()))
        if geometry_incomplete or report['classification_ms'] > self.budget_ms:
            mask[:] = False
            report.update(reason='entity_query_budget_rays_preserved', matched=0,
                          discarded_partial_per_link_counts=counts,
                          per_link_counts={}, raw_link_match_count=0)
        if self.mode == 'audit':
            mask[:] = False
        report['suppressed'] = int(mask.sum())
        # This is input accounting, not a claim that an unmatched point is a
        # leg, a free cell, or a verified obstacle. A nominal posture cannot
        # identify real articulation without an independent measurement.
        report['unmatched_near_point_count'] = len(selected)-report['matched']
        return SelfFilterResult(mask, report)


def settings_from_session(session):
    """Optional to old sessions; the mainline explicitly enables model filtering."""
    value = session.get('entity_self_filter')
    if value is None:
        return None
    if (not isinstance(value, dict) or value.get('schema') != 1
            or value.get('body_frame') != session.get('navigation_contract', {}).get(
                'frames', {}).get('body_frame')
            or value.get('pose_source', 'source_stamped_joints') not in (
                'source_stamped_joints', 'fixed_standing_cad')):
        raise ValueError('invalid_entity_self_filter_session')
    if value.get('pose_source', 'source_stamped_joints') == 'source_stamped_joints':
        if (not isinstance(value.get('joint_state_topic'), str)
                or not value['joint_state_topic'].startswith('/')
                or value.get('joint_source') != 'source_stamped_named_radians'):
            raise ValueError('invalid_entity_joint_source')
    elif (value.get('joint_source') not in (None, 'not_available_not_required')
            or not isinstance(value.get('fixed_stance'), dict)):
        raise ValueError('invalid_fixed_stance_source')
    return value
