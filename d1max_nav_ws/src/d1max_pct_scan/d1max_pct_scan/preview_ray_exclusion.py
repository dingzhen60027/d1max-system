"""Explicit no-motion experiment, NOT calibrated robot self filtering.

An endpoint inside one of at most four small body-frame boxes drops its ENTIRE ray.
No endpoint substitution, ray extension, occupancy edit or free-space inference.
An external object in the same box is indistinguishable: execution is forbidden.
Schema 2 is an explicit whole near-body preview mask, not a measured self model.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np


@dataclass(frozen=True)
class Box:
    name: str
    sensor_id: int
    minimum: tuple
    maximum: tuple


@dataclass(frozen=True)
class PreviewRayExclusion:
    mode: str
    frame_id: str
    pad_m: tuple
    boxes: tuple
    digest: str

    def matches(self, endpoints_body, sensor_id):
        points = np.asarray(endpoints_body, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) > 250000
                or not np.isfinite(points).all() or type(sensor_id) is not int
                or sensor_id not in (0, 1)):
            raise ValueError('invalid_preview_exclusion_geometry')
        mask = np.zeros(len(points), dtype=bool)
        for box in self.boxes:
            if box.sensor_id == sensor_id:
                mask |= np.all((points >= np.asarray(box.minimum)-np.asarray(self.pad_m))
                               & (points <= np.asarray(box.maximum)+np.asarray(self.pad_m)), axis=1)
        return mask


def from_session(session, body_frame):
    """Absent means byte-preserving baseline. Reject ambiguous/execution use."""
    if 'preview_ray_exclusion' not in session:
        return None
    value = session['preview_ray_exclusion']
    if (session.get('mode') != 'LIVE_VISUALIZATION_NO_MOTION'
            or session.get('motion_control_enabled') is not False
            or session.get('motion') is not None
            or session.get('perception_backend') != 'per_sensor_rays'):
        raise ValueError('ray_exclusion_requires_explicit_no_motion_preview')
    fields = {'schema', 'mode', 'frame_id', 'calibration_verified', 'pad_m', 'boxes'}
    near_body = isinstance(value, dict) and value.get('schema') == 2
    if near_body:
        fields.add('scope')
    if (not isinstance(value, dict) or set(value) != fields
            or type(value['schema']) is not int or value['schema'] not in (1, 2)
            or value['mode'] not in ('audit', 'preview_drop_rays')
            or value['frame_id'] != body_frame or not isinstance(body_frame, str) or not body_frame
            or value['calibration_verified'] is not False
            or near_body and value['scope'] != 'near_body_preview'):
        raise ValueError('invalid_unverified_preview_exclusion')
    padding = value['pad_m']
    pad = (padding,)*3 if type(padding) in (int, float) else padding
    if (not isinstance(pad, (list, tuple)) or len(pad) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= .02
                   for x in pad)):
        raise ValueError('preview_exclusion_padding_must_be_0_to_2cm')
    pad = tuple(float(x) for x in pad)
    if not isinstance(value['boxes'], list) or not 1 <= len(value['boxes']) <= 4:
        raise ValueError('preview_exclusion_requires_one_to_four_local_boxes')
    boxes, names = [], set()
    for box in value['boxes']:
        if (not isinstance(box, dict) or set(box) != {'name', 'sensor_id', 'min_m', 'max_m'}
                or not isinstance(box['name'], str) or not 1 <= len(box['name']) <= 64
                or box['name'] in names or type(box['sensor_id']) is not int
                or box['sensor_id'] not in (0, 1)):
            raise ValueError('invalid_preview_exclusion_box')
        for key in ('min_m', 'max_m'):
            vec = box[key]
            if (not isinstance(vec, (list, tuple)) or len(vec) != 3
                    or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x)+pad[i] > 1.2
                           for i, x in enumerate(vec))):
                raise ValueError('invalid_preview_exclusion_bounds')
        dimensions = np.asarray(box['max_m'])-np.asarray(box['min_m'])
        if np.any(dimensions <= 0) or (not near_body and np.any(dimensions > .25)):
            raise ValueError('preview_exclusion_box_is_not_local')
        if near_body:
            # Deliberately bounded geometric suppression, never an inferred
            # self/ground classification. No extra padding or remote boxes.
            minimum, maximum = np.asarray(box['min_m']), np.asarray(box['max_m'])
            bound = np.array([.65, .40, .55])
            if (any(pad) or np.any(minimum > 0) or np.any(maximum < 0)
                    or np.any(minimum < -bound) or np.any(maximum > bound)):
                raise ValueError('near_body_preview_mask_exceeds_bounds')
        names.add(box['name'])
        boxes.append(Box(box['name'], box['sensor_id'], tuple(box['min_m']), tuple(box['max_m'])))
    if near_body and (len(boxes) != 2 or {box.sensor_id for box in boxes} != {0, 1}
            or boxes[0].minimum != boxes[1].minimum or boxes[0].maximum != boxes[1].maximum):
        raise ValueError('near_body_preview_requires_same_mask_for_both_sensors')
    digest = hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                      allow_nan=False).encode()).hexdigest()
    return PreviewRayExclusion(value['mode'], body_frame, pad, tuple(boxes), digest)
