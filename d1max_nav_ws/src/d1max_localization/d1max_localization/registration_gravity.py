"""Gravity evidence for registration, not floor fitting or pose flattening.

Query gravity is the LIO posterior at the deskewed scan's exact source time.
It lives in tracking after applying the inverse posterior rotation; odom Z is
not assumed vertical. Map gravity is separately bound to the actual PCD and
its recorded mapping lineage. Neither source grants physical acceptance.
"""
from collections import deque
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .estimation.contracts import epoch_header, sample_pose, vector
from .math_utils import quaternion_conjugate, rotate_vector


def direction(value):
    value = np.asarray(vector(value, bound=20.), dtype=float)
    length = float(np.linalg.norm(value))
    if length < .5:
        raise ValueError('invalid_gravity_direction')
    return (value / length).tolist()


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _sha(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def load_map_gravity_contract(serialized, map_sha256, map_frame):
    """Verify the recorded LIO->PGO->byte-exact-cleaning frame chain once.

    This concrete importer is deliberately strict; other mapping pipelines
    need their own evidence importer, not a guessed +Z for an arbitrary PCD.
    """
    import yaml
    try:
        record = json.loads(serialized)
        if (record.get('schema') != 1 or record.get('kind') != 'lio_pgo_preserved_records'
                or not _sha(map_sha256) or record.get('map_sha256') != map_sha256
                or record.get('frame_id') != map_frame):
            raise ValueError('map_gravity_identity_mismatch')
        required = {'frontend_config', 'frontend_state', 'pgo_odometry', 'pgo_optimized', 'processed_manifest'}
        evidence = record['evidence']
        if set(evidence) != required:
            raise ValueError('map_gravity_lineage_incomplete')
        paths = {}
        for name in required:
            item = evidence[name]
            path = Path(item['path'])
            if not path.is_absolute() or not _sha(item['sha256']) or _sha256(path) != item['sha256']:
                raise ValueError('map_gravity_evidence_changed:' + name)
            paths[name] = path
        frontend = yaml.safe_load(paths['frontend_config'].read_text())
        params = next(iter(frontend.values()))['ros__parameters']
        aligned = params.get('mapping', {}).get('gravity_aligned_world', params.get('mapping.gravity_aligned_world'))
        if aligned is not True:
            raise ValueError('map_gravity_initialization_not_recorded')
        with paths['frontend_state'].open() as stream:
            first = next(csv.DictReader(stream))
        gravity = [float(first[key]) for key in ('gx', 'gy', 'gz')]
        if not 8.5 < np.linalg.norm(gravity) < 11.:
            raise ValueError('map_gravity_state_invalid')
        rotations = []
        for name in ('pgo_odometry', 'pgo_optimized'):
            with paths[name].open() as stream:
                pose = np.asarray([float(v) for v in stream.readline().split()]).reshape(3, 4)
            rot = pose[:, :3]
            if (not np.isfinite(pose).all() or not np.allclose(rot.T @ rot, np.eye(3), atol=1.e-4)
                    or not np.isclose(np.linalg.det(rot), 1., atol=1.e-4)):
                raise ValueError('map_gravity_anchor_invalid')
            rotations.append(rot)
        # The first PGO anchor defines map<-original-odom; retain its rotation
        # even if a future audited map was not saved with vertical world Z.
        gravity_map = rotations[1] @ rotations[0].T @ np.asarray(gravity)
        processed = json.loads(paths['processed_manifest'].read_text())
        if (processed.get('output_sha256') != map_sha256 or processed.get('frame_id') != map_frame
                or processed.get('source_unchanged') is not True
                or processed.get('record_preservation') != 'byte_exact_original_records_in_original_order'):
            raise ValueError('map_gravity_processed_coordinates_not_preserved')
        return dict(direction=direction(gravity_map.tolist()), frame_id=map_frame,
                    map_sha256=map_sha256, source='recorded_lio_pgo_frame_lineage',
                    evidence_id=hashlib.sha256(serialized.encode()).hexdigest(),
                    physical_calibration_verified=False)
    except (KeyError, TypeError, StopIteration, OSError, json.JSONDecodeError) as error:
        raise ValueError('map_gravity_contract_unavailable:' + type(error).__name__) from error


class QueryGravityHistory:
    """Small exact-time posterior cache. Invalid/reset input cannot reuse it."""
    def __init__(self, odom_frame, tracking_frame):
        self.odom_frame, self.tracking_frame = odom_frame, tracking_frame
        self.epoch = 0
        self.samples = deque(maxlen=32)

    def observe(self, value, now):
        epoch, valid = epoch_header(value, now)
        if epoch < self.epoch:
            return
        if epoch > self.epoch or not valid:
            self.samples.clear()
        self.epoch = epoch
        if not valid:
            return
        if value.get('frame') != self.odom_frame or value.get('child_frame') != self.tracking_frame:
            raise ValueError('query_gravity_frame_mismatch')
        stamp = int(value['stamp_ns'])
        if stamp <= 0 or not 0 <= now - stamp * 1.e-9 <= .5:
            raise ValueError('query_gravity_source_stale_or_future')
        inertial = value['inertial']
        gravity = vector(inertial['gravity'], bound=12.)
        if inertial.get('schema') != 1 or not 8.5 < np.linalg.norm(gravity) < 11.:
            raise ValueError('query_gravity_posterior_invalid')
        pose = sample_pose(value)
        gravity_tracking = rotate_vector(quaternion_conjugate(pose.orientation), gravity)
        if self.samples and stamp <= self.samples[-1]['source_stamp_ns']:
            return
        self.samples.append(dict(direction=direction(gravity_tracking), epoch=epoch,
            source_stamp_ns=stamp, frame_id=self.tracking_frame, source='lio_inertial_posterior'))

    def at(self, source_ns, epoch, now):
        if epoch != self.epoch or not 0 <= now - source_ns * 1.e-9 <= .5:
            return None
        for item in reversed(self.samples):
            # Native ROS/cloud nanoseconds can differ by float conversion only;
            # never use a nearest/latest scan pose or a new epoch's gravity.
            if abs(item['source_stamp_ns'] - source_ns) <= 1000:
                return dict(item, query_stamp_ns=source_ns)
        return None


def validate_registration_gravity(value, map_sha256=None):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('registration_gravity_contract_missing')
    try:
        query, target = value['query'], value['map']
        if (query.get('source') != 'lio_inertial_posterior'
                or type(query.get('epoch')) is not int or query['epoch'] <= 0
                or type(query.get('source_stamp_ns')) is not int or query['source_stamp_ns'] <= 0
                or type(query.get('query_stamp_ns')) is not int
                or abs(query['query_stamp_ns'] - query['source_stamp_ns']) > 1000
                or not query.get('frame_id') or not target.get('frame_id')
                or not _sha(target.get('map_sha256')) or not _sha(target.get('evidence_id'))
                or not target.get('source')
                or map_sha256 is not None and target['map_sha256'] != map_sha256):
            raise ValueError('registration_gravity_identity_invalid')
        return dict(schema=1, query=dict(query, direction=direction(query['direction'])),
                    map=dict(target, direction=direction(target['direction'])))
    except (KeyError, TypeError) as error:
        raise ValueError('registration_gravity_contract_invalid') from error


def gravity_error(transform, context):
    expected = np.asarray(context['map']['direction'])
    actual = np.asarray(transform)[:3, :3] @ np.asarray(context['query']['direction'])
    return math.acos(float(np.clip(np.dot(actual, expected), -1., 1.)))
