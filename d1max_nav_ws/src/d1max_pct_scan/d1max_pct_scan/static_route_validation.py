"""Bounded, ROS-free validation of immutable native route/map snapshots."""
from concurrent.futures import ThreadPoolExecutor
import time

import numpy as np

from .live_global_contract import GlobalPlanError, labels_for_result


def validate_static_route(result, tomogram, bridge, *, builder=None, map_version_id=''):
    if (not isinstance(result, dict)
            or result.get('source_tomogram_sha256') != tomogram.sha256
            or result.get('execution_authorized') is not False):
        raise GlobalPlanError('native_result_map_or_shadow_contract_mismatch')
    same_floor = None
    if result.get('route_type') == 'same_floor':
        same_floor = {'lower':'floor1','upper':'floor2'}.get(result.get('floor'))
        if same_floor is None:
            raise GlobalPlanError('native_same_floor_id_invalid')
    points, labels = labels_for_result(result,same_floor=same_floor)
    layers = np.asarray(result.get('layer_ids'),dtype=int)
    if len(points) > 20000 or len(layers) != len(points):
        raise GlobalPlanError('native_result_size_or_layers_invalid')
    tomogram.validate_path(points,layers)
    if builder is None or not map_version_id:
        raise GlobalPlanError('source_route_builder_and_map_identity_required')
    # Preserve physical source-layer IDs even when the PCT runtime returns
    # compact indices for a same-floor route.
    enriched = dict(result, source_layer_ids=tomogram.source_layers[layers].astype(int).tolist())
    snapshot = builder.build(enriched, points, labels, map_version_id=map_version_id)
    payload = snapshot.payload()
    xyz = np.asarray(payload['xyz'],dtype=float)
    if xyz.shape != points.shape or not np.isfinite(xyz).all():
        raise GlobalPlanError('source_frame_ground_path_invalid')
    xyz = xyz.copy()
    xyz.flags.writeable = False
    return dict(xyz=xyz,diagnostics=payload['geometry_evidence']['projection'],
                source_tomogram_sha256=tomogram.sha256, route_snapshot=snapshot)


def _timed(call):
    began = time.monotonic()
    try:
        return call(),None,time.monotonic()-began
    except Exception as exc:
        return None,exc,time.monotonic()-began


class StaticRouteValidator:
    """One running + one latest pending job, never an unbounded executor queue.

    Called only by the ROS owner thread. Workers read immutable map/route data,
    never touch the Node, publish or authorize motion. Revocation cannot stop
    an already running Python function but immediately retires its generation.
    """
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=1,thread_name_prefix='pct-static-proof')
        self.future = None
        self.running_generation = None
        self.pending = None
        self.desired_generation = None
        self.last_generation = 0
        self.closed = False

    def submit(self,generation,call):
        if self.closed or type(generation) is not int or generation <= self.last_generation:
            raise ValueError('fresh_static_validation_generation_required')
        self.last_generation = self.desired_generation = generation
        self.pending = (generation,call)
        self._start_pending()

    def _start_pending(self):
        if self.future is None and self.pending is not None:
            self.running_generation,call = self.pending
            self.pending = None
            self.future = self.executor.submit(_timed,call)

    def poll(self):
        if self.future is None or not self.future.done():
            return None
        generation = self.running_generation
        value = self.future.result() if not self.future.cancelled() else None
        self.future = self.running_generation = None
        self._start_pending()
        if generation == self.desired_generation and value is not None:
            return (generation,*value)
        return None

    def invalidate(self):
        self.desired_generation = None
        self.pending = None
        if self.future is not None:
            self.future.cancel()

    def close(self):
        self.invalidate()
        self.closed = True
        self.executor.shutdown(wait=False,cancel_futures=True)
