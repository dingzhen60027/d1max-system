#!/usr/bin/env python3
"""Same immutable route and deliberately stalled IPC, without a ROS graph."""
import argparse
from dataclasses import asdict
import importlib.util
import hashlib
import json
import multiprocessing
from multiprocessing.reduction import ForkingPickler
import os
from pathlib import Path
import statistics
import struct
import threading
import time
import tracemalloc
from types import SimpleNamespace

import numpy as np
from d1max_pct_scan.bounded_worker_channel import BoundedWorkerChannel
from d1max_pct_scan.continuous_reference import ContinuousReference, Observation
from d1max_pct_scan.control_frame_contract import BodySample, Context, Rigid, create_anchor
from d1max_pct_scan.source_route import SourceRouteBuilder


def timings(function, repeats):
    values = []
    for _ in range(repeats):
        started = time.perf_counter_ns()
        result = function()
        values.append((time.perf_counter_ns() - started) / 1000.)
    tracemalloc.start()
    result = function()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return result, dict(repeats=repeats, p50_us=statistics.median(values),
        p95_us=float(np.percentile(values, 95)), max_us=max(values), python_peak_bytes=peak)


def route_benchmark(baseline_path):
    spec = importlib.util.spec_from_file_location('d1max_pct_scan._reference_before', baseline_path)
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    class Bridge:
        source_frame = 'd1max_loc_map'
        planning_frame = 'd1max_multifloor_planning'
        def to_localization_ground(self, points, labels):
            return SimpleNamespace(xyz=np.asarray(points), diagnostics={})
    points = [[i * .01, 0., 0.] for i in range(20000)]
    snapshot = SourceRouteBuilder(bridge=Bridge(), source_map_sha256='a'*64,
        conditioning_sha256='b'*64, tomogram_sha256='c'*64).build(
            dict(route_type='same_floor', layer_ids=[4]*len(points), source_layer_ids=[4]*len(points)),
            points, ['floor1']*len(points), map_version_id='map')
    context = Context('session', 1, 'seed', 'map')
    def sample(t=0., x=0., frame='d1max_loc_odom'):
        return Observation(BodySample(context, frame, 'd1max_loc_base_link',
            10_000_000_000+round(t*1e9), Rigid((x, 0., .55), (0., 0., 0., 1.))), 100.+t)
    anchor = create_anchor(sample(frame='d1max_loc_map').body, sample().body, 1)
    output = {'route_hash': snapshot.route_hash, 'point_count': len(points), 'encoded_bytes': len(snapshot.encoded),
        'baseline_source_sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
        'current_source_sha256': hashlib.sha256(Path(__file__).resolve().parents[2].joinpath(
            'src/d1max_pct_scan/d1max_pct_scan/continuous_reference.py').read_bytes()).hexdigest()}
    signatures = []
    for name, cls in [('before_b344f52', module.ContinuousReference), ('after', ContinuousReference)]:
        construct = lambda: cls(snapshot, context=context, task_id='task', anchor=anchor,
            body_height_m=.55, body_height_calibration_id='benchmark-unverified')
        ref, construction = timings(construct, 9)
        ref.project(sample(), current_source_ns=10_000_000_000, now_monotonic=100.)
        local, global_ = sample(.1), sample(.1, x=.02, frame='d1max_loc_map')
        ref.observe_pair(global_, local, revision=2, current_source_ns=local.body.source_ns, now_monotonic=100.1)
        prepare = lambda: ref.prepare_reanchor(local, current_source_ns=local.body.source_ns, now_monotonic=100.1)
        staged, reanchor = timings(prepare, 31)
        signatures.append((asdict(staged._progress), asdict(staged.window(
            current_source_ns=local.body.source_ns, now_monotonic=100.1))))
        output[name] = dict(construct=construction, reanchor=reanchor)
    assert signatures[0] == signatures[1], 'reference_geometry_or_source_semantics_changed'
    output['same_output'] = True
    return output


def ipc_benchmark():
    packet = {'kind': 'result', 'generation': 1, 'xyz': [[0., 1., 2.]]}
    encoded = ForkingPickler.dumps(packet)
    def once(wrapped):
        parent, peer = multiprocessing.Pipe()
        channel = BoundedWorkerChannel(parent) if wrapped else parent
        header_sent = threading.Event()
        release_payload = threading.Event()
        def producer():
            os.write(peer.fileno(), struct.pack('!i', len(encoded)))
            header_sent.set()
            release_payload.wait()
            time.sleep(.05)  # controlled transport stall, not claimed network measurement
            os.write(peer.fileno(), encoded)
        thread = threading.Thread(target=producer)
        thread.start()
        assert header_sent.wait(1.)
        if not wrapped:
            assert channel.poll(0)
        release_payload.set()
        started = time.perf_counter_ns()
        if wrapped:
            assert not channel.poll(0)
        else:
            assert channel.recv() == packet
        elapsed = (time.perf_counter_ns()-started)/1000.
        thread.join(1.)
        if wrapped:
            deadline = time.monotonic()+1.
            while not channel.poll(0) and time.monotonic()<deadline:
                time.sleep(.001)
            assert channel.recv() == packet
        channel.close()
        peer.close()
        if wrapped:
            deadline = time.monotonic()+1.
            while not channel.retirement_ready and time.monotonic()<deadline:
                time.sleep(.001)
            assert channel.retirement_ready
        return elapsed
    output = dict(payload_delay_ms=50., request_capacity=1, reply_capacity=4,
                  max_request_bytes=4096, max_reply_bytes=BoundedWorkerChannel.MAX_REPLY_BYTES)
    for name, wrapped in [('before_raw_pipe', False), ('after', True)]:
        values = [once(wrapped) for _ in range(7)]
        output[name] = dict(owner_p50_us=statistics.median(values), owner_max_us=max(values))
    output['same_complete_packet'] = True
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', type=Path, default=Path(__file__).resolve().parents[2] /
        'experiments/navigation_performance_20261004/owner_before/continuous_reference.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.baseline.is_file():
        raise SystemExit('explicit_published_baseline_path_required')
    report = dict(scope='pure production reference core + controlled partial IPC; no ROS/SDK/replay',
                  route=route_benchmark(args.baseline), ipc=ipc_benchmark())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
