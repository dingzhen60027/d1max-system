#!/usr/bin/env python3
"""Same recorded CDR/native GridMap comparison; no ROS graph, SDK or motion.

Requires the existing recorded replay harness, not a substitute map. Refuses
to overwrite results. Does not compile, install or start any service.
"""
import argparse
import importlib.util
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--harness', required=True, type=Path)
    parser.add_argument('--before', required=True, type=Path)
    parser.add_argument('--after', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--reverse-order', action='store_true',
        help='run after before before, preserving labels, to expose host/order timing bias')
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('same_recorded_native_replay',
                                                 args.harness.resolve(strict=True))
    replay = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(replay)
    binaries = {key:getattr(args, key).resolve(strict=True) for key in ('before', 'after')}
    output = args.output.resolve()
    output.mkdir(parents=False, exist_ok=False)
    reports, rows, summaries = {}, {}, {}
    parameters = replay.GRID_VARIANTS['official_ros2_grid_005']
    execution_order = ('after', 'before') if args.reverse_order else ('before', 'after')
    for label in execution_order:
        binary = binaries[label]
        report, directory = replay.run_variant(binary, output, label, parameters, diagnostics=False)
        reports[label], rows[label] = report, replay.events(directory)
        summaries[label] = replay.summarize_variant(report, directory, rows[label])
    def rays(events):
        fields = ('row_id', 'receipt_ns', 'cdr_sha256', 'cdr_bytes', 'sensor_id',
                  'projection_sequence', 'context_sequence', 'scan_begin_ns',
                  'acquisition_end_ns', 'alignment_stamp_ns')
        return [{k:e[k] for k in fields} for e in events if e['type']=='recorded_rays']
    def decisions(events):
        fields = ('label', 'position', 'yaw', 'query_policy', 'collision',
                  'counts_free_occupied_unknown_outside', 'first_occupied_unknown_outside')
        return [{k:q[k] for k in fields} for e in events if e['type']=='status_scheduled_tick'
                for q in e['result']['queries']]
    native_keys = ('raw_buffer_fnv1a64', 'raw_voxel_counts_free_occupied_unknown',
        'source_stamp_ns', 'source_stamps_ns', 'integrated_counts', 'drops', 'unattributed_drops',
        'revision', 'raw_occupancy_revision', 'free_evidence_revision',
        'enforce_free_freshness', 'require_observed_free', 'query_policy')
    def counters(events):
        return [{k:e['result'][k] for k in native_keys if k in e['result']}
                for e in events if e['type']=='status_scheduled_tick']
    a, b = reports['before'], reports['after']
    checks = dict(same_raw_cdr_and_metadata=rays(rows['before'])==rays(rows['after']),
        same_parameters=a['resolved_native_parameters']==b['resolved_native_parameters'],
        same_context=a['context_events']==b['context_events'],
        same_tick_clocks_and_query_requests=replay.tick_inputs(rows['before'])==replay.tick_inputs(rows['after']),
        same_all_collision_and_raw_query_results=decisions(rows['before'])==decisions(rows['after']),
        same_per_tick_fusion_and_clock_counters=counters(rows['before'])==counters(rows['after']),
        same_every_scan_acceptance=[e['result']['accepted'] for e in rows['before'] if e['type']=='recorded_rays']==[
            e['result']['accepted'] for e in rows['after'] if e['type']=='recorded_rays'],
        same_final_raw_map_and_source_freshness_state=all(a['native_final'][k]==b['native_final'][k] for k in native_keys),
        both_no_errors=a['error'] is None and b['error'] is None)
    result = dict(kind='EXACT_NATIVE_QUEUE_UPDATE_RECORDED_AB', invariants=checks,
        execution_order=execution_order,
        all_invariants_passed=all(checks.values()),
        probe_sha256={key:replay.digest(binary) for key,binary in binaries.items()},
        replay_harness_sha256=replay.digest(args.harness), variants=summaries,
        no_ros_initialized=True, no_robot_or_motion=True,
        limitations=['One ordered run at uncontrolled host load; not a NUC timing acceptance.',
            'Native tick includes complete integration and unchanged native collision/raw-overlap queries.',
            'Existing capture is truncated below its five-second warmup requirement.',
            'No trajectory optimizer, full live pipeline or physical geometry acceptance.'])
    (output/'comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(invariants=checks, variants={key:dict(
        native_tick_ms=value['native_processing_ms']['status_tick_integration_and_queries'],
        final_raw_hash=value['native_final']['raw_buffer_fnv1a64'])
        for key,value in summaries.items()}), indent=2))
    if not all(checks.values()):
        raise SystemExit('Same-input production GridMap evidence mismatch')


if __name__ == '__main__':
    main()
