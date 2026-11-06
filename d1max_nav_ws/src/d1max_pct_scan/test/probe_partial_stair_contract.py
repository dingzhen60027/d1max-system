#!/usr/bin/env python3
"""Replay saved native results through production source/reference contracts.

No ROS graph, native planning rerun, SDK, service or deployment is performed.
Only the explicitly requested result file is written. This verifies reference
handoff, not stair locomotion or collision clearance from live lidar.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from d1max_pct_scan.navigation_map_package import validate_package
from d1max_pct_scan.reference_support import ConditionedReferenceSupport
from d1max_pct_scan.source_route import SourceRouteBuilder, digest_file
from d1max_pct_scan.source_route_ros import to_message, from_message
from d1max_pct_scan.static_route_validation import validate_static_route
from test_continuous_reference_node import harness, local_geometry_commit, native_received
from test_continuous_reference_transport import BASE_NS, envelope, state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-results', required=True, type=Path)
    parser.add_argument('--map-directory', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    native = json.loads(args.native_results.read_text())
    package = validate_package(args.map_directory, purpose='planning_only')
    grid, bridge = package['tomogram'], package['bridge']
    assert native['map_sha'] == grid.sha256, 'native report and actual map differ'
    builder = SourceRouteBuilder.from_artifacts(package['manifest_path'], grid, bridge)
    provider = ConditionedReferenceSupport(package['manifest'], builder.support, grid)
    records = []
    for case in native['cases']:
        record = dict(begin=case['begin'], end=case['end'], ok=False)
        try:
            assert case['ok'], case.get('error', 'native route missing')
            raw = deepcopy(case['result'])
            checked = validate_static_route(raw, grid, bridge, builder=builder, map_version_id='map')
            route = checked['route_snapshot']
            value = route.payload()
            node, clock, output = harness(execution_mode_policy='fixed_stair', geometry_owner='local_executor',
                expected_source_map_sha256=value['source_map_sha256'],
                expected_conditioning_sha256=value['conditioning_sha256'],
                expected_tomogram_sha256=value['tomogram_sha256'])
            node.support_map = provider
            start = np.asarray(value['xyz'][0])
            sample = state(t=.01)
            for odometry in (sample.global_odometry, sample.local_odometry):
                odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z = (
                    float(start[0]), float(start[1]), float(start[2]+.55))
            clock.ns, clock.mono = BASE_NS+10_000_000, 100.01
            node.on_navigation(sample)
            message = envelope(t=.01)
            message.route_hash = route.route_hash
            message.snapshot = to_message(route, session_id='session', task_id='task', route_id='route',
                epoch=1, seed_id='seed', stamp=message.source_stamp)
            assert from_message(message.snapshot).encoded == route.encoded
            node.on_route(message)
            assert node.proposal is not None, node.error
            proposal = native_received(node)
            support = node.prepared_for(proposal.version).support
            record.update(support_verified=support.verified, support_reason=support.reason,
                          support_points=len(support.support_ground_xyz))
            assert support.verified, support.reason
            assert support.floor_id == value['segments'][0]['floor_id']
            assert support.required_mode == value['segments'][0]['required_mode']
            actual_goal = [proposal.goal_position.x, proposal.goal_position.y, proposal.goal_position.z]
            expected_goal = np.asarray(value['xyz'][-1])+[0., 0., .55]
            np.testing.assert_allclose(actual_goal, expected_goal, rtol=0., atol=1e-12)
            assert proposal.reference.point_reference == 'body_center'
            assert proposal.reference.path.header.stamp == sample.source_stamp
            assert value['geometry_evidence']['ground_to_body_height_applications'] == 0
            commit = local_geometry_commit(node, clock, proposal)
            assert node.on_local_commit(commit), node.error
            assert node.transport.snapshot.route_hash == route.route_hash
            assert node.permit is None
            assert not [name for name, _ in output if name in ('sdk', 'motion', 'handoff')]
            record.update(ok=True, route_hash=route.route_hash, direction=value['direction'],
                segments=[{k:s[k] for k in ('segment_id','kind','floor_id','required_mode')} for s in value['segments']],
                selected_source_ground=value['xyz'][-1], proposed_body_goal=actual_goal,
                ground_to_body_applications=node.transport.core.window(
                    current_source_ns=clock.ns, now_monotonic=clock.mono).ground_to_body_height_applications,
                first_window_points=len(proposal.reference.path.poses),
                route_points=len(value['xyz']), physical_execution_authorized=False,
                source_support=value['geometry_evidence']['measured_source_support'])
        except Exception as error:
            record.update(error=str(error), error_type=type(error).__name__)
        records.append(record)
        print(case['begin'], case['end'], 'PASS' if record['ok'] else 'FAIL',
              record.get('error', record.get('support_reason')), flush=True)
    report = dict(schema=1, native_results=str(args.native_results.resolve()),
        native_results_sha256=digest_file(args.native_results), map_directory=str(args.map_directory.resolve()),
        map_sha256=grid.sha256, source_map_sha256=builder.provenance['source_map_sha256'],
        checks='production static route -> source snapshot -> typed wire -> reference/support -> geometry commit',
        no_ros_graph=True, no_sdk=True, not_a_native_scan_or_motion_test=True,
        cases=records, passed=sum(item['ok'] for item in records), total=len(records))
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['passed'] == report['total'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
