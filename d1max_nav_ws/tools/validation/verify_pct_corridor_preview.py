#!/usr/bin/env python3
"""Verify a preserved goal in the private planning-only RViz session. No SDK/control."""
import argparse
import json
from pathlib import Path
import sys

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / 'src/d1max_pct_planner'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--preserved-path', type=Path, required=True)
    args = parser.parse_args()
    snapshot = json.loads((args.session / 'session.json').read_text())
    if (snapshot['mode'] != 'planning_preview' or snapshot['robot_connected']
            or snapshot['real_motion_enabled'] or snapshot['router_port'] != 7465):
        raise ValueError('Only an isolated, motion-disabled preview is permitted')
    output = args.session / 'quality_live_verification.json'
    if output.exists():
        raise FileExistsError(output)
    from d1max_pct_planner.preview_session import runtime_environment
    runtime_environment(args.session)
    import rclpy
    from d1max_pct_planner import preview_verify
    from d1max_pct_planner.path_quality import path_quality
    from d1max_pct_planner.tomogram_map import TomogramMap
    preview_verify.FRAME = 'd1max_flat_floor_planning'
    old = json.loads(args.preserved_path.read_text())
    report = {'passed': False, 'checks': [], 'mode': 'private_planning_only',
              'rendered_screen_inspected': False, 'robot_control_used': False}
    rclpy.init()
    node = preview_verify.PreviewVerifier(report)
    try:
        node.wait(lambda: node.status is not None
                  and node.status['state'] != 'initializing', 12., 'PCT ready')
        if node.status['robot_connected'] or node.status['motion_enabled']:
            raise AssertionError('Unexpected robot/control-enabled status')
        node.wait(lambda: all(p.get_subscription_count() for p in node.points.values())
                  and node.commands['plan'].get_subscription_count() > 0,
                  6., 'Preview point/plan subscribers')
        node.clear()
        for role in ('start', 'goal'):
            xyz = old[role + '_xyz']
            node.select(role, xyz, frame=preview_verify.FRAME)
            node.wait(lambda: node.matches(role, xyz), 4., role + ' exact XYZ selection')
        points = node.plan(old['start_xyz'], old['goal_xyz'])
        result = node.status['result']
        if not result.get('path_refinement', {}).get('applied'):
            raise AssertionError('This regression must use the checked corridor refinement')
        tomo = TomogramMap(WS / 'maps/processed/sc_pgo_20260919_pct_flat_floor_v6_20260923/tomogram.npz',
                          minimum_headroom_m=.55, max_ground_step_m=.17)
        checked = tomo.validate_path(points, result['layer_ids'])
        subscribers = node.get_subscriptions_info_by_topic(preview_verify.PREFIX + '/path')
        rviz = [s.node_name for s in subscribers if s.node_name == 'pct_preview_rviz']
        if not rviz:
            raise AssertionError('RViz must subscribe to the published path')
        report.update(passed=True, status=node.status, path_quality=path_quality(points),
                      independent_published_path_validation=checked,
                      rviz_path_subscribers=rviz, events=node.events,
                      preserved_source=str(args.preserved_path))
    except Exception as exc:
        report.update(error=str(exc), status=node.status, events=node.events)
        raise
    finally:
        with output.open('x') as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'passed': report['passed'], 'report': str(output),
                      'quality': report.get('path_quality'),
                      'rviz_path_subscribers': report.get('rviz_path_subscribers')},
                     ensure_ascii=False))


if __name__ == '__main__':
    main()
