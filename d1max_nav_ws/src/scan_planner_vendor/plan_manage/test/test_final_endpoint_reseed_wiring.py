"""File-only production wiring checks; do not construct a ROS graph.

The short-connection predicate has executable C++ behavior tests. These checks
specifically retain its qualified final-target marker across the async worker
snapshot and verify that ordinary reference updates clear the marker.
"""
from pathlib import Path
import re

PACKAGE = Path(__file__).resolve().parents[1]


def test_async_worker_copies_the_selected_final_target_marker():
    source = (PACKAGE / 'src/planner_manager.cpp').read_text()
    begin = source.index('void SCANPlannerManager::prepareSolveWorker(')
    end = source.index('void SCANPlannerManager::', begin + 1)
    worker_snapshot = source[begin:end]
    assert 'local_reference_=owner.local_reference_;' in worker_snapshot
    assert 'local_reference_support_=owner.local_reference_support_;' in worker_snapshot
    assert 'final_endpoint_reseed_=owner.final_endpoint_reseed_;' in worker_snapshot


def test_reference_setter_defaults_to_normal_solve_and_overwrites_marker():
    source = (PACKAGE / 'include/plan_manage/planner_manager.h').read_text()
    assert re.search(r'setLocalReference\(const std::vector<Eigen::Vector3d> &points,\s*'
                     r'bool final_endpoint_reseed=false,\s*'
                     r'bool has_start_projection_connector=false,\s*'
                     r'std::shared_ptr<const PlanningSupport> geometry_support=\{\}\)', source)
    assert 'local_reference_=points;final_endpoint_reseed_=final_endpoint_reseed;' in source
    assert 'local_reference_support_=std::move(geometry_support);' in source
    assert 'bool final_endpoint_reseed_{false};' in source


def test_short_solve_exception_is_bound_to_actual_reference_endpoints():
    source = (PACKAGE / 'src/planner_manager.cpp').read_text()
    assert re.search(r'if\s*\(\(start_pt - local_target_pt\)\.norm\(\) < 0\.2\s*&&\s*'
                     r'!finalEndpointShortConnectionAllowed\(final_endpoint_reseed_,local_reference_,start_pt,\s*'
                     r'local_target_pt,planned_splice_\.has_value\(\)\)\)', source)
    assert source.index('finalEndpointShortConnectionAllowed(') < source.index('waiting_goal_reached')
    assert source.index('waiting_goal_reached') < source.index('buildCollisionAwareReference(')


def test_only_the_final_endpoint_selection_marks_a_short_solve():
    source = (PACKAGE / 'src/scan_replan_fsm.cpp').read_text()
    assert re.search(r'setLocalReference\(local_debug_selected_reference_,\s*'
                     r'selection\.reason\s*==\s*"reference_target_final_endpoint_reseed",\s*'
                     r'has_start_projection_connector,\s*source_support\)', source)


if __name__ == '__main__':
    test_async_worker_copies_the_selected_final_target_marker()
    test_reference_setter_defaults_to_normal_solve_and_overwrites_marker()
    test_short_solve_exception_is_bound_to_actual_reference_endpoints()
    test_only_the_final_endpoint_selection_marks_a_short_solve()
    print('PASS: four final endpoint solve/snapshot wiring checks (no ROS graph)')
