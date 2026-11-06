"""Guard actual side-effect wiring; ROS behavior is exercised by the private probe."""
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / 'src' / 'navigator_node.cpp').read_text()


def section(start, end):
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def test_only_planning_operations_use_the_software_retirement_barrier():
    for start, end in (
        ('ReadyState inputsReady(', 'void requestRoute('),
        ('void requestRoute(', 'Result pollRoute('),
        ('void requestFollow(', 'Result pollFollow('),
        ('void tick() {', 'void preparationTick()'),
    ):
        assert 'planningOperationsDone()' in section(start, end)
    assert 'planningRetirementFault()' in section('ReadyState contextValid(', 'ReadyState initialLocalizationReady(')
    assert 'goal.execution_confirmed = false;' in section('void requestFollow(', 'Result pollFollow(')


def test_sdk_grant_and_lifecycle_keep_the_full_stop_barrier():
    confirm = section('void confirmTaskFollower(', 'void taskFollowerTick(')
    assert 'response.reason=navigationAdmissionReason();if(!response.reason.empty())return;' in confirm
    admission = section('std::string navigationAdmissionReason(', 'static bool validateYaw(')
    assert 'retiredOperationsDone();' in admission
    assert 'physicalRequired(*t)&&!stopConfirmed(*t)' in admission
    assert 'planningOperationsDone' not in admission
    lifecycle = section('nav2_util::CallbackReturn on_activate(', 'ReadyState contextValid(')
    assert lifecycle.count('retiredOperationsDone()') >= 2


def test_legacy_barrier_is_not_relaxed():
    planning = section('bool planningOperationsDone()', 'void completePublicResult(')
    assert 'planningRetirementReady(taskFollower(),softwareOperationsDone(),fully_retired)' in planning
    assert 'planningRetirementReason(taskFollower(),retirement_fault_)' in planning
