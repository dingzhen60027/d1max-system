"""Read-only source wiring checks for event logs; no ROS or robot process.

Runtime transition semantics are exercised by the real BehaviorTree.CPP
tests in test_engine.cpp. These checks only prevent silently losing the
task-specific evidence again, or moving it into every control/status tick.
"""
from pathlib import Path
import re


SOURCE = (Path(__file__).parents[1] / 'src' / 'navigator_node.cpp').read_text()


def test_task_lifecycle_events_have_explicit_identity_and_original_goal_time():
    events = set(re.findall(r'logTaskEvent\([^;]*?,"([a-z_]+)"', SOURCE))
    assert events == {'goal_accepted', 'compute_requested', 'compute_rejected',
                      'compute_result', 'route_committed', 'follow_requested',
                      'follow_rejected', 'follow_result', 'task_finished',
                      'retirement_started', 'public_result'}
    function = SOURCE.split('  void logTaskEvent(', 1)[1].split('  bool taskFollower()', 1)[0]
    for field in ('task_id', 'route_id', 'route_hash', 'goal_source_ns',
                  'cancellation_requested', 'current_task', 'reason', 'sequence'):
        assert '"' + field + '"' in function
    assert 'goalStamp(task.goal)' in function
    assert 'noexcept' in function and 'catch(...)' in function


def test_diagnostic_log_is_not_a_per_tick_or_feedback_state_machine():
    for start, end in (('  void tick() {', '  void preparationTick()'),
                       ('  void publishStatus() {', '\n  std::string session_,')):
        assert 'logTaskEvent(' not in SOURCE.split(start, 1)[1].split(end, 1)[0]
    for match in re.finditer(r'options\.feedback_callback = \[task\]', SOURCE):
        callback = SOURCE[match.start():].split('\n    };', 1)[0]
        assert 'logTaskEvent(' not in callback


def test_late_results_and_public_stop_outcome_remain_separate_events():
    for operation in ('compute', 'follow'):
        callback = SOURCE.split(f'options.result_callback = [this, task](const {operation.title()}Handle', 1)[1]
        callback = callback.split('\n    };', 1)[0]
        assert callback.index('"' + operation + '_result"') < callback.index('if (!accepted) return;')
        assert '"result_accepted",accepted' in callback
        assert '"worker_retirement_confirmed"' in callback
    public = SOURCE.split('  void completePublicResult(', 1)[1].split('  void reclaimCompletedTasks()', 1)[0]
    assert public.index('if (!task->finished || task->public_terminal) return;') < public.index('"public_result"')
    assert '"physical_stop_required"' in public and '"measured_stop_confirmed"' in public
