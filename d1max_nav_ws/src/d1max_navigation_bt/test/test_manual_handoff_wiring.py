"""Source-level transport contract: handoff cannot depend on discovery order.

Core event ordering, identity and replay behavior are exercised by the compiled
SDKManualControlFence tests. These guards cover the actual ROS wiring only.
"""
from pathlib import Path
import re


SOURCE = (Path(__file__).resolve().parents[1] / 'src' / 'navigator_node.cpp').read_text()


def test_event_and_ack_are_both_retained_for_late_discovery():
    ack = re.search(r'manual_handoff_ack_pub_=.*?;', SOURCE, re.S).group()
    event = re.search(r'manual_handoff_sub_=.*?\[this\]', SOURCE, re.S).group()
    assert '"/d1max_sdk_bridge/manual_handoff_ack"' in ack
    assert '"/d1max_sdk_bridge/manual_handoff"' in event
    assert 'rclcpp::QoS(1).reliable().transient_local()' in ack
    assert 'rclcpp::QoS(1).reliable().transient_local()' in event
    # The event can arrive during lifecycle setup: receipt ACKs are not gated
    # by activation of a lifecycle publisher.
    assert 'rclcpp::create_publisher<std_msgs::msg::String>' in ack


def test_ack_is_bound_to_exact_session_and_lease_after_intent_retirement():
    callback = SOURCE.split('manual_handoff_sub_=', 1)[1].split('sdk_state_sub_=', 1)[0]
    assert 'if(!decision.accepted)return;' in callback
    assert '{"sdk_session",expected_sdk_session_}' in callback
    assert '{"lease",event.at("lease")}' in callback
    assert callback.index('cancelManualControlIntents(decision.cutoff)') < callback.index(
        'manual_handoff_ack_pub_->publish(ack)')
    # Duplicate events may re-ACK but may never cancel a newly submitted goal.
    assert 'if(!decision.duplicate)cancelManualControlIntents(decision.cutoff)' in callback


def test_already_draining_tasks_receive_retained_stop_semantics_without_recancellation():
    callback = SOURCE.split('void cancelManualControlIntents(', 1)[1].split('void onHealth(', 1)[0]
    draining = callback.split('if(task.retired||task.finished)', 1)[1].split('continue;', 1)[0]
    assert 'physicalRequired(task)&&!stopConfirmed(task)' in draining
    assert 'task.follower->stopForManualTakeover(cutoff*1e-9)' in draining
    assert 'cancel_current' not in draining
    assert 'engine_->cancel' not in draining
