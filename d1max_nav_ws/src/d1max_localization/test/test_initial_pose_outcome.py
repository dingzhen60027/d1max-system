"""Exact localization owner methods without creating a ROS node or endpoints."""
import json
import time
from unittest.mock import Mock
import pytest
import rclpy
from test_lio_localizer import node, sample, status, startup_node
from d1max_localization.lio_localizer import initial_pose_outcome, transaction_intent_stamp


def command(**changes):
    value=dict(id='intent',session_id='test',created_at=time.time(),reference='body',
        x=1.,y=2.,z=.5,yaw=0.,initial_pose_transaction='bt_initial_pose_transaction_v1',
        intent_source_stamp_ns='99000000000',source_map_sha256='a'*64)
    value.update(changes)
    return value


def test_seed_applied_outcome_is_actual_identity_not_localization_success(node):
    sample(node);status(node,1);node.session['version_id']='version';node.initial_outcome_pub=Mock()
    (node.directory/'initial_pose.json').write_text(json.dumps(command()))
    node.read_command()
    result=node.initial_outcome_pub.publish.call_args.args[0]
    assert result.accepted and result.applied and result.request_id=='intent'
    assert result.localization_epoch==1 and result.localization_seed_id==node.active_seed=='100000000000'
    assert result.intent_source_stamp.sec==99 and result.source_stamp.sec==100
    assert node.confirmed_seed is None and node.verified_confirmations==0
    assert node.last_applied_initial_intent_ns==99000000000
    node.read_command();assert node.initial_outcome_pub.publish.call_count==1


def test_known_sensor_rejection_reports_unapplied_matching_request(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    (node.directory/'initial_pose.json').write_text(json.dumps(command()))
    node.read_command();result=node.initial_outcome_pub.publish.call_args.args[0]
    assert not result.accepted and not result.applied and result.request_id=='intent'
    node.seed_pub.publish.assert_not_called()
    assert node.last_applied_initial_intent_ns==0
    assert node.last_attempted_initial_intent_ns==0


def test_partial_seed_failure_cannot_report_known_unapplied(node):
    sample(node);status(node,1);node.session['version_id']='version';node.initial_outcome_pub=Mock()
    node.seed_pub.publish.side_effect=ValueError('injected publisher failure')
    (node.directory/'initial_pose.json').write_text(json.dumps(command()))
    node.read_command();result=node.initial_outcome_pub.publish.call_args.args[0]
    assert not result.accepted and result.applied
    assert node.last_applied_initial_intent_ns==0
    assert node.last_attempted_initial_intent_ns==99000000000


def submit(node, value, *, at=100.):
    """Only the fixture mailbox and mocked publishers; never a ROS endpoint."""
    sample(node,t=at);status(node,1)
    node.get_clock.return_value.now.return_value=rclpy.time.Time(seconds=at)
    (node.directory/'initial_pose.json').write_text(json.dumps(value))
    node.read_command()
    return node.initial_outcome_pub.publish.call_args.args[0]


def test_late_or_equal_distinct_typed_intent_cannot_replace_applied_new_seed(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    newer=command(id='newer',intent_source_stamp_ns='99000000002')
    assert submit(node,newer).accepted
    active=node.active_seed;seed_time=node.core.seed_time
    for identifier,source in (('late','99000000001'),('equal','99000000002')):
        rejected=submit(node,command(id=identifier,intent_source_stamp_ns=source),at=100.01)
        assert not rejected.accepted and not rejected.applied
        assert rejected.request_id==identifier and '源时间' in rejected.reason
        assert node.last_applied_initial_intent_ns==99000000002
        assert node.active_seed==active and node.core.seed_time==seed_time
        assert node.seed_pub.publish.call_count==1
    # A rejected late UUID must not erase the original accepted UUID's dedup
    # fact and turn its reliable duplicate into a second negative outcome.
    outcomes=node.initial_outcome_pub.publish.call_count
    submit(node,newer,at=100.015)
    assert node.seed_pub.publish.call_count==1
    assert node.initial_outcome_pub.publish.call_count==outcomes
    assert node.last_applied_initial_intent_id=='newer'
    assert node.last_applied_initial_intent_ns==99000000002
    # Same XYZ is not a duplicate intent. A genuinely newer source/new UUID
    # reseeds through the same consumer without rewriting the original time.
    fresh=command(id='fresh',intent_source_stamp_ns='99000000003')
    result=submit(node,fresh,at=100.02)
    assert result.accepted and result.applied and result.request_id=='fresh'
    assert result.intent_source_stamp.sec==99 and result.intent_source_stamp.nanosec==3
    assert node.last_applied_initial_intent_ns==99000000003
    assert node.active_seed!=active and node.core.seed_time==100.02
    assert node.seed_pub.publish.call_count==2
    assert node.seed_pub.publish.call_args_list[0].args[0].pose.pose.position==node.seed_pub.publish.call_args_list[1].args[0].pose.pose.position
    assert json.loads((node.directory/'initial_pose.json').read_text())['intent_source_stamp_ns']=='99000000003'


def test_known_rejection_does_not_advance_applied_watermark_or_automatically_retry_same_id(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    value=command(id='not-ready',intent_source_stamp_ns='99000000002')
    (node.directory/'initial_pose.json').write_text(json.dumps(value))
    node.read_command()
    rejected=node.initial_outcome_pub.publish.call_args.args[0]
    assert not rejected.accepted and not rejected.applied
    assert node.last_applied_initial_intent_ns==0
    assert node.last_attempted_initial_intent_ns==0
    sample(node);status(node,1)
    node.read_command()
    node.seed_pub.publish.assert_not_called()
    assert node.initial_outcome_pub.publish.call_count==1
    # A separate, valid request may reuse a source never successfully applied;
    # this does not resurrect or automatically retry the rejected old UUID.
    result=submit(node,command(id='valid-new-id',intent_source_stamp_ns='99000000002'))
    assert result.accepted and result.applied
    assert node.last_applied_initial_intent_ns==99000000002
    assert node.seed_pub.publish.call_count==1


@pytest.mark.parametrize('changes', [
    {'session_id':'foreign'}, {'intent_source_stamp_ns':'099000000002'},
    {'created_at':0.}, {'x':float('nan')}])
def test_exact_invalid_manual_failure_cannot_poison_successful_applied_watermark(node,changes):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    assert submit(node,command(id='accepted',intent_source_stamp_ns='99000000001')).accepted
    active=node.active_seed
    invalid=command(id='invalid',intent_source_stamp_ns='99000000003')
    invalid.update(changes)
    submit(node,invalid,at=100.01)
    assert not node.command_result['accepted']
    assert node.last_applied_initial_intent_ns==99000000001
    assert node.last_attempted_initial_intent_ns==99000000001
    assert node.active_seed==active and node.seed_pub.publish.call_count==1
    assert submit(node,command(id='valid',intent_source_stamp_ns='99000000002'),at=100.02).accepted
    assert node.last_applied_initial_intent_ns==99000000002


def test_legacy_seed_does_not_read_or_advance_typed_manual_intent_fence(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    assert submit(node,command(id='typed',intent_source_stamp_ns='99000000002')).accepted
    legacy=command(id='legacy',intent_source_stamp_ns='99000000001')
    legacy.pop('initial_pose_transaction')
    submit(node,legacy,at=100.01)
    assert node.command_result['accepted'] and node.seed_pub.publish.call_count==2
    assert node.initial_outcome_pub.publish.call_count==1  # No typed outcome fabricated.
    assert node.last_applied_initial_intent_ns==99000000002
    assert node.last_attempted_initial_intent_ns==99000000002


@pytest.mark.parametrize('global_backend',[False,True])
def test_partial_application_fences_older_and_equal_new_uuid_without_claiming_success(node,global_backend):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    if global_backend:node=startup_node(node)
    assert submit(node,command(id='accepted',intent_source_stamp_ns='99000000100')).accepted
    previous=node.last_applied_initial_intent_ns
    node.seed_pub.publish.side_effect=ValueError('injected publish failure after core.seed')
    partial_command=command(id='partial',intent_source_stamp_ns='99000000200',x=5.)
    partial=submit(node,partial_command,at=100.01)
    assert not partial.accepted and partial.applied
    assert node.core.seed_time==100.01
    assert node.last_applied_initial_intent_ns==previous
    assert node.last_attempted_initial_intent_ns==99000000200
    assert node.last_attempted_initial_intent_id=='partial'
    node.seed_pub.publish.side_effect=None
    calls=node.seed_pub.publish.call_count
    for identifier,source in (('late','99000000150'),('equal-new-id','99000000200')):
        rejected=submit(node,command(id=identifier,intent_source_stamp_ns=source),at=100.02)
        assert not rejected.accepted and not rejected.applied
        assert node.core.seed_time==100.01 and node.seed_pub.publish.call_count==calls
        assert node.last_applied_initial_intent_ns==previous
        assert node.last_attempted_initial_intent_ns==99000000200
    # Re-delivery of the partial request retains its UNKNOWN partial outcome;
    # it cannot fabricate a later known-unapplied result for that same UUID.
    outcomes=node.initial_outcome_pub.publish.call_count
    submit(node,partial_command,at=100.02)
    assert node.initial_outcome_pub.publish.call_count==outcomes
    assert node.seed_pub.publish.call_count==calls
    fresh=submit(node,command(id='fresh',intent_source_stamp_ns='99000000201',x=5.),at=100.03)
    assert fresh.accepted and fresh.applied
    assert node.last_applied_initial_intent_ns==node.last_attempted_initial_intent_ns==99000000201
    assert node.seed_pub.publish.call_count==calls+1 and node.core.seed_time==100.03


def test_global_subclass_manual_watermark_fences_late_mailbox_and_old_busy_worker(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    owner=startup_node(node);worker=owner.relocalization_worker
    worker.poll.return_value=dict(kind='result',accepted=True,
        request_id=owner.startup.request['request_id'],map_sha256='a'*64,
        candidate=dict(transform=[[1,0,0,9],[0,1,0,2],[0,0,1,0],[0,0,0,1]]))
    assert submit(owner,command(id='manual',intent_source_stamp_ns='99000000002')).accepted
    worker.close.assert_called_once()
    assert owner.startup.reason.startswith('manual_seed') and owner.startup.request is None
    active=owner.active_seed
    late=submit(owner,command(id='late',intent_source_stamp_ns='99000000001'),at=100.01)
    assert not late.accepted and not late.applied and owner.active_seed==active
    owner.poll_relocalization(100.01)
    worker.poll.assert_not_called()
    assert owner.seed_pub.publish.call_count==1 and owner.last_applied_initial_intent_ns==99000000002
    assert submit(owner,command(id='manual-new',intent_source_stamp_ns='99000000003'),at=100.02).accepted
    assert owner.seed_pub.publish.call_count==2 and owner.active_seed!=active
    assert owner.startup.seed_id==owner.active_seed and owner.last_applied_initial_intent_ns==99000000003
    worker.close.assert_called_once()


@pytest.mark.parametrize('stamp',['0','099000000000','99.0','-1',99,True,str(2**63),'９９'])
def test_malformed_intent_stamp_does_not_fake_matching_outcome(stamp):
    with pytest.raises(ValueError):transaction_intent_stamp(command(intent_source_stamp_ns=stamp),'test')


def test_legacy_internal_seed_does_not_impersonate_manual_transaction():
    assert initial_pose_outcome(dict(id='old'),session_id='test',map_version_id='version',
        source_ns=100000000000,epoch=1,seed='new',accepted=True,applied=True,reason='seed') is None


def test_foreign_session_and_non_ascii_identity_rejected():
    with pytest.raises(ValueError):transaction_intent_stamp(command(session_id='other'),'test')
    with pytest.raises(ValueError):transaction_intent_stamp(command(id='种子'),'test')
