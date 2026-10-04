"""Exact localization owner methods without creating a ROS node or endpoints."""
import json
import time
from unittest.mock import Mock
import pytest
import rclpy
from test_lio_localizer import node, sample, status
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
    node.read_command();assert node.initial_outcome_pub.publish.call_count==1


def test_known_sensor_rejection_reports_unapplied_matching_request(node):
    node.session['version_id']='version';node.initial_outcome_pub=Mock()
    (node.directory/'initial_pose.json').write_text(json.dumps(command()))
    node.read_command();result=node.initial_outcome_pub.publish.call_args.args[0]
    assert not result.accepted and not result.applied and result.request_id=='intent'
    node.seed_pub.publish.assert_not_called()


def test_partial_seed_failure_cannot_report_known_unapplied(node):
    sample(node);status(node,1);node.session['version_id']='version';node.initial_outcome_pub=Mock()
    node.seed_pub.publish.side_effect=ValueError('injected publisher failure')
    (node.directory/'initial_pose.json').write_text(json.dumps(command()))
    node.read_command();result=node.initial_outcome_pub.publish.call_args.args[0]
    assert not result.accepted and result.applied


@pytest.mark.parametrize('stamp',['0','099000000000','99.0','-1',99,True,str(2**63),'９９'])
def test_malformed_intent_stamp_does_not_fake_matching_outcome(stamp):
    with pytest.raises(ValueError):transaction_intent_stamp(command(intent_source_stamp_ns=stamp),'test')


def test_legacy_internal_seed_does_not_impersonate_manual_transaction():
    assert initial_pose_outcome(dict(id='old'),session_id='test',map_version_id='version',
        source_ns=100000000000,epoch=1,seed='new',accepted=True,applied=True,reason='seed') is None


def test_foreign_session_and_non_ascii_identity_rejected():
    with pytest.raises(ValueError):transaction_intent_stamp(command(session_id='other'),'test')
    with pytest.raises(ValueError):transaction_intent_stamp(command(id='种子'),'test')
