import pytest
from d1max_pct_scan.component_health import FunctionalHealth, SupervisorHealth,required_components,advisory_components


def test_local_executor_health_matches_the_only_running_execution_owner():
    assert required_components(motion_pipeline=True,local_executor=True)==('global','reference','tracker')
    assert required_components(motion_pipeline=False,local_executor=True)==('global','reference','tracker')


def test_global_route_scope_observes_only_running_global_producer_not_dormant_local_modules():
    required=required_components(motion_pipeline=False,local_executor=False,planning_scope='global_route')
    assert required==('global',)
    assert advisory_components(planning_scope='global_route')==()
    health=FunctionalHealth('s',required,advisory=(),started=0.)
    assert health.observe('global',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    value=health.snapshot(wall_now=100.,monotonic=0.)
    assert value['ready'] and value['advisory_ready'] and not value['fatal']
    assert [row['name'] for row in value['components']]==['global']
    assert not value['motion_authorized'] and not value['sensor_validity_authorized']
    # A global producer fault is still real liveness failure, not bypassed by scope.
    assert not health.snapshot(wall_now=102.,monotonic=2.)['ready']


@pytest.mark.parametrize('motion,local',[(True,False),(False,True),(True,True)])
def test_global_scope_cannot_claim_control_component_readiness(motion,local):
    with pytest.raises(ValueError,match='global_route_health_cannot_include_motion_pipeline'):
        required_components(motion_pipeline=motion,local_executor=local,planning_scope='global_route')


def test_planner_only_does_not_wait_for_absent_control_components():
    components=required_components(motion_pipeline=False)
    assert components==('global','reference')
    health=FunctionalHealth('s',components,advisory=advisory_components(),started=0.)
    for component in components:
        assert health.observe(component,dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    value=health.snapshot(wall_now=100.,monotonic=0.)
    assert value['ready'] and not value['advisory_ready']
    assert health.components==('global','reference','perception','map')
    assert all(row['name'] not in ('tracker','safety') for row in value['components'])
    assert required_components(motion_pipeline=True)==components+('tracker','safety')


def test_missing_and_stalled_json_relays_are_advisory_not_bt_vetoes():
    required=required_components(motion_pipeline=True)
    health=FunctionalHealth('s',required,advisory=advisory_components(),started=0.)
    for component in health.components:
        assert health.observe(component,dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    for t in range(1,32):
        for component in required:
            assert health.observe(component,dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
        value=health.snapshot(wall_now=100.+t,monotonic=t)
        assert value['ready']
    assert value['ready'] and not value['fatal'] and value['reason']==''
    assert not value['advisory_ready']
    assert value['advisory_reason']=='component_executor_stalled:map,perception'
    assert not value['motion_authorized'] and not value['sensor_validity_authorized']
    assert {row['name']:row['required'] for row in value['components']}=={
        'global':True,'reference':True,'tracker':True,'safety':True,'perception':False,'map':False}


def test_never_seen_advisory_relays_cannot_latch_startup_timeout():
    required=required_components(motion_pipeline=True)
    health=FunctionalHealth('s',required,advisory=advisory_components(),started=0.)
    for component in required:
        assert health.observe(component,dict(session_id='s',stamp=131.),wall_now=131.,monotonic=31.)
    value=health.snapshot(wall_now=131.,monotonic=31.)
    assert value['ready'] and not value['fatal'] and value['reason']==''
    assert value['advisory_reason']=='waiting_components:perception,map'


@pytest.mark.parametrize('dead',required_components(motion_pipeline=True))
def test_required_producer_stall_still_blocks_with_fresh_advisories(dead):
    required=required_components(motion_pipeline=True)
    health=FunctionalHealth('s',required,advisory=advisory_components(),started=0.)
    for component in health.components:
        assert health.observe(component,dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    for t in (1.,2.):
        for component in health.components:
            if component!=dead:
                assert health.observe(component,dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
    value=health.snapshot(wall_now=102.,monotonic=2.)
    assert not value['ready'] and not value['fatal']
    assert value['reason']=='component_executor_stalled:'+dead
    assert value['advisory_ready']


@pytest.mark.parametrize('missing',required_components(motion_pipeline=True))
def test_never_initialized_required_core_still_latches_startup_timeout(missing):
    required=required_components(motion_pipeline=True)
    health=FunctionalHealth('s',required,advisory=advisory_components(),started=0.)
    for component in health.components:
        if component!=missing:
            assert health.observe(component,dict(session_id='s',stamp=131.),wall_now=131.,monotonic=31.)
    value=health.snapshot(wall_now=131.,monotonic=31.)
    assert not value['ready'] and value['fatal'] and value['advisory_ready']
    assert value['reason']=='component_startup_timeout:'+missing


def test_replayed_advisory_report_cannot_renew_its_diagnostic_heartbeat():
    health=FunctionalHealth('s',('reference',),advisory=advisory_components(),started=0.)
    value=dict(session_id='s',stamp=100.)
    assert health.observe('perception',value,wall_now=100.,monotonic=0.)
    assert not health.observe('perception',value,wall_now=101.,monotonic=1.)
    assert not health.observe('perception',dict(session_id='s',stamp=99.9),wall_now=101.,monotonic=1.)
    assert health.observe('reference',dict(session_id='s',stamp=102.),wall_now=102.,monotonic=2.)
    report=health.snapshot(wall_now=102.,monotonic=2.)
    assert report['ready'] and not report['advisory_ready']
    assert report['advisory_reason']=='component_executor_stalled:perception'
    assert health.last['perception']==(100.,0.)


def test_required_and_advisory_membership_cannot_overlap():
    with pytest.raises(ValueError,match='invalid_component_health_contract'):
        FunctionalHealth('s',('reference',),advisory=('reference',),started=0.)


def test_waiting_input_is_alive_but_never_motion_permission():
    health=FunctionalHealth('s',('map','tracker'),started=10.)
    assert not health.snapshot(wall_now=100.,monotonic=10.)['ready']
    for component in ('map','tracker'):
        assert health.observe(component,dict(session_id='s',received_at_unix=100.,ready=False),wall_now=100.,monotonic=10.)
    value=health.snapshot(wall_now=100.,monotonic=10.)
    assert value['ready'] and not value['motion_authorized'] and not value['sensor_validity_authorized']


def test_repeated_or_queued_health_never_renews_stalled_executor():
    health=FunctionalHealth('s',('map',),started=0.)
    value=dict(session_id='s',stamp=100.)
    assert health.observe('map',value,wall_now=100.,monotonic=0.)
    assert not health.observe('map',value,wall_now=101.,monotonic=1.)
    fault=health.snapshot(wall_now=102.,monotonic=2.)
    assert not fault['ready'] and not fault['fatal'] and 'executor_stalled' in fault['reason']
    assert health.observe('map',dict(session_id='s',stamp=102.),wall_now=102.,monotonic=2.)
    recovering=health.snapshot(wall_now=102.,monotonic=2.)
    assert recovering['ready'] and not recovering['fatal'] and recovering['reason']==''
    assert recovering['components'][0]['recovering']
    # Timer ticks do not create producer samples or stretch a source window.
    assert health.snapshot(wall_now=102.1,monotonic=2.1)['components'][0]['recovery_new_samples']==1
    assert not health.observe('map',dict(session_id='s',stamp=102.),wall_now=102.1,monotonic=2.1)
    assert health.observe('map',dict(session_id='s',stamp=102.3),wall_now=102.3,monotonic=2.3)
    assert health.snapshot(wall_now=102.3,monotonic=2.3)['components'][0]['recovering']
    assert health.snapshot(wall_now=102.65,monotonic=2.65)['components'][0]['recovering']
    assert health.observe('map',dict(session_id='s',stamp=102.7),wall_now=102.7,monotonic=2.7)
    recovered=health.snapshot(wall_now=102.7,monotonic=2.7)
    assert recovered['ready'] and not recovered['fatal'] and recovered['reason']==''
    assert not recovered['motion_authorized'] and not recovered['sensor_validity_authorized']


def test_wrong_session_future_and_old_reports_cannot_establish_readiness():
    health=FunctionalHealth('s',('map',),started=0.)
    for value in (dict(session_id='old',stamp=100.),dict(session_id='s',stamp=105.),dict(session_id='s',stamp=90.)):
        assert not health.observe('map',value,wall_now=100.,monotonic=0.)
    assert health.snapshot(wall_now=131.,monotonic=31.)['fatal']


def test_functional_liveness_clock_is_independent_of_recorded_sensor_ros_time():
    health=FunctionalHealth('s',('map',),started=0.)
    value=dict(session_id='s',received_at_unix=10.,callback_wall_time=100.)
    assert health.observe('map',value,wall_now=100.,monotonic=.1)
    assert health.snapshot(wall_now=100.,monotonic=.1)['ready']
    assert not health.snapshot(wall_now=100.,monotonic=.1)['motion_authorized']


def test_dead_monitor_file_does_not_keep_supervisor_alive():
    supervisor=SupervisorHealth('s',started=0.)
    value=dict(schema=1,session_id='s',sequence=1,ready=True,fatal=False)
    assert supervisor.check(value,monotonic=1.)==''
    assert supervisor.check(value,monotonic=2.)==''
    assert supervisor.check(value,monotonic=3.)=='component_health_monitor_stalled'


def test_component_failure_propagates_without_restart_or_reauthorization():
    supervisor=SupervisorHealth('s',started=0.)
    assert supervisor.check(dict(schema=1,session_id='s',sequence=2,fatal=True,reason='map_hung'),monotonic=1.)=='map_hung'


def test_every_stalled_producer_requires_its_own_new_reports():
    health=FunctionalHealth('s',('map','tracker'),started=0.)
    for component in ('map','tracker'):
        assert health.observe(component,dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    assert not health.snapshot(wall_now=102.,monotonic=2.)['ready']
    for t in (2.,2.3,2.7):
        assert health.observe('map',dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
    value=health.snapshot(wall_now=102.7,monotonic=2.7)
    assert not value['ready'] and not value['fatal']
    assert value['reason']=='component_executor_stalled:tracker'
    for t in (2.7,3.,3.4):
        assert health.observe('tracker',dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
    assert health.snapshot(wall_now=103.4,monotonic=3.4)['ready']


def test_recovery_needs_both_distinct_samples_and_elapsed_producer_reports():
    for samples in ((2.,2.1,2.2),(2.,2.7)):
        health=FunctionalHealth('s',('map',),started=0.)
        assert health.observe('map',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
        for t in samples:
            assert health.observe('map',dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
        assert health.snapshot(wall_now=102.8,monotonic=2.8)['components'][0]['recovering']
        assert health.observe('map',dict(session_id='s',stamp=102.9),wall_now=102.9,monotonic=2.9)
        assert health.snapshot(wall_now=102.9,monotonic=2.9)['ready']


def test_gap_during_recovery_resets_only_that_producer_window():
    health=FunctionalHealth('s',('map',),started=0.)
    assert health.observe('map',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    for t in (2.,2.3):
        assert health.observe('map',dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
    # No snapshot needed to detect the second gap.
    assert health.observe('map',dict(session_id='s',stamp=104.),wall_now=104.,monotonic=4.)
    value=health.snapshot(wall_now=104.,monotonic=4.)
    assert value['ready'] and value['components'][0]['recovery_new_samples']==1
    for t in (4.3,4.7):
        assert health.observe('map',dict(session_id='s',stamp=100.+t),wall_now=100.+t,monotonic=t)
    assert health.snapshot(wall_now=104.7,monotonic=4.7)['ready']


def test_initialized_producer_long_gap_does_not_own_bt_terminal_budget():
    health=FunctionalHealth('s',('map',),started=0.)
    assert health.observe('map',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    value=health.snapshot(wall_now=160.,monotonic=60.)
    assert not value['ready'] and not value['fatal']
    assert value['reason']=='component_executor_stalled:map'


def test_never_initialized_producer_still_consumes_bounded_startup():
    health=FunctionalHealth('s',('map',),started=0.,startup_s=60.)
    assert not health.snapshot(wall_now=160.,monotonic=60.)['fatal']
    value=health.snapshot(wall_now=160.1,monotonic=60.1)
    assert value['fatal'] and value['reason']=='component_startup_timeout:map'
    assert health.observe('map',dict(session_id='s',stamp=160.2),wall_now=160.2,monotonic=60.2)
    assert health.snapshot(wall_now=160.2,monotonic=60.2)['fatal']


def test_supervisor_keeps_recoverable_producer_gap_separate_from_dead_monitor():
    health=FunctionalHealth('s',('map',),started=0.)
    supervisor=SupervisorHealth('s',started=0.)
    assert health.observe('map',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    for t in (0.,1.,2.,3.,4.):
        value=health.snapshot(wall_now=100.+t,monotonic=t)
        assert supervisor.check(value,monotonic=t)==''
    assert supervisor.check(value,monotonic=6.)=='component_health_monitor_stalled'


def test_invalid_clock_and_monotonic_reordering_do_not_count_as_recovery():
    health=FunctionalHealth('s',('map',),started=0.)
    assert health.observe('map',dict(session_id='s',stamp=100.),wall_now=100.,monotonic=1.)
    for stamp,wall,mono in ((101.,101.,.5),(101.,float('nan'),2.),(101.,101.,float('nan'))):
        assert not health.observe('map',dict(session_id='s',stamp=stamp),wall_now=wall,monotonic=mono)
    assert health.last['map']==(100.,1.)


def test_first_genuine_recovery_report_can_start_bt_window_without_authorizing_motion():
    health=FunctionalHealth('s',('global','reference','tracker'),started=0.)
    for name in health.components:
        assert health.observe(name,dict(session_id='s',stamp=100.),wall_now=100.,monotonic=0.)
    assert not health.snapshot(wall_now=102.,monotonic=2.)['ready']
    for name in health.components:
        assert health.observe(name,dict(session_id='s',stamp=102.),wall_now=102.,monotonic=2.)
    recovered=health.snapshot(wall_now=102.,monotonic=2.)
    assert recovered['ready'] and all(c['recovering'] for c in recovered['components'])
    assert not recovered['motion_authorized'] and not recovered['sensor_validity_authorized']
    # A fresh monitor report cannot extend missing producer data indefinitely.
    expired=health.snapshot(wall_now=103.6,monotonic=3.6)
    assert not expired['ready'] and not expired['fatal']
