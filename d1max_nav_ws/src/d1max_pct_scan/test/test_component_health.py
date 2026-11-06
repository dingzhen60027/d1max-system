from d1max_pct_scan.component_health import FunctionalHealth, SupervisorHealth


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
    assert not recovering['ready'] and not recovering['fatal']
    assert recovering['reason']=='component_recovering:map'
    # Timer ticks do not create producer samples or stretch a source window.
    assert not health.snapshot(wall_now=102.1,monotonic=2.1)['ready']
    assert not health.observe('map',dict(session_id='s',stamp=102.),wall_now=102.1,monotonic=2.1)
    assert health.observe('map',dict(session_id='s',stamp=102.3),wall_now=102.3,monotonic=2.3)
    assert not health.snapshot(wall_now=102.3,monotonic=2.3)['ready']
    assert not health.snapshot(wall_now=102.65,monotonic=2.65)['ready']
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
        assert not health.snapshot(wall_now=102.8,monotonic=2.8)['ready']
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
    assert not value['ready'] and value['components'][0]['recovery_new_samples']==1
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
