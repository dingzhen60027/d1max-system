from d1max_pct_scan.component_health import FunctionalHealth, SupervisorHealth


def test_waiting_input_is_alive_but_never_motion_permission():
    health=FunctionalHealth('s',('map','tracker'),started=10.)
    assert not health.snapshot(wall_now=100.,monotonic=10.)['ready']
    for component in ('map','tracker'):
        assert health.observe(component,dict(session_id='s',received_at_unix=100.,ready=False),wall_now=100.,monotonic=10.)
    value=health.snapshot(wall_now=100.,monotonic=10.)
    assert value['ready'] and not value['motion_authorized'] and not value['sensor_validity_authorized']


def test_repeated_or_queued_health_never_renews_dead_executor_and_fault_latches():
    health=FunctionalHealth('s',('map',),started=0.)
    value=dict(session_id='s',stamp=100.)
    assert health.observe('map',value,wall_now=100.,monotonic=0.)
    assert not health.observe('map',value,wall_now=101.,monotonic=1.)
    fault=health.snapshot(wall_now=102.,monotonic=2.)
    assert fault['fatal'] and 'executor_stalled' in fault['reason']
    assert health.observe('map',dict(session_id='s',stamp=102.),wall_now=102.,monotonic=2.)
    assert health.snapshot(wall_now=102.,monotonic=2.)['fatal']


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
