"""ROS-free tests of report auditing, not navigation-component acceptance."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest


SPEC = importlib.util.spec_from_file_location('handoff_graph_audit',
    Path(__file__).with_name('audit_execution_handoff_graph.py'))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def ns(elapsed):
    return 100_000_000_000 + round(elapsed * 1_000_000_000)


def version(generation=1):
    return dict(schema_version=3, session_id='graph-session', task_id='task',
        route_id='route', route_hash='a'*64, map_version_id='source-map',
        localization_epoch=1, localization_seed_id='seed',
        reference_generation=generation, segment_id='floor-1', anchor_id='anchor',
        anchor_revision=generation, context_sequence=generation, map_geometry_revision=1)


def writer():
    return dict(execution_id='authorized-execution', control_epoch=4,
        sdk_session='sdk-session', sdk_arm_generation=2, transport_mode='isolated_mock')


def permit(generation, sequence, start, end):
    return dict(version=version(generation), trajectory_id=generation*11,
        sequence=sequence, permit_sequence=sequence, validation_sequence=sequence,
        source_stamp_ns=ns(start), valid_until_ns=ns(end),
        geometry_committed=True, allowed=True, revoked=False, phase='tracking', **writer())


def event(key, elapsed, **payload):
    return dict(key=key, elapsed_s=elapsed, **payload)


def sample_report():
    incumbent = permit(1, 2, .29, .54)
    candidate = permit(2, 3, .3, .55)
    candidate['geometry_committed'] = False
    grant = event('handoff_grant', .3, handoff_id='handoff-1', sequence=1,
        expected_commit_sequence=1, source_stamp_ns=ns(.3), valid_until_ns=ns(.55),
        transition_deadline_ns=ns(.55), retain_incumbent_until_ns=ns(.54),
        candidate=candidate, incumbent=incumbent)
    prepared = event('prepared_demand', .33, version=version(2), trajectory_id=22,
        handoff_id='handoff-1', grant_sequence=1, expected_commit_sequence=1,
        demand_sequence=20, permit_sequence=3, motion_validation_sequence=41,
        demand_source_stamp_ns=ns(.325), body_source_stamp_ns=ns(.32),
        entry_source_stamp_ns=ns(.32), valid_until_ns=ns(.425),
        entry_admission_sequence=5, vx=.1, wz=0., **writer())
    safe = dict(prepared, key='safe_prepared_demand', elapsed_s=.34)
    proof = event('motion_validation', .345, version=version(2), trajectory_id=22,
        handoff_id='handoff-1', sequence=41, demand_sequence=20, valid=True,
        vx=.1, wz=0.,
        permit_sequence=3, entry_admission_sequence=5,
        demand_source_stamp_ns=ns(.325), demand_body_source_stamp_ns=ns(.32),
        demand_valid_until_ns=ns(.425), check_begin_ns=ns(.335), check_end_ns=ns(.34),
        valid_until_ns=ns(.4), map_snapshot_revision=5, body_source_stamp_ns=ns(.33),
        front_ray_source_stamp_ns=ns(.25), rear_ray_source_stamp_ns=ns(.25), **writer())
    initial = event('commit_ack', .1, handoff_id='', grant_sequence=0,
        previous_commit_sequence=0, commit_sequence=1,
        incumbent_version=version(), candidate_version=version(),
        incumbent_trajectory_id=0, candidate_trajectory_id=11,
        permit_sequence=1, demand_sequence=10, motion_validation_sequence=31,
        entry_admission_sequence=0, applied_at_ns=ns(.09),
        demand_source_stamp_ns=ns(.08), demand_body_source_stamp_ns=ns(.075),
        entry_source_stamp_ns=ns(.075), body_source_stamp_ns=ns(.085),
        valid_until_ns=ns(.18), applied=True, write_submitted=True,
        write_acknowledged=False, sequence=1, **writer())
    ack = event('commit_ack', .36, handoff_id='handoff-1', grant_sequence=1,
        vx=.1, wz=0.,
        previous_commit_sequence=1, commit_sequence=2,
        incumbent_version=version(), candidate_version=version(2),
        incumbent_trajectory_id=11, candidate_trajectory_id=22,
        permit_sequence=3, demand_sequence=20, motion_validation_sequence=41,
        entry_admission_sequence=5, applied_at_ns=ns(.35),
        demand_source_stamp_ns=ns(.325), demand_body_source_stamp_ns=ns(.32),
        entry_source_stamp_ns=ns(.32), body_source_stamp_ns=ns(.345),
        valid_until_ns=ns(.4), applied=True, write_submitted=True,
        write_acknowledged=False, sequence=2, **writer())

    def motion(elapsed, generation, speed):
        return event('motion', elapsed, version=version(generation),
            trajectory_id=generation*11, x=speed, yaw=0.,
            observer_receipt_ns=ns(elapsed), source_stamp_ns=ns(elapsed-.01), **writer())

    def sdk(elapsed, generation):
        return event('sdk', elapsed, version=version(generation),
            applied_trajectory_id=generation*11, writer_commit_sequence=generation,
            phase='ready', source_stamp_ns=ns(elapsed), **writer())

    events = [event('bt', .001, phase='waiting_initialization'),
        event('permit', .05, **permit(1, 1, .04, .29)), initial,
        motion(.11, 1, .1), sdk(.12, 1), event('bt', .2, phase='tracking'),
        event('permit', .29, **incumbent), grant, prepared, safe, proof,
        # Actual command may arrive before ACK on a different ROS topic.
        motion(.355, 2, .1), ack, sdk(.37, 2),
        event('permit', .4, **permit(2, 4, .385, .635)),
        motion(.5, 2, 0.), motion(.6, 2, .1), motion(.8, 2, 0.)]
    sources = [dict(elapsed_s=i*.02, source_ns=ns(i*.02)) for i in range(46)]
    return dict(case='normal', elapsed_s=.9, events=events,
        route_hashes=['a'*64], route_display=[dict(points=21, geometry_sha256='b'*64)],
        late_route_display=dict(points=21, geometry_sha256='b'*64),
        state_sources=dict(local=deepcopy(sources), global_=deepcopy(sources),
                          **{'global':deepcopy(sources)}))


def result(report):
    return MODULE.audit_report(report)


def check(report, name):
    return next(c for c in result(report)['checks'] if c['name'] == name)


def one(report, key):
    return next(e for e in report['events'] if e['key'] == key and
        (key != 'commit_ack' or e['handoff_id']))


def reentry_report():
    report=sample_report()
    # Shift this short protocol fixture so an actual post-zero time window
    # can exist; all original source/expiry timestamps move together.
    def shift(value):
        if isinstance(value,dict):
            for key,item in value.items():
                if key.endswith('_ns') and isinstance(item,int) and item>0:value[key]=item+2_000_000_000
                elif key=='elapsed_s':value[key]=item+2.
                else:shift(item)
        elif isinstance(value,list):
            for item in value:shift(item)
    shift(report);report['elapsed_s']+=2.
    # The graph observer was running before the post-zero protocol window;
    # retain that fact instead of fabricating a report truncated at 2 seconds.
    report['events'].append(event('bt', .001, phase='waiting_initialization'))
    g=one(report,'handoff_grant');a=g['incumbent']
    a['allowed']=False;a['phase']='holding'
    g['schema_version']=2;g['transition_mode']=1
    g['retain_incumbent_until_ns']=g['source_stamp_ns']
    e=dict(schema_version=1,sequence=1,version=version(),writer_commit_sequence=1,
        applied_trajectory_id=11,zero_write_sequence=9,zero_ack_at_ns=ns(1.5),
        mc_raw_stamp_ns=123000,mc_clock_epoch='fixture-original-clock',
        time_basis='isolated_simulated_source_clock',source_stamp_ns=ns(2.29),
        received_stamp_ns=ns(2.29),capture_lower_bound_ns=ns(2.28),
        capture_upper_bound_ns=ns(2.29),valid_until_ns=ns(2.54),
        stationary_samples=35,stationary_duration_sec=.7,measured_linear_mps=0.,
        measured_angular_radps=0.,mc_capture_delay_bound_sec=.01,usable=True,
        nonzero_blocked=True,physical_acceptance_verified=False,**writer())
    g['valid_until_ns']=g['transition_deadline_ns']=ns(2.54)
    g['candidate']['valid_until_ns']=ns(2.54)
    g['stationary_evidence']=deepcopy(e)
    report['events'].append(event('stationary_evidence',2.29,**e))
    report['events'].sort(key=lambda x:x['elapsed_s'])
    return report


def test_exact_stationary_reentry_pins_independent_original_sdk_witness():
    assert result(reentry_report())['passed']


def installation_report():
    report=sample_report();report['handoff_contract']='writer_applied_cas_v3'
    initial=next(e for e in report['events'] if e['key']=='commit_ack' and not e['handoff_id'])
    initial.update(vx=.1,wz=0.)
    p=next(e for e in report['events'] if e['key']=='permit')
    p['frame_id']='d1max_loc_odom'
    report['events'].extend([
        event('demand', .082, version=version(),trajectory_id=11,sequence=10,
              permit_sequence=1,motion_validation_sequence=0,source_stamp_ns=ns(.08),
              body_source_stamp_ns=ns(.075),valid_until_ns=ns(.18),vx=.1,wz=0.,**writer()),
        event('safe_demand', .086, version=version(),trajectory_id=11,sequence=10,
              permit_sequence=1,motion_validation_sequence=31,source_stamp_ns=ns(.08),
              body_source_stamp_ns=ns(.075),valid_until_ns=ns(.18),vx=.1,wz=0.,**writer()),
        event('motion_validation', .085, version=version(),trajectory_id=11,sequence=31,
              handoff_id='',demand_sequence=10,permit_sequence=1,entry_admission_sequence=0,
              valid=True,map_snapshot_revision=5,demand_source_stamp_ns=ns(.08),
              demand_body_source_stamp_ns=ns(.075),demand_valid_until_ns=ns(.18),
              check_begin_ns=ns(.081),check_end_ns=ns(.085),body_source_stamp_ns=ns(.085),
              front_ray_source_stamp_ns=ns(.06),rear_ray_source_stamp_ns=ns(.06),
              valid_until_ns=ns(.18),vx=.1,wz=0.,**writer()),
        event('validation', .04, version=version(),trajectory_id=11,sequence=1,
              valid=True,check_end_ns=ns(.035),valid_until_ns=ns(.18)),
        event('trajectory_admission', .047, version=version(),trajectory_id=11,
              sequence=7,validation_sequence=1,accepted=True,
              checked_at_ns=ns(.045),valid_until_ns=ns(.15)),
        event('tracker_geometry_receipt', .065, version=version(),trajectory_id=11,
              schema_version=1,sequence=1,installation_sequence=1,installed=True,
              permit_sequence=1,admission_sequence=7,validation_sequence=1,
              installed_at_ns=ns(.05),body_source_stamp_ns=ns(.06),
              frame_id='d1max_loc_odom',transport_mode='isolated_mock')])
    report['events'].sort(key=lambda e:e['elapsed_s'])
    return report


def test_tracker_installation_is_a_separate_original_typed_fact():
    assert result(installation_report())['passed']


@pytest.mark.parametrize('change',['no_receipt','wrong_proof','expired_install','fake_body','renewed_install'])
def test_preparation_or_heartbeat_cannot_fake_tracker_installation(change):
    report=installation_report();r=one(report,'tracker_geometry_receipt')
    if change=='no_receipt':report['events'].remove(r)
    elif change=='wrong_proof':r['validation_sequence']+=1
    elif change=='expired_install':r['installed_at_ns']=ns(.16)
    elif change=='fake_body':r['body_source_stamp_ns']+=1
    else:
        duplicate=dict(r,elapsed_s=.07,sequence=2,installed_at_ns=ns(.051))
        report['events'].append(duplicate);report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report,'typed_tracker_installation_facts')['status']!='PASS'


@pytest.mark.parametrize('change',['missing_sdk_stream','renewed_witness','old_can_move','old_motion_retained',
    'too_short','two_samples','moving','before_ack','expired','wrong_commit','wrong_geometry','live_unaccepted'])
def test_dangerous_stationary_reentry_is_not_mistaken_for_continuous_replace(change):
    report=reentry_report();g=one(report,'handoff_grant');e=g['stationary_evidence']
    if change=='missing_sdk_stream':report['events']=[x for x in report['events'] if x['key']!='stationary_evidence']
    elif change=='renewed_witness':e['valid_until_ns']+=1
    elif change=='old_can_move':g['incumbent']['allowed']=True
    elif change=='old_motion_retained':g['retain_incumbent_until_ns']+=1
    elif change=='too_short':e['stationary_duration_sec']=.59
    elif change=='two_samples':e['stationary_samples']=2
    elif change=='moving':e['measured_linear_mps']=.031
    elif change=='before_ack':e['capture_lower_bound_ns']=e['zero_ack_at_ns']
    elif change=='expired':e['valid_until_ns']=g['source_stamp_ns']-1
    elif change=='wrong_commit':e['writer_commit_sequence']+=1
    elif change=='wrong_geometry':e['applied_trajectory_id']+=1
    else:e['transport_mode']='live'
    # Except renewal/missing-source, retain an independently observed witness
    # with the same dangerous contents so shape matching cannot hide the bug.
    if change not in ('missing_sdk_stream','renewed_witness'):
        independent=one(report,'stationary_evidence')
        independent.update(deepcopy(e))
    assert not result(report)['passed']


def test_complete_observation_passes_protocol_only_and_measures_short_zero():
    audited = result(sample_report())
    assert audited['passed'], audited['checks']
    assert not any(audited[k] for k in
        ('physical_acceptance', 'realtime_acceptance', 'continuity_acceptance'))
    assert audited['diagnostics']['zeros_between_nonzero_samples']['count'] == 1
    assert audited['diagnostics']['zeros_between_nonzero_samples']['total_s'] == pytest.approx(.1)


def test_cross_topic_ack_delivery_order_is_not_writer_application_order():
    report = sample_report()
    assert one(report, 'commit_ack')['elapsed_s'] > one(report, 'motion_validation')['elapsed_s']
    assert check(report, 'active_geometry_only_after_applied_ack')['status'] == 'PASS'


@pytest.mark.parametrize('field', ['valid_until_ns', 'applied_at_ns', 'demand_source_stamp_ns',
    'demand_body_source_stamp_ns', 'entry_source_stamp_ns', 'body_source_stamp_ns'])
def test_duplicate_ack_cannot_renew_or_change_original_facts(field):
    report = sample_report()
    duplicate = deepcopy(one(report, 'commit_ack'))
    duplicate.update(sequence=3, elapsed_s=.45, write_acknowledged=True)
    duplicate[field] += 1
    report['events'].append(duplicate)
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'FAIL'


def test_later_callback_duplicate_is_idempotent_not_a_new_lease():
    report = sample_report()
    duplicate = dict(one(report, 'commit_ack'), sequence=3, elapsed_s=.45,
                     write_acknowledged=True)
    report['events'].append(duplicate)
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert result(report)['passed']


@pytest.mark.parametrize('field', ['vx', 'demand_source_stamp_ns', 'body_source_stamp_ns', 'valid_until_ns'])
def test_safe_prepared_cannot_rewrite_original_command(field):
    report = sample_report()
    one(report, 'safe_prepared_demand')[field] += 1
    assert check(report, 'immutable_prepared_command_payload')['status'] == 'FAIL'


def test_grant_candidate_does_not_become_ordinary_active_permission():
    report = sample_report()
    report['events'].append(event('permit', .32,
        **dict(one(report, 'handoff_grant')['candidate'], geometry_committed=True)))
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'active_geometry_only_after_applied_ack')['status'] == 'FAIL'


def test_grant_cannot_extend_incumbent_original_lease():
    report = sample_report()
    one(report, 'handoff_grant')['retain_incumbent_until_ns'] += 1
    assert check(report, 'fixed_conditional_grants')['status'] == 'FAIL'


def test_missing_native_sweep_deadline_is_unverified_not_pass():
    report = sample_report()
    del one(report, 'motion_validation')['demand_valid_until_ns']
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'UNVERIFIED'
    assert not result(report)['passed']


def test_missing_state_sources_is_unverified_not_fake_heartbeat_frequency():
    report = sample_report()
    del report['state_sources']['local']
    assert check(report, 'original_state_sources')['status'] == 'UNVERIFIED'
    assert not result(report)['passed']


def test_changed_global_route_and_late_display_fail():
    report = sample_report()
    report['route_hashes'].append('c'*64)
    assert check(report, 'immutable_global_route')['status'] == 'FAIL'
    report = sample_report()
    report['late_route_display']['geometry_sha256'] = 'c'*64
    assert check(report, 'immutable_global_route')['status'] == 'FAIL'


def test_initial_empty_display_is_not_a_global_route_change():
    report = sample_report()
    report['route_display'].insert(0, dict(points=0, geometry_sha256='empty-display'))
    assert check(report, 'immutable_global_route')['status'] == 'PASS'
    report['route_display'].append(dict(points=0, geometry_sha256='empty-display'))
    assert check(report, 'immutable_global_route')['status'] == 'FAIL'


def test_terminal_failure_long_zero_is_not_hidden_by_last_nonzero_sample():
    report = sample_report()
    report['elapsed_s'] = 35.
    report['events'].append(event('bt', 34., phase='failed'))
    audited = result(report)
    assert audited['diagnostics']['actual_zero_intervals']['max_s'] == pytest.approx(33.2)
    assert audited['diagnostics']['actual_zero_intervals']['window_end_reason'] == 'failed'
    assert audited['diagnostics']['zeros_between_nonzero_samples']['max_s'] == pytest.approx(.1)


def test_actual_receipt_before_actual_writer_selection_fails():
    report = sample_report()
    actual = next(e for e in report['events'] if e['key'] == 'motion' and e['trajectory_id'] == 22)
    actual['observer_receipt_ns'] = ns(.349)
    assert check(report, 'active_geometry_only_after_applied_ack')['status'] == 'FAIL'


def test_same_geometry_does_not_bypass_wrong_writer_or_sdk_commit_checks():
    report = sample_report()
    report['events'].append(event('sdk', .7, version=version(2), applied_trajectory_id=22,
        writer_commit_sequence=3, phase='ready', source_stamp_ns=ns(.7), **writer()))
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'active_geometry_only_after_applied_ack')['status'] == 'FAIL'
    report = sample_report()
    actual = next(e for e in report['events'] if e['key']=='motion' and e['elapsed_s']==.6)
    actual['execution_id'] = 'foreign-execution'
    assert check(report, 'active_geometry_only_after_applied_ack')['status'] == 'FAIL'


def test_zero_prepared_can_commit_only_with_bound_full_candidate_curve_proof():
    report = sample_report()
    for key in ('prepared_demand', 'safe_prepared_demand'):
        one(report,key).update(vx=0.,wz=0.,motion_validation_sequence=0)
    one(report,'commit_ack')['motion_validation_sequence'] = 0
    report['events'].append(event('validation', .345, version=version(2), trajectory_id=22,
        sequence=3, valid=True, whole_curve=True, remaining_curve=False,
        checked_from_time=0.,checked_to_time=1.,curve_duration=1.,
        support_reference_id='support', support_hash='c'*64, map_snapshot_revision=5,
        check_begin_ns=ns(.335),check_end_ns=ns(.34),valid_until_ns=ns(.4),
        body_source_stamp_ns=ns(.33),front_ray_source_stamp_ns=ns(.25),rear_ray_source_stamp_ns=ns(.25)))
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'PASS'
    one(report,'validation')['whole_curve'] = False
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'FAIL'


def test_nonzero_prepared_cannot_borrow_zero_motion_proof_semantics():
    report = sample_report()
    for key in ('prepared_demand', 'safe_prepared_demand', 'commit_ack'):
        one(report,key)['motion_validation_sequence'] = 0
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'FAIL'


def test_unobserved_sdk_phase_does_not_claim_absence_of_hold():
    report = sample_report()
    for sdk in (e for e in report['events'] if e['key']=='sdk'):
        del sdk['phase']
    audited = result(report)
    assert all(z['coincides_with']['sdk_hold'] is None for z in
               audited['diagnostics']['actual_zero_intervals']['intervals'])


def test_native_proof_cannot_renew_original_demand_deadline():
    report = sample_report()
    one(report, 'motion_validation')['demand_valid_until_ns'] += 1
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'FAIL'


def test_source_gap_and_receiver_gap_reported_separately():
    stats = MODULE.source_stats([dict(elapsed_s=0.,source_ns=ns(0.)),
        dict(elapsed_s=.5,source_ns=ns(.02)), dict(elapsed_s=.52,source_ns=ns(.04))])
    assert stats['maximum_source_gap_s'] == pytest.approx(.02)
    assert stats['maximum_receipt_gap_s'] == pytest.approx(.5)


def test_malformed_observation_does_not_crash_or_pass():
    report = sample_report()
    report['events'].append(dict(key='unknown'))
    audited = result(report)
    assert not audited['passed']
    assert any(c['name']=='event_format' and c['status']=='FAIL' for c in audited['checks'])


def test_no_replacement_evidence_is_not_handoff_success():
    report = sample_report()
    report['events'] = [e for e in report['events'] if e['elapsed_s'] < .29]
    assert check(report, 'writer_submission_facts_and_idempotence')['status'] == 'UNVERIFIED'
    assert not result(report)['passed']


def test_local_recovery_uses_distinct_actual_sources_and_complete_stable_window():
    report = sample_report()
    report['case'] = 'local_gap'
    report['state_sources']['local'] = [dict(elapsed_s=t,source_ns=ns(t))
        for t in (0.,.1,.6,.62,.64,.66,.68,.7,.8)]
    report['events'] += [event('bt', .15, phase='paused'), event('bt', .85, phase='tracking')]
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'local_gap_recovery_window')['status'] == 'FAIL'
    # Duplicates cannot fabricate the three-new-source requirement.
    report['state_sources']['local'][3]['source_ns'] = ns(.6)
    assert not result(report)['passed']


def test_global_only_gap_with_task_pause_is_not_hidden_as_continuity():
    report = sample_report()
    report['case'] = 'global_gap'
    report['state_sources']['global'] = [dict(elapsed_s=t,source_ns=ns(t)) for t in (0.,.1,.6,.62,.64)]
    report['events'] += [event('bt', .15, phase='paused'), event('bt', .7, phase='tracking')]
    report['events'].sort(key=lambda e:e['elapsed_s'])
    assert check(report, 'global_gap_does_not_revoke_local_task')['status'] == 'FAIL'


def test_explicit_observer_loss_cannot_claim_complete_evidence():
    report=sample_report()
    report['observation_window']={'event_capacity':20000,'dropped_events':1}
    assert check(report,'observation_coverage')['status']=='UNVERIFIED'
    report['observation_window']['dropped_events']=-1
    assert check(report,'observation_coverage')['status']=='FAIL'


def test_observer_lifecycle_is_not_component_startup_or_execution_evidence():
    report=sample_report()
    report['events'].insert(0,event('observer_started',0.,event_capacity=20000))
    report['observation_window']={'event_capacity':20000,'dropped_events':0}
    assert check(report,'observation_coverage')['status']=='PASS'
    report['events']=[e for e in report['events'] if e['key']=='observer_started']
    assert check(report,'writer_submission_facts_and_idempotence')['status']=='UNVERIFIED'


def test_reported_observer_loss_is_not_a_fabricated_writer_failure_or_pass():
    report=sample_report()
    report['events']=[e for e in report['events']
        if not (e['key']=='commit_ack' and not e['handoff_id'])]
    assert check(report,'writer_commit_chain')['status']=='FAIL'
    report['observation_window']={'event_capacity':60000,'dropped_events':1}
    assert check(report,'writer_commit_chain')['status']=='UNVERIFIED'
    assert check(report,'active_geometry_only_after_applied_ack')['status']=='UNVERIFIED'
    assert not result(report)['passed']


def test_observer_loss_does_not_hide_an_observed_deadline_contradiction():
    report=sample_report()
    report['observation_window']={'event_capacity':60000,'dropped_events':1}
    ack=next(e for e in report['events'] if e['key']=='commit_ack' and e['handoff_id'])
    ack['valid_until_ns']+=1
    assert check(report,'writer_submission_facts_and_idempotence')['status']=='FAIL'
    assert not result(report)['passed']


def test_observer_loss_does_not_hide_a_later_fully_observed_commit_conflict():
    report=sample_report()
    report['events']=[e for e in report['events']
        if not (e['key']=='commit_ack' and not e['handoff_id'])]
    report['observation_window']={'event_capacity':60000,'dropped_events':1}
    sdk=next(e for e in report['events'] if e['key']=='sdk' and e['applied_trajectory_id']==22)
    sdk['writer_commit_sequence']=99
    assert check(report,'active_geometry_only_after_applied_ack')['status']=='FAIL'


def test_observer_loss_does_not_hide_observed_time_regression():
    report=sample_report()
    report['observation_window']={'event_capacity':60000,'dropped_events':1}
    report['events'][0],report['events'][1]=report['events'][1],report['events'][0]
    assert check(report,'observation_coverage')['status']=='FAIL'


@pytest.mark.parametrize('key',['commit_ack','motion_validation'])
def test_conditional_velocity_is_exactly_the_original_prepared_command(key):
    report=installation_report()
    record=next(e for e in report['events'] if e['key']==key and e.get('handoff_id'))
    record['vx']=.9
    assert check(report,'writer_submission_facts_and_idempotence')['status']=='FAIL'
    del record['vx']
    assert check(report,'writer_submission_facts_and_idempotence')['status']=='UNVERIFIED'
