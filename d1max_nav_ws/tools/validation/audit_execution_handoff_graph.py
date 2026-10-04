#!/usr/bin/env python3
"""Read-only audit of observed production-component handoffs in graph reports.

This is not a protocol fixture, collision checker, physical acceptance test, or
real-time certification. Missing observations never count as passing evidence.
Command zero durations are observer-receipt sample intervals, not physical
braking times. ACK timestamps retain writer time; receipt time is never used to
renew a source or authority lease.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path


VERSION_FIELDS = (
    'schema_version', 'session_id', 'task_id', 'route_id', 'route_hash',
    'map_version_id', 'localization_epoch', 'localization_seed_id',
    'reference_generation', 'segment_id', 'anchor_id', 'anchor_revision',
    'context_sequence', 'map_geometry_revision',
)
WRITER_FIELDS = ('execution_id', 'control_epoch', 'sdk_session',
                 'sdk_arm_generation', 'transport_mode')
TASK_FIELDS = VERSION_FIELDS[:8]


class MissingEvidence(ValueError):
    pass


def fields(value, names):
    if not isinstance(value, dict):
        raise ValueError('observation payload is not an object')
    absent = [name for name in names if name not in value]
    if absent:
        raise MissingEvidence('missing fields: ' + ', '.join(absent))
    return tuple(value[name] for name in names)


def full_version(value):
    result = fields(value, VERSION_FIELDS)
    if value['schema_version'] != 3:
        raise ValueError('execution version is not schema 3')
    if (any(not isinstance(value[name], str) or not value[name] for name in
            ('session_id', 'task_id', 'route_id', 'map_version_id',
             'localization_seed_id', 'segment_id', 'anchor_id'))
            or not isinstance(value['route_hash'], str)
            or len(value['route_hash']) != 64
            or any(c not in '0123456789abcdef' for c in value['route_hash'])
            or any(not isinstance(value[name], int) or value[name] <= 0 for name in
                   ('localization_epoch', 'reference_generation',
                    'anchor_revision', 'context_sequence'))):
        raise ValueError('incomplete execution identity')
    return result


def geometry(value, version_name='version', trajectory_name='trajectory_id'):
    fields(value, (version_name, trajectory_name))
    return full_version(value[version_name]), value[trajectory_name]


def writer(value):
    return fields(value, WRITER_FIELDS)


def original_demand(value):
    # Safe admission may upgrade only a proof signature, not this payload.
    return (geometry(value), writer(value), fields(value, (
        'handoff_id', 'grant_sequence', 'expected_commit_sequence',
        'demand_sequence', 'permit_sequence', 'demand_source_stamp_ns',
        'body_source_stamp_ns', 'entry_source_stamp_ns', 'valid_until_ns',
        'entry_admission_sequence', 'vx', 'wz')))


def nonzero(event):
    linear = event.get('x', event.get('vx'))
    angular = event.get('yaw', event.get('wz'))
    if linear is None or angular is None:
        raise MissingEvidence('missing actual command velocity')
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError('non-finite actual command')
    return abs(linear) + abs(angular) > 1e-9


class Audit:
    def __init__(self):
        self.checks = []

    def record(self, name, status, detail, *, evidence=None):
        value = dict(name=name, status=status, detail=detail)
        if evidence is not None:
            value['evidence'] = evidence
        self.checks.append(value)

    def verify(self, name, function):
        try:
            evidence = function()
            self.record(name, 'PASS', 'Observed invariant holds.', evidence=evidence)
            return evidence
        except MissingEvidence as exc:
            self.record(name, 'UNVERIFIED', str(exc))
        except (AssertionError, ValueError, TypeError, KeyError) as exc:
            self.record(name, 'FAIL', str(exc) or 'Observed invariant violated.')
        return None


def source_stats(samples, threshold=.1):
    if not samples:
        raise MissingEvidence('no original state-source observations')
    gaps = []
    repeats = reversals = 0
    maximum_source = maximum_receipt = 0.
    previous = None
    for sample in samples:
        fields(sample, ('elapsed_s', 'source_ns'))
        if (not math.isfinite(sample['elapsed_s']) or sample['elapsed_s'] < 0
                or not isinstance(sample['source_ns'], int) or sample['source_ns'] <= 0):
            raise ValueError('invalid original state-source observation')
        if previous:
            source = (sample['source_ns'] - previous['source_ns']) * 1e-9
            receipt = sample['elapsed_s'] - previous['elapsed_s']
            repeats += source == 0
            reversals += source < 0 or receipt < 0
            maximum_source = max(maximum_source, source)
            maximum_receipt = max(maximum_receipt, receipt)
            if source > threshold or receipt > threshold:
                gaps.append(dict(begin_s=previous['elapsed_s'],
                    end_s=sample['elapsed_s'], source_gap_s=source,
                    receipt_gap_s=receipt, before_source_ns=previous['source_ns'],
                    after_source_ns=sample['source_ns']))
        previous = sample
    return dict(samples=len(samples), repeated_sources=repeats,
        reversed_sources_or_receipts=reversals, maximum_source_gap_s=maximum_source,
        maximum_receipt_gap_s=maximum_receipt, gaps=gaps)


def intervals(events, predicate, end):
    result = []
    began = None
    first = None
    for event in events:
        if predicate(event):
            if began is None:
                began, first = event['elapsed_s'], event
        elif began is not None:
            result.append(dict(begin_s=began, end_s=event['elapsed_s'],
                               duration_s=event['elapsed_s']-began,
                               first_event=first))
            began = None
    if began is not None:
        result.append(dict(begin_s=began, end_s=end, duration_s=end-began,
                           first_event=first))
    return result


def overlaps(a, b):
    return a['begin_s'] < b['end_s'] and b['begin_s'] < a['end_s']


def audit_report(report, *, source_gap_s=.1):
    audit = Audit()
    supplied_events = report.get('events', [])
    events = []
    by_key = defaultdict(list)
    for event in supplied_events:
        if not isinstance(event, dict) or 'key' not in event or 'elapsed_s' not in event:
            audit.record('event_format', 'FAIL', 'Malformed observation event.')
            continue
        if not isinstance(event['elapsed_s'], (int, float)) or not math.isfinite(event['elapsed_s']) or event['elapsed_s'] < 0:
            audit.record('event_format', 'FAIL', 'Invalid observer elapsed time.')
            continue
        events.append(event)
        by_key[event['key']].append(event)
    end = report.get('elapsed_s', max((e['elapsed_s'] for e in events), default=0.))
    window = report.get('observation_window')
    known_observer_loss = (isinstance(window, dict)
        and type(window.get('dropped_events')) is int and window['dropped_events'] > 0)

    def coverage():
        if not events:
            raise MissingEvidence('empty graph event stream')
        if any(b['elapsed_s'] < a['elapsed_s'] for a, b in zip(events, events[1:])):
            raise ValueError('observer event time regressed')
        window=report.get('observation_window')
        if window is not None:
            if (not isinstance(window,dict) or type(window.get('dropped_events')) is not int
                    or window['dropped_events']<0):
                raise ValueError('invalid bounded observer loss count')
            if window['dropped_events']:
                raise MissingEvidence('bounded observer dropped original graph events')
        if events[0]['elapsed_s'] > .5:
            raise MissingEvidence('initial graph observations were truncated or absent')
        return dict(events=len(events), first_elapsed_s=events[0]['elapsed_s'],
                    last_elapsed_s=events[-1]['elapsed_s'])
    audit.verify('observation_coverage', coverage)

    def immutable_route():
        hashes = set(report.get('route_hashes', []))
        for event in events:
            for name in ('version', 'candidate_version'):
                if name in event and event[name].get('route_hash'):
                    hashes.add(event[name]['route_hash'])
            for name in ('candidate', 'incumbent'):
                value = event.get(name, {}).get('version', {})
                if value.get('route_hash'):
                    hashes.add(value['route_hash'])
        if not hashes:
            raise MissingEvidence('no committed global route hash')
        assert len(hashes) == 1, 'global route content hash changed: ' + repr(sorted(hashes))
        display = report.get('route_display', [])
        if not display:
            raise MissingEvidence('no fixed global route display observations')
        committed_display = []
        for observation in display:
            count = fields(observation, ('points',))[0]
            if count > 0:
                committed_display.append(observation)
            elif committed_display:
                raise ValueError('committed displayed route was cleared')
        display = committed_display
        if not display:
            raise MissingEvidence('only initial empty route display was observed')
        display_hashes = {fields(e, ('geometry_sha256',))[0] for e in display}
        assert len(display_hashes) == 1, 'displayed global geometry changed'
        if report.get('late_route_display'):
            assert report['late_route_display']['geometry_sha256'] in display_hashes, 'late RViz view differs'
        return dict(route_hash=next(iter(hashes)), display_sha256=next(iter(display_hashes)))
    audit.verify('immutable_global_route', immutable_route)

    grants = {}
    grant_errors = []
    missing_grants = []
    fixed_grant_fields = ('sequence', 'expected_commit_sequence', 'source_stamp_ns',
        'valid_until_ns', 'transition_deadline_ns', 'retain_incumbent_until_ns',
        'candidate', 'incumbent')
    for event in by_key['handoff_grant']:
        try:
            identifier = fields(event, ('handoff_id',))[0]
            fixed = fields(event, fixed_grant_fields)
            a, b = event['incumbent'], event['candidate']
            fields(a, ('allowed', 'geometry_committed', 'revoked', 'phase', 'valid_until_ns'))
            fields(b, ('allowed', 'geometry_committed', 'revoked', 'phase', 'valid_until_ns'))
            old, new = geometry(a), geometry(b)
            assert identifier and event['expected_commit_sequence'] > 0, 'replacement has zero/empty CAS identity'
            assert old != new, 'candidate equals incumbent geometry'
            assert tuple(a['version'][k] for k in TASK_FIELDS) == tuple(b['version'][k] for k in TASK_FIELDS), 'task identity changed'
            assert writer(a) == writer(b), 'writer identity changed'
            assert a['geometry_committed'] and not a['revoked'], 'incumbent lost actual applied identity'
            assert b['allowed'] and not b['geometry_committed'] and not b['revoked'], 'candidate leaked active geometry'
            mode = event.get('transition_mode', 0)  # Historical v1 reports are explicitly mode 0.
            if 'transition_mode' in event:
                assert fields(event, ('schema_version',))[0] == 2, 'wrong typed handoff schema'
            assert mode in (0, 1), 'unknown transition mode'
            if mode == 0:
                assert a['allowed'] and a['phase'] == b['phase'] == 'tracking', 'moving handoff lacks old authorization'
                assert event['retain_incumbent_until_ns'] <= a['valid_until_ns'], 'incumbent original lease extended'
            else:
                assert not a['allowed'] and a['phase'] == 'holding' and b['phase'] == 'tracking', 'reentry granted old motion'
                assert event['retain_incumbent_until_ns'] == event['source_stamp_ns'], 'reentry retained old motion'
                e = fields(event, ('stationary_evidence',))[0]
                keys = ('schema_version', 'sequence', 'writer_commit_sequence', 'applied_trajectory_id',
                    'zero_write_sequence', 'mc_raw_stamp_ns', 'mc_clock_epoch', 'time_basis',
                    'stationary_samples', 'stationary_duration_sec', 'measured_linear_mps',
                    'measured_angular_radps', 'mc_capture_delay_bound_sec', 'usable', 'nonzero_blocked',
                    'physical_acceptance_verified', 'zero_ack_at_ns', 'source_stamp_ns', 'received_stamp_ns',
                    'capture_lower_bound_ns', 'capture_upper_bound_ns', 'valid_until_ns')
                signature = fields(e, keys), full_version(e['version']), writer(e)
                assert any((fields(s, keys), full_version(s['version']), writer(s)) == signature
                    for s in by_key['stationary_evidence'] if s.get('usable')), 'reentry witness not independently SDK-published'
                assert full_version(e['version']) == old[0] and e['applied_trajectory_id'] == old[1], 'stationary witness wrong applied geometry'
                assert writer(e) == writer(a) and e['writer_commit_sequence'] == event['expected_commit_sequence'], 'stationary witness wrong writer commit'
                assert e['schema_version'] == 1 and e['usable'] and e['nonzero_blocked'], 'unusable stopped witness'
                assert e['zero_write_sequence'] > 0 and e['mc_raw_stamp_ns'] > 0 and e['mc_clock_epoch'], 'missing original zero/MC identity'
                assert e['stationary_samples'] >= 3 and e['stationary_duration_sec'] >= .6, 'insufficient stationary sources'
                assert 0 <= e['measured_linear_mps'] <= .03 and 0 <= e['measured_angular_radps'] <= .05, 'robot not stationary'
                assert 0 <= e['mc_capture_delay_bound_sec'] <= .25, 'unbounded capture uncertainty'
                assert 0 < e['zero_ack_at_ns'] < e['capture_lower_bound_ns'] <= e['capture_upper_bound_ns'] <= e['received_stamp_ns'], 'MC is not provably after zero ACK'
                assert e['capture_lower_bound_ns'] - e['zero_ack_at_ns'] >= 600_000_000, 'stationary duration exceeds available post-ACK time'
                assert 0 <= event['source_stamp_ns'] - e['source_stamp_ns'] <= 250_000_000, 'old stationary source pinned'
                assert event['transition_deadline_ns'] <= e['valid_until_ns'] <= e['source_stamp_ns'] + 250_000_000, 'stop witness source renewed'
                assert e['time_basis'] == ('source_delta_host_anchor_approximate' if e['transport_mode'] == 'live' else 'isolated_simulated_source_clock'), 'wrong clock basis'
                assert e['transport_mode'] != 'live' or e['physical_acceptance_verified'], 'unaccepted live stop evidence'
            assert event['source_stamp_ns'] < event['transition_deadline_ns'] <= event['source_stamp_ns']+250_000_000, 'unbounded transition deadline'
            assert event['transition_deadline_ns'] <= event['valid_until_ns'], 'transition outlives grant'
            assert b['valid_until_ns'] <= event['valid_until_ns'], 'candidate outlives grant'
            if identifier in grants:
                assert fixed == fields(grants[identifier], fixed_grant_fields), 'same handoff renewed lease or changed geometry'
                assert event.get('transition_mode',0) == grants[identifier].get('transition_mode',0), 'same handoff changed transition mode'
                assert event.get('stationary_evidence') == grants[identifier].get('stationary_evidence'), 'same handoff renewed stationary evidence'
            else:
                grants[identifier] = event
        except MissingEvidence as exc:
            missing_grants.append(dict(elapsed_s=event['elapsed_s'], reason=str(exc)))
        except (AssertionError, ValueError, TypeError, KeyError) as exc:
            grant_errors.append(dict(elapsed_s=event['elapsed_s'], reason=str(exc)))
    audit.record('fixed_conditional_grants', 'FAIL' if grant_errors else 'UNVERIFIED' if missing_grants or not grants else 'PASS',
        'Conditional grants retain the original two-slot limits.',
        evidence=dict(unique_grants=len(grants), errors=grant_errors[:32], missing=missing_grants[:32]))

    prepared = {}
    missing_prepared = []
    invalid_prepared = []
    for event in by_key['prepared_demand']:
        try:
            prepared[original_demand(event)] = event
        except MissingEvidence as exc:
            missing_prepared.append(str(exc))
        except (ValueError, TypeError, KeyError) as exc:
            invalid_prepared.append(str(exc))

    def prepared_originals():
        if not by_key['safe_prepared_demand']:
            raise MissingEvidence('no independently admitted prepared command')
        if missing_prepared:
            raise MissingEvidence(missing_prepared[0])
        assert not invalid_prepared, 'invalid original prepared observations: '+repr(invalid_prepared[:8])
        matched = 0
        for event in by_key['safe_prepared_demand']:
            key = original_demand(event)
            assert key in prepared, 'safe prepared output changed original command, source or lease'
            g = grants.get(event['handoff_id'])
            if g is None:
                raise MissingEvidence('safe prepared output has no complete observed grant')
            assert event['grant_sequence'] == g['sequence'], 'prepared grant sequence mismatch'
            assert event['expected_commit_sequence'] == g['expected_commit_sequence'], 'prepared CAS mismatch'
            assert geometry(event) == geometry(g['candidate']), 'prepared geometry differs from pin'
            assert writer(event) == writer(g['candidate']), 'prepared writer identity differs'
            assert event['valid_until_ns'] <= g['transition_deadline_ns'], 'prepared command extended transition'
            matched += 1
        return dict(matched_safe_commands=matched)
    audit.verify('immutable_prepared_command_payload', prepared_originals)

    accepted = []
    ack_errors = []
    ack_missing = []
    facts = {}
    safe = by_key['safe_prepared_demand']
    fixed_ack_fields = ('handoff_id', 'grant_sequence', 'previous_commit_sequence', 'commit_sequence',
        'incumbent_version', 'candidate_version', 'incumbent_trajectory_id', 'candidate_trajectory_id',
        'execution_id', 'control_epoch', 'sdk_session', 'sdk_arm_generation', 'transport_mode',
        'permit_sequence', 'demand_sequence', 'motion_validation_sequence', 'entry_admission_sequence',
        'applied_at_ns', 'demand_source_stamp_ns', 'demand_body_source_stamp_ns', 'entry_source_stamp_ns',
        'body_source_stamp_ns', 'valid_until_ns', 'applied', 'write_submitted')
    if report.get('handoff_contract') == 'writer_applied_cas_v3':
        fixed_ack_fields += ('vx', 'wz')
    for event in by_key['commit_ack']:
        try:
            identifier = fields(event, ('handoff_id',))[0]
            new = geometry(event, 'candidate_version', 'candidate_trajectory_id')
            w = writer(event)
            fields(event, ('previous_commit_sequence', 'commit_sequence', 'applied', 'applied_at_ns'))
            g = grants.get(identifier) if identifier else None
            if identifier and g is None:
                raise MissingEvidence('ACK has no complete observed conditional grant')
            if not event['applied']:
                assert identifier and event['commit_sequence'] == event['previous_commit_sequence'] == g['expected_commit_sequence'], 'negative ACK changed commit sequence'
                assert new == geometry(g['candidate']) and w == writer(g['candidate']), 'foreign negative ACK'
                continue
            fixed = fields(event, fixed_ack_fields)
            assert event['commit_sequence'] == event['previous_commit_sequence']+1, 'commit is not expected CAS + 1'
            fact_key = w, event['commit_sequence']
            if fact_key in facts:
                assert fixed == fields(facts[fact_key], fixed_ack_fields), 'same commit ACK renewed lease or changed facts'
                continue
            fields(event, ('demand_source_stamp_ns', 'demand_body_source_stamp_ns', 'body_source_stamp_ns', 'valid_until_ns'))
            at = event['applied_at_ns']
            assert 0 <= at-event['demand_source_stamp_ns'] <= 100_000_000, 'old command applied'
            assert 0 <= at-event['demand_body_source_stamp_ns'] <= 100_000_000, 'old original body source applied'
            assert 0 <= at-event['body_source_stamp_ns'] <= 100_000_000, 'old writer state used for entry'
            assert at <= event['valid_until_ns'], 'writer applied after original command evidence expired'
            if identifier:
                assert event['previous_commit_sequence'] == g['expected_commit_sequence'] > 0, 'wrong expected old commit'
                assert geometry(event, 'incumbent_version', 'incumbent_trajectory_id') == geometry(g['incumbent']), 'wrong incumbent ACK'
                assert new == geometry(g['candidate']) and w == writer(g['candidate']), 'wrong candidate ACK'
                assert event['permit_sequence'] == g['candidate']['permit_sequence'], 'wrong conditional permit'
                assert g['source_stamp_ns'] <= at <= g['transition_deadline_ns'], 'write outside fixed transition'
                assert event['entry_source_stamp_ns'] == event['demand_body_source_stamp_ns'], 'original entry source restamped'
                matched = [s for s in safe if s['handoff_id'] == identifier
                    and s.get('demand_sequence') == event['demand_sequence']
                    and geometry(s) == new and writer(s) == w
                    and s['permit_sequence'] == event['permit_sequence']
                    and s['motion_validation_sequence'] == event['motion_validation_sequence']
                    and s['entry_admission_sequence'] == event['entry_admission_sequence']
                    and s['demand_source_stamp_ns'] == event['demand_source_stamp_ns']
                    and s['body_source_stamp_ns'] == event['demand_body_source_stamp_ns']]
                if not matched:
                    raise MissingEvidence('applied replacement has no exact safe prepared command witness')
                if report.get('handoff_contract') == 'writer_applied_cas_v3':
                    fields(event, ('vx', 'wz'))
                    fields(matched[0], ('vx', 'wz'))
                    assert (event['vx'],event['wz']) == (matched[0]['vx'],matched[0]['wz']), 'conditional ACK changed original prepared velocity'
                assert at <= event['valid_until_ns'] <= min(matched[0]['valid_until_ns'],
                    g['candidate']['valid_until_ns'], g['transition_deadline_ns']), 'ACK extended original prepared deadline'
                if not event['motion_validation_sequence']:
                    assert not nonzero(matched[0]), 'nonzero prepared command has no exact native sweep'
                    native = [v for v in by_key['validation']
                        if v.get('sequence') == g['candidate']['validation_sequence'] and geometry(v) == new]
                    if not native:
                        raise MissingEvidence('zero prepared geometry has no exact full-curve native proof')
                    proof = native[0]
                    fields(proof, ('valid', 'whole_curve', 'remaining_curve', 'checked_from_time',
                        'checked_to_time', 'curve_duration', 'support_reference_id', 'support_hash',
                        'map_snapshot_revision', 'check_begin_ns', 'check_end_ns', 'valid_until_ns',
                        'body_source_stamp_ns', 'front_ray_source_stamp_ns', 'rear_ray_source_stamp_ns'))
                    assert proof['valid'] and proof['whole_curve'] and not proof['remaining_curve'], 'zero demand cannot bypass full candidate geometry validation'
                    assert proof['checked_from_time'] <= 0 and proof['checked_to_time'] >= proof['curve_duration'] > 0, 'incomplete candidate curve range'
                    assert proof['support_reference_id'] and len(proof['support_hash']) == 64 and proof['map_snapshot_revision'] > 0, 'unbound candidate support'
                    assert proof['check_begin_ns'] <= proof['check_end_ns'] <= at <= proof['valid_until_ns'], 'zero candidate evidence expired or postdated writer call'
                    assert event['valid_until_ns'] <= proof['valid_until_ns'], 'zero conditional ACK extended native geometry deadline'
                    assert 0 <= at-proof['body_source_stamp_ns'] <= 100_000_000, 'zero candidate body evidence too old'
                    assert all(0 <= at-proof[k] <= 500_000_000 for k in ('front_ray_source_stamp_ns', 'rear_ray_source_stamp_ns')), 'zero candidate two-LiDAR evidence too old'
                    facts[fact_key] = event
                    accepted.append(event)
                    continue
                sweep = [m for m in by_key['motion_validation']
                    if m.get('sequence') == event['motion_validation_sequence']
                    and m.get('handoff_id') == identifier
                    and geometry(m) == new and writer(m) == w
                    and m.get('demand_sequence') == event['demand_sequence']]
                if not sweep:
                    raise MissingEvidence('applied replacement has no exact native command-sweep observation')
                proof = sweep[0]
                if report.get('handoff_contract') == 'writer_applied_cas_v3':
                    fields(proof, ('vx', 'wz'))
                    assert (proof['vx'],proof['wz']) == (matched[0]['vx'],matched[0]['wz']), 'conditional native sweep used a different command velocity'
                fields(proof, ('valid', 'permit_sequence', 'entry_admission_sequence',
                    'demand_source_stamp_ns', 'demand_body_source_stamp_ns', 'demand_valid_until_ns',
                    'check_begin_ns', 'check_end_ns', 'valid_until_ns', 'map_snapshot_revision',
                    'body_source_stamp_ns', 'front_ray_source_stamp_ns', 'rear_ray_source_stamp_ns'))
                assert proof['valid'] and proof['map_snapshot_revision'] > 0, 'invalid or unbound native command sweep'
                assert proof['permit_sequence'] == event['permit_sequence'], 'native sweep used wrong conditional permission'
                assert proof['entry_admission_sequence'] == event['entry_admission_sequence'], 'native sweep used wrong entry witness'
                assert proof['demand_source_stamp_ns'] == event['demand_source_stamp_ns'], 'native sweep restamped original command'
                assert proof['demand_body_source_stamp_ns'] == event['demand_body_source_stamp_ns'], 'native sweep restamped original body'
                assert proof['demand_valid_until_ns'] == matched[0]['valid_until_ns'], 'native sweep changed original command deadline'
                assert proof['check_begin_ns'] <= proof['check_end_ns'] <= at <= proof['valid_until_ns'], 'native sweep expired or postdated actual writer call'
                assert event['valid_until_ns'] <= proof['valid_until_ns'], 'conditional ACK extended native sweep deadline'
                assert 0 <= at-proof['body_source_stamp_ns'] <= 100_000_000, 'native actual-body evidence was too old'
                assert all(0 <= at-proof[k] <= 500_000_000 for k in ('front_ray_source_stamp_ns', 'rear_ray_source_stamp_ns')), 'native two-LiDAR evidence was too old'
            else:
                assert event['previous_commit_sequence'] == 0 and event['commit_sequence'] == 1 and event['grant_sequence'] == 0, 'invalid initial writer commit'
                issued = [p for p in by_key['permit'] if p.get('sequence') == event['permit_sequence']
                    and geometry(p) == new and writer(p) == w and p.get('allowed')
                    and p.get('geometry_committed') and not p.get('revoked')
                    and p['source_stamp_ns'] <= at <= p['valid_until_ns']]
                if not issued:
                    raise MissingEvidence('initial ACK has no exact original authorized ordinary lease')
                if report.get('handoff_contract') == 'writer_applied_cas_v3':
                    fields(event, ('vx','wz'))
                    commands = [d for d in by_key['safe_demand']
                        if d.get('demand_sequence',d.get('sequence')) == event['demand_sequence']]
                    if not commands:
                        raise MissingEvidence('initial writer application has no exact safe command')
                    original = [o for o in by_key['demand']
                        if o.get('demand_sequence',o.get('sequence')) == event['demand_sequence']]
                    if not original:
                        raise MissingEvidence('initial safe command has no original tracker demand')
                    o = original[0]
                    payload_fields = ('permit_sequence','source_stamp_ns','body_source_stamp_ns','valid_until_ns','vx','wz')
                    for command in commands:
                        assert geometry(command) == new and writer(command) == w, 'initial safe command has foreign geometry or writer'
                        assert command['permit_sequence'] == event['permit_sequence'], 'initial safe command used different permission'
                        assert geometry(o) == geometry(command) and writer(o) == writer(command), 'initial safe command changed original identity'
                        assert fields(o,payload_fields) == fields(command,payload_fields), 'initial safe command altered original payload or deadline'
                    # The independent safety gate can upgrade only the proof
                    # signature of the SAME immutable command. Bind the actual
                    # writer ACK to that exact witness, not the observer's first
                    # safe receipt. All original payloads remain checked above.
                    matching_commands = [command for command in commands if
                        command['motion_validation_sequence'] == event['motion_validation_sequence']]
                    if not matching_commands:
                        raise MissingEvidence('initial ACK has no exact safe proof signature witness')
                    d = matching_commands[0]
                    assert d['source_stamp_ns'] == event['demand_source_stamp_ns'], 'initial command source restamped'
                    assert d['body_source_stamp_ns'] == event['demand_body_source_stamp_ns'], 'initial body source restamped'
                    assert (d['vx'],d['wz']) == (event['vx'],event['wz']), 'initial writer applied a different command'
                    assert at <= event['valid_until_ns'] <= d['valid_until_ns'], 'initial ACK extended original command deadline'
                    assert d['motion_validation_sequence'] == event['motion_validation_sequence'], 'initial writer changed native sweep signature'
                    if not event['motion_validation_sequence']:
                        assert not nonzero(d), 'nonzero initial command lacks native sweep'
                        # The writer selects its latest received native proof,
                        # not necessarily the owner's signing floor. Observer
                        # arrival order is not writer receipt order. Bind the
                        # original zero ACK cap to an unambiguous native proof;
                        # when two different proofs could explain that cap we
                        # cannot certify which one was actually selected.
                        native = [v for v in by_key['validation']
                            if v.get('sequence', 0) >= issued[0]['validation_sequence']
                            and geometry(v) == new and v.get('check_end_ns', at+1) <= at]
                        if not native:
                            raise MissingEvidence('zero initial command has no observed native geometry proof')
                        original_native = {}
                        for v in native:
                            payload = {k:value for k,value in v.items() if k != 'elapsed_s'}
                            if v['sequence'] in original_native:
                                assert original_native[v['sequence']] == payload, 'native geometry sequence changed original proof payload'
                            original_native[v['sequence']] = payload
                        bound = {v['sequence']: v for v in native if min(
                            d['valid_until_ns'], issued[0]['valid_until_ns'],
                            v['valid_until_ns']) == event['valid_until_ns']}
                        assert bound, 'zero initial ACK cap has no original native proof binding'
                        if len(bound) != 1:
                            raise MissingEvidence('zero initial ACK cap ambiguously matches multiple native proofs')
                        proof = next(iter(bound.values()))
                        fields(proof,('valid','whole_curve','remaining_curve','checked_from_time',
                            'checked_to_time','curve_duration','support_reference_id','support_hash',
                            'map_snapshot_revision','check_begin_ns','check_end_ns','valid_until_ns',
                            'body_source_stamp_ns','front_ray_source_stamp_ns','rear_ray_source_stamp_ns'))
                        assert proof['valid'], 'zero initial geometry proof is invalid'
                        assert not any(not v.get('valid') and v['sequence'] > issued[0]['validation_sequence']
                            for v in native), 'zero initial geometry crossed a newer native invalid fence'
                        assert bool(proof['whole_curve']) != bool(proof['remaining_curve']), 'zero initial proof has ambiguous or absent geometry scope'
                        if proof['whole_curve']:
                            assert proof['checked_from_time'] <= 0 and proof['checked_to_time'] >= proof['curve_duration'] > 0, 'zero initial command has incomplete curve range'
                            assert 0 <= at-proof['body_source_stamp_ns'] <= 100_000_000, 'zero initial body too old'
                            assert all(0 <= at-proof[k] <= 500_000_000 for k in ('front_ray_source_stamp_ns','rear_ray_source_stamp_ns')), 'zero initial two-LiDAR evidence too old'
                        else:
                            fields(proof, ('valid_start_time','reverse_margin_m','frame_id','collision_policy'))
                            assert proof['frame_id'] == 'd1max_loc_odom' and proof['collision_policy'] == 'observed_free', 'initial remaining proof uses a different frame or collision policy'
                            assert all(math.isfinite(proof[k]) for k in ('curve_duration','checked_from_time','checked_to_time','valid_start_time','reverse_margin_m')), 'non-finite remaining curve domain'
                            assert 0 < proof['curve_duration'] <= 120 and 0 <= proof['checked_from_time'] <= proof['valid_start_time'] <= proof['checked_to_time'], 'invalid remaining curve domain'
                            assert abs(proof['checked_to_time']-proof['curve_duration']) <= 1e-6 and proof['reverse_margin_m'] >= .15, 'remaining proof omits curve end or reverse coverage'
                            # Remaining geometry is only legitimate after an
                            # independent real software installation of this
                            # same curve; it never admits new CAS geometry.
                            receipts = [r for r in by_key['tracker_geometry_receipt']
                                if r.get('installed') and geometry(r) == new
                                and r.get('installed_at_ns', at+1) <= at]
                            if not receipts:
                                raise MissingEvidence('initial remaining geometry has no actual same-curve installation')
                            original_proofs = [v for v in by_key['validation']
                                if any(v.get('sequence') == r.get('validation_sequence')
                                    and v.get('check_end_ns', at+1) <= r['installed_at_ns'] <= v.get('valid_until_ns', 0)
                                    for r in receipts) and geometry(v) == new]
                            if not original_proofs:
                                raise MissingEvidence('initial remaining geometry lacks original whole installation proof')
                            original_whole = original_proofs[0]
                            fields(original_whole, ('valid','whole_curve','remaining_curve',
                                'checked_from_time','checked_to_time','curve_duration'))
                            assert original_whole['valid'] and original_whole['whole_curve'] and not original_whole['remaining_curve'], 'original installation did not prove whole geometry'
                            assert original_whole['checked_from_time'] == 0 and original_whole['checked_to_time'] >= original_whole['curve_duration'] > 0, 'original whole installation range was incomplete'
                            if not by_key['tracking_progress']:
                                raise MissingEvidence('initial remaining geometry has no typed measured progress')
                            progress = [t for t in by_key['tracking_progress']
                                if fields(t, ('source_stamp_ns',))[0] <= at]
                            assert progress, 'all typed remaining progress sources postdate writer application'
                            t = max(progress, key=lambda item: item['source_stamp_ns'])
                            fields(t, ('schema_version','valid','frame_id','generation','trajectory_id',
                                'session_id','task_id','route_id','route_hash','map_version_id',
                                'localization_epoch','localization_seed_id','segment_id','anchor_id',
                                'anchor_revision','context_sequence','curve_time','arc_length',
                                'position','quaternion','linear_velocity','angular_velocity'))
                            x = issued[0]['version']
                            assert t['schema_version'] == 2 and t['valid'] and t['frame_id'] == 'd1max_loc_odom', 'invalid typed remaining progress'
                            assert t['generation'] == x['reference_generation'] and t['trajectory_id'] == new[1], 'remaining progress uses a different curve or generation'
                            assert all(t[k] == x[k] for k in ('session_id','task_id','route_id','route_hash',
                                'map_version_id','localization_epoch','localization_seed_id','segment_id',
                                'anchor_id','anchor_revision','context_sequence')), 'remaining progress has foreign execution identity'
                            assert 0 <= at-t['source_stamp_ns'] <= 100_000_000, 'remaining progress expired'
                            assert math.isfinite(t['curve_time']) and math.isfinite(t['arc_length']) and t['arc_length'] >= 0 and proof['checked_from_time'] <= t['curve_time'] <= proof['checked_to_time'], 'remaining proof does not cover actual measured progress'
                            local = [s for s in report.get('state_sources', {}).get('local', [])
                                if s.get('source_ns') == event['body_source_stamp_ns']]
                            if not local:
                                raise MissingEvidence('remaining progress has no exact original writer body observation')
                            b = local[0]
                            fields(b, ('usable','frame_id','session_id','map_version_id','localization_epoch',
                                'localization_seed_id','posterior_ns','imu_ns','position','quaternion',
                                'linear_velocity','angular_velocity'))
                            assert b['usable'] and b['frame_id'] == 'd1max_loc_odom' and all(b[k] == x[k] for k in ('session_id','map_version_id','localization_epoch','localization_seed_id')), 'remaining geometry actual body identity invalid'
                            assert 0 <= at-b['source_ns'] <= 100_000_000 and b['posterior_ns'] <= b['source_ns'] and b['imu_ns'] <= b['source_ns'], 'remaining geometry actual body expired or postdated'
                            for state in (t,b):
                                for name,size in (('position',3),('quaternion',4),('linear_velocity',3),('angular_velocity',3)):
                                    assert len(state[name]) == size and all(math.isfinite(v) for v in state[name]), 'remaining geometry contains non-finite body measurement'
                                assert abs(math.sqrt(sum(v*v for v in state['quaternion']))-1) <= .001, 'remaining geometry body quaternion is not normalized'
                            assert math.dist(t['position'], b['position']) <= .0125, 'remaining progress no longer joins actual XYZ'
                            # Local odometry Twist is in body axes; the SDK
                            # rotates it to continuous odom before this check.
                            qx,qy,qz,qw = b['quaternion']; vx,vy,vz = b['linear_velocity']
                            tx,ty,tz = 2*(qy*vz-qz*vy),2*(qz*vx-qx*vz),2*(qx*vy-qy*vx)
                            local_velocity = (vx+qw*tx+qy*tz-qz*ty,
                                vy+qw*ty+qz*tx-qx*tz,vz+qw*tz+qx*ty-qy*tx)
                            assert math.dist(t['linear_velocity'], local_velocity) <= .05, 'remaining progress no longer joins actual velocity'
                            assert 0 <= at-proof['check_end_ns'] <= 250_000_000 and proof['valid_until_ns']-proof['check_end_ns'] <= 250_001_000, 'remaining proof exceeded original native deadline budget'
                            assert 0 <= at-proof['body_source_stamp_ns'] <= 400_000_000, 'remaining proof body too old'
                            assert all(0 <= at-proof[k] <= 600_000_000 for k in ('front_ray_source_stamp_ns','rear_ray_source_stamp_ns')), 'remaining proof two-LiDAR evidence too old'
                        assert proof['support_reference_id'] and len(proof['support_hash']) == 64 and proof['map_snapshot_revision'] > 0, 'zero initial command has unbound support'
                        assert proof['check_begin_ns'] <= proof['check_end_ns'] <= at < proof['valid_until_ns'], 'zero initial geometry proof expired or postdated write'
                        assert event['valid_until_ns'] <= proof['valid_until_ns'], 'zero initial ACK extended native geometry deadline'
                        facts[fact_key]=event;accepted.append(event)
                        continue
                    proofs = [v for v in by_key['motion_validation']
                        if v.get('sequence') == event['motion_validation_sequence']]
                    if not proofs:
                        raise MissingEvidence('initial writer application has no exact native sweep')
                    proof = proofs[0]
                    assert geometry(proof) == new and writer(proof) == w and not proof.get('handoff_id'), 'initial sweep has foreign identity'
                    assert proof['demand_sequence'] == event['demand_sequence'], 'initial sweep checked a different command'
                    fields(proof, ('valid','map_snapshot_revision','permit_sequence','entry_admission_sequence',
                        'demand_source_stamp_ns','demand_body_source_stamp_ns','demand_valid_until_ns',
                        'check_begin_ns','check_end_ns','valid_until_ns','body_source_stamp_ns',
                        'front_ray_source_stamp_ns','rear_ray_source_stamp_ns','vx','wz'))
                    assert proof['valid'] and proof['map_snapshot_revision'] > 0, 'invalid initial command sweep'
                    assert proof['permit_sequence'] == event['permit_sequence'], 'initial sweep used different permission'
                    assert proof['entry_admission_sequence'] == event['entry_admission_sequence'], 'initial sweep changed command entry witness'
                    assert (proof['vx'],proof['wz']) == (d['vx'],d['wz']), 'initial sweep checked different velocity'
                    assert proof['demand_source_stamp_ns'] == d['source_stamp_ns'], 'initial sweep restamped command'
                    assert proof['demand_body_source_stamp_ns'] == d['body_source_stamp_ns'], 'initial sweep restamped body'
                    assert proof['demand_valid_until_ns'] == d['valid_until_ns'], 'initial sweep changed original command deadline'
                    assert proof['check_begin_ns'] <= proof['check_end_ns'] <= at <= proof['valid_until_ns'], 'initial sweep expired or postdated writer application'
                    assert event['valid_until_ns'] <= proof['valid_until_ns'], 'initial ACK extended native sweep deadline'
                    assert 0 <= at-proof['body_source_stamp_ns'] <= 100_000_000, 'initial sweep actual body too old'
                    assert all(0 <= at-proof[k] <= 500_000_000 for k in ('front_ray_source_stamp_ns','rear_ray_source_stamp_ns')), 'initial sweep two-LiDAR evidence too old'
            facts[fact_key] = event
            accepted.append(event)
        except MissingEvidence as exc:
            ack_missing.append(dict(elapsed_s=event['elapsed_s'], reason=str(exc)))
        except (AssertionError, ValueError, TypeError, KeyError) as exc:
            ack_errors.append(dict(elapsed_s=event['elapsed_s'], reason=str(exc)))
    replacements = [a for a in accepted if a['handoff_id']]
    audit.record('writer_submission_facts_and_idempotence',
        'FAIL' if ack_errors else 'UNVERIFIED' if ack_missing or not replacements else 'PASS',
        'Only exact writer submission facts advance geometry; callbacks cannot renew original facts.',
        evidence=dict(initial_commits=sum(not a['handoff_id'] for a in accepted),
            replacement_commits=len(replacements), errors=ack_errors[:32], missing=ack_missing[:32]))

    # This fact is deliberately separate from prepared admission and actual
    # SDK application. Historical v1/v2 reports cannot certify the new receipt
    # contract and are kept as historical protocol evidence only.
    if report.get('handoff_contract') == 'writer_applied_cas_v3':
        def tracker_installations():
            originals = {}
            observed = []
            body_sources = {s['source_ns'] for s in report.get('state_sources', {}).get('local', [])}
            for r in by_key['tracker_geometry_receipt']:
                if not r.get('installed'):
                    continue
                fields(r, ('schema_version', 'sequence', 'installation_sequence', 'permit_sequence',
                    'admission_sequence', 'validation_sequence', 'installed_at_ns', 'body_source_stamp_ns',
                    'frame_id', 'transport_mode'))
                assert r['schema_version'] == 1 and r['sequence'] > 0 and r['installation_sequence'] > 0, 'invalid installation receipt'
                geom = geometry(r)
                fixed = geom, fields(r, ('permit_sequence', 'admission_sequence', 'validation_sequence',
                    'installed_at_ns', 'frame_id', 'transport_mode'))
                key = r['version']['session_id'], r['installation_sequence']
                if key in originals:
                    assert originals[key] == fixed, 'installation heartbeat renewed or changed original facts'
                originals[key] = fixed
                assert r['body_source_stamp_ns'] in body_sources, 'receipt did not preserve original local body source'
                if not by_key['permit']:
                    raise MissingEvidence('installation has no original published geometry lease')
                leases = [p for p in by_key['permit'] if p.get('sequence') == r['permit_sequence']
                    and p.get('geometry_committed') and not p.get('revoked')
                    and geometry(p) == geom and p.get('frame_id') == r['frame_id']
                    and p.get('transport_mode') == r['transport_mode']]
                if not leases:
                    # An already SDK-applied replacement installs from a
                    # conditional grant, not an ordinary pre-application lease.
                    leases = [g['candidate'] for g in grants.values()
                        if g['candidate'].get('permit_sequence') == r['permit_sequence']
                        and geometry(g['candidate']) == geom]
                if not leases:
                    raise MissingEvidence('installation cannot be bound to a published original lease')
                p = leases[0]
                at = r['installed_at_ns']
                applied = [a for a in accepted if a['handoff_id'] and
                    geometry(a, 'candidate_version', 'candidate_trajectory_id') == geom]
                if applied:
                    # An irrevocable writer fact may arrive after its original
                    # authorization expired. Recording/installing that actual
                    # geometry under HOLD does not renew the expired proof.
                    a = applied[0]
                    assert a['applied_at_ns'] <= at, 'tracker claimed installation before writer application'
                    assert a['permit_sequence'] == r['permit_sequence'] and a['entry_admission_sequence'] == r['admission_sequence'], 'replacement installation changed original applied entry'
                    assert r['validation_sequence'] >= p['validation_sequence'], 'replacement installation regressed native proof floor'
                    observed.append(r)
                    continue
                assert p['validation_sequence'] == r['validation_sequence'], 'installation native floor changed'
                assert p['source_stamp_ns'] <= at <= p['valid_until_ns'], 'software installation occurred outside original lease'
                admissions = [a for a in by_key['trajectory_admission'] if a.get('sequence') == r['admission_sequence']
                    and a.get('accepted') and geometry(a) == geom and a.get('validation_sequence') == r['validation_sequence']]
                if not admissions:
                    admissions = [s for s in by_key['safe_prepared_demand']
                        if s.get('entry_admission_sequence') == r['admission_sequence'] and geometry(s) == geom]
                if not admissions:
                    raise MissingEvidence('software installation has no exact accepted original admission')
                a = admissions[0]
                # Conditional installations occur after SDK application; their
                # actual-entry record is separately checked above. Original
                # initial preparation must still be alive at software install.
                if 'checked_at_ns' in a:
                    assert a['checked_at_ns'] <= at <= a['valid_until_ns'], 'initial installation borrowed expired preparation'
                proof = [v for v in by_key['validation'] if v.get('sequence') == r['validation_sequence'] and geometry(v) == geom]
                if not proof:
                    raise MissingEvidence('software installation has no original native proof')
                assert proof[0]['valid'] and proof[0]['check_end_ns'] <= at <= proof[0]['valid_until_ns'], 'software installation borrowed invalid or expired collision proof'
                observed.append(r)
            for ack in accepted:
                if ack['handoff_id']:
                    continue
                assert any(geometry(r) == geometry(ack, 'candidate_version', 'candidate_trajectory_id')
                    and r['installed_at_ns'] <= ack['applied_at_ns'] for r in observed), 'initial SDK application has no actual tracker installation fact'
            return dict(unique_installations=len(originals), receipt_observations=len(observed),
                        scope='software installation only, never SDK or physical acceptance')
        audit.verify('typed_tracker_installation_facts', tracker_installations)

    def commit_chain():
        if not accepted:
            raise MissingEvidence('no fully bound writer commits')
        chains = defaultdict(list)
        for ack in accepted:
            chains[writer(ack)].append(ack)
        lengths = []
        absent_initial = False
        for chain in chains.values():
            chain.sort(key=lambda a: a['applied_at_ns'])
            previous = None
            for ack in chain:
                if previous is None:
                    if known_observer_loss and (ack['commit_sequence'] != 1 or ack['handoff_id']):
                        absent_initial = True
                    else:
                        assert ack['commit_sequence'] == 1 and not ack['handoff_id'], 'writer history lacks initial commit'
                else:
                    assert ack['previous_commit_sequence'] == previous['commit_sequence'], 'writer CAS skipped or forked a commit'
                    assert geometry(ack, 'incumbent_version', 'incumbent_trajectory_id') == geometry(previous, 'candidate_version', 'candidate_trajectory_id'), 'writer CAS predecessor geometry differs'
                previous = ack
            lengths.append(len(chain))
        if absent_initial:
            raise MissingEvidence('initial writer history absent after reported observer loss')
        return dict(writer_chains=len(chains), commit_counts=lengths)
    audit.verify('writer_commit_chain', commit_chain)

    def geometry_advances():
        if not accepted:
            raise MissingEvidence('no fully bound applied writer ACK')
        acknowledged = {geometry(a, 'candidate_version', 'candidate_trajectory_id'): a for a in accepted}
        initial = {geometry(a, 'candidate_version', 'candidate_trajectory_id') for a in accepted if not a['handoff_id']}
        advances = []
        previous = {}
        absent_ack = []
        for event in events:
            key = event['key']
            if key == 'motion':
                if not nonzero(event):
                    continue
                geom = geometry(event)
            elif key == 'permit':
                if not event.get('geometry_committed') or not event.get('allowed') or event.get('revoked'):
                    continue
                if report.get('handoff_contract') == 'writer_applied_cas_v3':
                    # Initial ordinary permits are desired installation intents,
                    # not writer facts. Restaging B can race an in-flight A write
                    # before its ACK reaches the owner. Requiring an ACK for
                    # every unused intent would repeat the production bug this
                    # contract separates. Exact permits actually used are bound
                    # by the writer-command/ACK and installation checks above;
                    # only actual SDK state/output advances geometry here.
                    continue
                geom = geometry(event)
            elif key == 'sdk':
                if not event.get('writer_commit_sequence'):
                    continue
                geom = geometry(event, 'version', 'applied_trajectory_id')
            else:
                continue
            ack = acknowledged.get(geom)
            if ack is None and known_observer_loss:
                absent_ack.append(key)
                continue
            assert ack is not None, 'active geometry advanced without exact writer ACK: '+key
            assert writer(event) == writer(ack), 'geometry advanced under a foreign writer'
            if key == 'permit' and geom not in initial:
                # Initial permission necessarily precedes its first writer
                # call. Replacement permission cannot precede that fact.
                assert event['source_stamp_ns'] >= ack['applied_at_ns'], 'owner signed new active geometry before writer application'
            if key == 'motion':
                fields(event, ('observer_receipt_ns',))
                assert event['observer_receipt_ns'] >= ack['applied_at_ns'], 'new actual command preceded its writer application'
            if key == 'sdk':
                assert event['writer_commit_sequence'] == ack['commit_sequence'], 'SDK commit sequence differs from ACK'
            if geom == previous.get(key):
                continue
            advances.append(dict(key=key, elapsed_s=event['elapsed_s'],
                trajectory_id=geom[1], commit_sequence=ack['commit_sequence']))
            previous[key] = geom
        if not by_key['motion'] or not by_key['sdk']:
            raise MissingEvidence('missing actual writer output or SDK applied-state stream')
        if absent_ack:
            raise MissingEvidence('actual geometry lacks bindable ACK after reported observer loss: '+','.join(sorted(set(absent_ack))))
        return advances
    audit.verify('active_geometry_only_after_applied_ack', geometry_advances)

    diagnostics = {}
    try:
        local = source_stats(report.get('state_sources', {}).get('local', []), source_gap_s)
        global_ = source_stats(report.get('state_sources', {}).get('global', []), source_gap_s)
        diagnostics['original_state_sources'] = dict(local=local, global_=global_)
        audit.record('original_state_sources', 'FAIL' if local['reversed_sources_or_receipts'] or global_['reversed_sources_or_receipts'] else 'PASS',
            'Actual published source samples, not SDK/status heartbeat frequency.')
    except (MissingEvidence, ValueError, TypeError) as exc:
        local = global_ = None
        audit.record('original_state_sources', 'UNVERIFIED' if isinstance(exc, MissingEvidence) else 'FAIL', str(exc))
    motions = by_key['motion']
    try:
        moving = [e for e in motions if nonzero(e)]
        if not moving:
            raise MissingEvidence('no nonzero actual writer output')
        work_begin = moving[0]['elapsed_s']
        terminal = next((e for e in by_key['bt'] if e['elapsed_s'] > work_begin
            and e.get('phase') in ('succeeded', 'failed', 'cancelled', 'completed')), None)
        work_end = terminal['elapsed_s'] if terminal else end
        zeros = intervals(motions, lambda e: not nonzero(e), end)
        zeros = [dict(z, begin_s=max(z['begin_s'], work_begin),
                      end_s=min(z['end_s'], work_end),
                      duration_s=min(z['end_s'], work_end)-max(z['begin_s'], work_begin))
                 for z in zeros if z['begin_s'] < work_end and work_begin < z['end_s']]
        pauses = intervals(by_key['bt'], lambda e: e.get('phase') == 'paused', end)
        sdk_phases_observed = bool(by_key['sdk']) and all('phase' in e for e in by_key['sdk'])
        holds = intervals(by_key['sdk'], lambda e: e.get('phase') == 'holding', end)
        for zero in zeros:
            zero['coincides_with'] = dict(task_pause=any(overlaps(zero, p) for p in pauses),
                sdk_hold=any(overlaps(zero, h) for h in holds) if sdk_phases_observed else None,
                local_source_gap=None if local is None else any(overlaps(zero, g) for g in local['gaps']),
                global_source_gap=None if global_ is None else any(overlaps(zero, g) for g in global_['gaps']))
        diagnostics['actual_zero_intervals'] = dict(scope='observer_receipt_sample_intervals_not_physical_braking',
            moving_begin_s=work_begin, moving_end_s=work_end, count=len(zeros),
            window_end_reason=terminal['phase'] if terminal else 'report_end_no_terminal_observation',
            last_nonzero_s=moving[-1]['elapsed_s'],
            total_s=sum(z['duration_s'] for z in zeros), max_s=max((z['duration_s'] for z in zeros), default=0.),
            intervals=zeros)
        mid_motion = [dict(z, end_s=min(z['end_s'], moving[-1]['elapsed_s']),
                           duration_s=min(z['end_s'],moving[-1]['elapsed_s'])-z['begin_s'])
            for z in zeros if z['begin_s'] < moving[-1]['elapsed_s']]
        diagnostics['zeros_between_nonzero_samples'] = dict(count=len(mid_motion),
            total_s=sum(z['duration_s'] for z in mid_motion),
            max_s=max((z['duration_s'] for z in mid_motion), default=0.))
        associations = []
        for pause in pauses:
            item = dict(pause)
            item['local_source_gaps'] = None if local is None else [g for g in local['gaps'] if overlaps(pause, g)]
            item['global_source_gaps'] = None if global_ is None else [g for g in global_['gaps'] if overlaps(pause, g)]
            if local and item['local_source_gaps']:
                last_gap = item['local_source_gaps'][-1]
                new = [s for s in report['state_sources']['local']
                       if last_gap['end_s'] <= s['elapsed_s'] <= pause['end_s']]
                item['observed_recovery_s'] = pause['end_s']-last_gap['end_s']
                item['distinct_new_sources_before_resume'] = len({s['source_ns'] for s in new})
            associations.append(item)
        diagnostics['task_pause_source_associations'] = associations
        audit.record('actual_output_and_pause_observed', 'PASS',
            'Zero intervals and source coincidences measured; coincidences do not prove causality.')
        if report.get('case') == 'local_gap':
            recovery = [p for p in associations if p.get('local_source_gaps')]
            if not recovery:
                audit.record('local_gap_recovery_window', 'UNVERIFIED', 'No source-associated task pause observed.')
            else:
                good = all(p['observed_recovery_s'] >= .6 and p['distinct_new_sources_before_resume'] >= 3 for p in recovery)
                audit.record('local_gap_recovery_window', 'PASS' if good else 'FAIL',
                    'Observed task resume must follow 0.6 s and three distinct published local sources.', evidence=recovery)
        if report.get('case') == 'global_gap' and local and global_:
            unexpected = [p for p in associations if p['global_source_gaps'] and not p['local_source_gaps']]
            audit.record('global_gap_does_not_revoke_local_task', 'FAIL' if unexpected else 'PASS',
                'Global-only source gaps must not reset the task; independent SDK/map faults need separate diagnosis.', evidence=unexpected)
    except (MissingEvidence, ValueError, TypeError) as exc:
        audit.record('actual_output_and_pause_observed', 'UNVERIFIED', str(exc))

    return dict(schema=1, scope='observed_real_component_graph_software_protocol_only',
        physical_acceptance=False, realtime_acceptance=False, continuity_acceptance=False,
        passed=bool(audit.checks) and all(c['status'] == 'PASS' for c in audit.checks),
        checks=audit.checks, diagnostics=diagnostics,
        warning='Passing protocol invariants is not proof of obstacle clearance, flow quality, physical execution, or 20 ms real-time performance.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('graph_report', type=Path)
    parser.add_argument('--output', type=Path, help='Optional derived audit JSON; input is never modified.')
    parser.add_argument('--source-gap-ms', type=float, default=100., help='Diagnostic state-source gap threshold, not a new safety limit.')
    args = parser.parse_args(argv)
    if not math.isfinite(args.source_gap_ms) or args.source_gap_ms <= 0:
        parser.error('--source-gap-ms must be finite and positive')
    raw = args.graph_report.read_bytes()
    report = json.loads(raw)
    result = audit_report(report, source_gap_s=args.source_gap_ms*1e-3)
    result['input'] = dict(path=str(args.graph_report.resolve()), sha256=hashlib.sha256(raw).hexdigest())
    result['auditor'] = dict(path=str(Path(__file__).resolve()),
        sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    rendered = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.output:
        if args.output.resolve() == args.graph_report.resolve():
            parser.error('output must not overwrite graph input')
        args.output.write_text(rendered)
    else:
        print(rendered, end='')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
