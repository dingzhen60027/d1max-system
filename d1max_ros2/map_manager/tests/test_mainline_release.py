"""Read-only temporary fixtures: no services, ROS, SDK, or setup execution."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

from backend import mainline_release as module


def test_navigation_launch_has_no_fixed_mode_or_implicit_execution():
    raw = dict(launch_policy='navigation')
    resolved = module.activation_launch_config(raw)
    assert resolved['purpose'] == 'planning_only'
    assert resolved['sdk_session_policy'] == 'bind_current_on_start'
    assert raw == dict(launch_policy='navigation')


def test_navigation_launch_resolves_complete_explicit_execution_binding():
    raw = dict(launch_policy='navigation', physical_acceptance_record='/measured/record.json',
               physical_acceptance_record_sha256='a'*64, expected_sdk_session='measured-sdk')
    resolved = module.activation_launch_config(raw)
    assert resolved['purpose'] == 'execution' and resolved['sdk_session_policy'] == 'fixed'
    assert resolved['expected_sdk_session'] == 'measured-sdk'
    assert 'purpose' not in raw


def test_navigation_launch_runtime_profile_needs_no_calibration_report_or_fixed_session():
    raw=dict(launch_policy='navigation',runtime_motion_profile='/release/profile.json',
             runtime_motion_profile_sha256='a'*64)
    resolved=module.activation_launch_config(raw)
    assert resolved['purpose']=='execution'
    assert resolved['sdk_session_policy']=='bind_current_on_start'
    assert resolved['motion_profile_record']=='/release/profile.json'
    assert resolved['motion_profile_sha256']=='a'*64
    assert 'physical_acceptance_record' not in resolved
    assert 'expected_sdk_session' not in resolved
    assert 'purpose' not in raw


@pytest.mark.parametrize('change',[
    dict(runtime_motion_profile='relative'), dict(runtime_motion_profile_sha256='bad'),
    dict(expected_sdk_session='unbound'),dict(physical_acceptance_record='/legacy'),
    dict(physical_acceptance_record_sha256='b'*64)])
def test_navigation_runtime_profile_cannot_mix_or_drop_configuration_identity(change):
    with pytest.raises(ValueError):
        module.activation_launch_config({'launch_policy':'navigation',
            'runtime_motion_profile':'/release/profile.json','runtime_motion_profile_sha256':'a'*64,**change})


@pytest.mark.parametrize('change', [
    dict(purpose='planning_only'), dict(purpose='execution'),
    dict(sdk_session_policy='bind_current_on_start'), dict(launch_policy='auto_motion')])
def test_navigation_launch_cannot_mix_legacy_mode_or_unrecognized_policy(change):
    with pytest.raises(ValueError):
        module.activation_launch_config({'launch_policy':'navigation', **change})


@pytest.mark.parametrize('fields', [
    dict(expected_sdk_session='existing'),
    dict(physical_acceptance_record='/record.json', physical_acceptance_record_sha256='a'*64),
    dict(physical_acceptance_record='relative.json', physical_acceptance_record_sha256='a'*64,
         expected_sdk_session='existing'),
    dict(physical_acceptance_record='/record.json', physical_acceptance_record_sha256='unknown',
         expected_sdk_session='existing')])
def test_navigation_launch_incomplete_or_invalid_bindings_do_not_silently_downgrade(fields):
    with pytest.raises(ValueError, match='navigation_execution_binding'):
        module.activation_launch_config(dict(launch_policy='navigation', **fields))


def write(path, value, *, private=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) if isinstance(value, dict) else value)
    if private:path.chmod(0o600)
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bundle(tmp_path, *, name='entry_v1', frozen=False):
    nav=tmp_path/'nav';release=nav/'experiments'/name
    entry=write(nav/'tools/single_floor_entry.sh','# fixture, never execute\n')
    entry.chmod(0o700)
    checker=write(nav/'tools/release/single_floor_entry_preflight.py','# fixture, never execute\n')
    tools=release/'snapshot/tools' if frozen else nav/'tools'
    if frozen:
        write(tools/'single_floor_entry.sh','# frozen fixture')
        (tools/'single_floor_entry.sh').chmod(0o700)
        write(tools/'release/single_floor_entry_preflight.py','# frozen checker fixture')
    descriptor=dict(schema=1,sealed_manifest='seal.json',entry_module='d1max_pct_scan.single_floor_session')
    if frozen:descriptor.update(template_source_root='snapshot',tools_source_root='snapshot/tools')
    descriptor_path=write(release/'release.json',descriptor)
    payload=write(release/'actual-elf.fixture','large file checks mocked, not ELF')
    files={str(p):digest(p) for p in (descriptor_path,payload,tools/'single_floor_entry.sh',
                                    tools/'release/single_floor_entry_preflight.py')}
    manifest=dict(schema=3,files=files)
    if frozen:manifest['startup_closure']={'schema':1,'fixture':True}
    seal=write(release/'seal.json',manifest)
    profile=dict(schema_version=1,entry_module='d1max_pct_scan.single_floor_session',
        motion_purpose='execution',release_directory='experiments/'+name,release_manifest_sha256=digest(seal))
    selector=write(nav/'deploy/single_floor_release.json',profile)
    activation_path=tmp_path/'private/activation.json'
    activation=dict(schema_version=3,profile='single_floor_live',purpose='planning_only',
        release_root=str(release),entrypoint=str(entry),entrypoint_sha256=digest(entry),
        release_manifest=str(seal),release_manifest_sha256=digest(seal),expected_sdk_session='existing-sdk')
    return dict(nav=nav,release=release,entry=entry,checker=checker,tools=tools,payload=payload,
        seal=seal,selector=selector,activation_path=activation_path,activation=activation)


def settle(probe, predicate=lambda s:s['selected_readiness']!='checking'):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        state=probe.snapshot()
        if predicate(state):return state
        time.sleep(.001)
    raise AssertionError('diagnostic worker did not finish')


def test_missing_activation_still_diagnoses_selected_old_seal(tmp_path):
    b=bundle(tmp_path);b['entry'].write_text('# changed current entry')
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe)
    assert state['selected_id']=='entry_v1' and state['configured_id'] is None
    assert state['readiness']=='missing_activation'
    assert state['selected_readiness']=='invalid' and state['selected_reason_code']=='release_file_changed'
    assert state['sdk_connected'] is False and state['motion_authorized'] is False


def test_valid_legacy_seal_is_not_claimed_as_frozen_new_contract(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready' and state['release_contract']=='legacy_schema3'
    assert state['startup_closure_verified'] is False and state['motion_authorized'] is False


def test_navigation_launch_probe_resolves_same_entry_without_fixed_purpose(tmp_path):
    b = bundle(tmp_path)
    a = dict(b['activation'], launch_policy='navigation')
    a.pop('purpose'); a.pop('expected_sdk_session')
    write(b['activation_path'], a, private=True)
    state = settle(module.MainlineReleaseProbe(b['nav'], b['activation_path']),
                   lambda s:s['readiness']!='checking')
    assert state['readiness'] == 'ready' and state['motion_authorized'] is False
    assert 'purpose' not in json.loads(b['activation_path'].read_text())


def test_launch_hash_verification_is_cancellable_at_chunk_boundary(tmp_path):
    b=bundle(tmp_path);calls=[]
    def cancel():
        calls.append(True)
        raise InterruptedError('offline_cancel')
    with pytest.raises(InterruptedError,match='offline_cancel'):
        module.verify_release_files(b['nav'],b['release'],b['seal'],check_cancelled=cancel)
    assert len(calls)==1


def test_launch_startup_check_uses_owned_runner_not_default_probe_runner(tmp_path,monkeypatch):
    b=bundle(tmp_path,frozen=True);calls=[]
    monkeypatch.setattr(module.subprocess,'run',lambda *args,**kwargs:pytest.fail('unowned_subprocess'))
    def owned(command,**kwargs):
        calls.append(command);return subprocess.CompletedProcess(command,0,'{}','')
    module.verify_release_files(b['nav'],b['release'],b['seal'],runner=owned)
    assert len(calls)==1 and 'startup-check' in calls[0]


def test_launch_keeps_stricter_admitted_frozen_tools_binding(tmp_path):
    b=bundle(tmp_path,frozen=True)
    with pytest.raises(module.ReleaseDiagnosticError,match='tools_binding'):
        module.verify_release_files(b['nav'],b['release'],b['seal'],require_admitted_tools=True)


def test_probe_thread_start_failure_releases_pending_reservation(tmp_path,monkeypatch):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    with monkeypatch.context() as patcher:
        patcher.setattr(module.threading.Thread,'start',lambda _: (_ for _ in ()).throw(RuntimeError('no_thread')))
        assert probe.snapshot()['readiness']=='invalid'
    assert probe._worker is None
    assert settle(probe,lambda state:state['readiness']!='checking')['readiness']=='ready'


def test_generic_root_entry_keeps_old_release_contract_and_both_hash_checks(tmp_path):
    b=bundle(tmp_path);seal_before=b['seal'].read_bytes()
    descriptor_before=(b['release']/'release.json').read_bytes()
    generic=write(b['nav']/'tools/navigation_entry.sh','# generic dispatcher fixture, never execute')
    generic.chmod(0o700)
    a=dict(b['activation'],entrypoint=str(generic),entrypoint_sha256=digest(generic),
        compatibility_entrypoint_sha256=digest(b['entry']))
    write(b['activation_path'],a,private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready' and state['release_contract']=='legacy_schema3'
    assert b['seal'].read_bytes()==seal_before
    assert (b['release']/'release.json').read_bytes()==descriptor_before
    generic.write_text('# changed generic dispatcher')
    state=probe.snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='entrypoint_changed'
    a['entrypoint_sha256']=digest(generic);write(b['activation_path'],a,private=True)
    b['entry'].write_text('# changed sealed legacy implementation')
    assert probe.snapshot()['reason_code']=='entrypoint_changed'
    a['compatibility_entrypoint_sha256']=digest(b['entry']);write(b['activation_path'],a,private=True)
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']=='invalid' and state['reason_code']=='release_file_changed'


def test_generic_frozen_release_binds_current_compatibility_dispatcher(tmp_path,monkeypatch):
    b=bundle(tmp_path,frozen=True)
    generic=write(b['nav']/'tools/navigation_entry.sh','# generic fixture, never execute')
    generic.chmod(0o700)
    a=dict(b['activation'],entrypoint=str(generic),entrypoint_sha256=digest(generic),
        compatibility_entrypoint_sha256=digest(b['entry']))
    write(b['activation_path'],a,private=True)
    monkeypatch.setattr(module.subprocess,'run',lambda command,**kw:subprocess.CompletedProcess(command,0,'{}',''))
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready' and state['release_contract']=='frozen_startup_schema3'
    seal_before=b['seal'].read_bytes();frozen_before=(b['tools']/'single_floor_entry.sh').read_bytes()
    assert str(b['entry']) not in json.loads(b['seal'].read_text())['files']
    b['entry'].write_text('# changed workspace dispatcher, frozen seal unchanged')
    state=probe.snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='entrypoint_changed'
    assert b['seal'].read_bytes()==seal_before
    assert (b['tools']/'single_floor_entry.sh').read_bytes()==frozen_before


def test_generic_activation_requires_compatibility_hash_without_changing_old_activation(tmp_path):
    b=bundle(tmp_path)
    generic=write(b['nav']/'tools/navigation_entry.sh','# generic fixture')
    generic.chmod(0o700)
    a=dict(b['activation'],entrypoint=str(generic),entrypoint_sha256=digest(generic))
    write(b['activation_path'],a,private=True)
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='entrypoint_changed'
    write(b['activation_path'],b['activation'],private=True)
    state=settle(module.MainlineReleaseProbe(b['nav'],b['activation_path']),lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready'


def test_generic_wrapper_fingerprint_withdraws_legacy_entry_cached_readiness(tmp_path):
    b=bundle(tmp_path)
    generic=write(b['nav']/'tools/navigation_entry.sh','# generic fixture')
    generic.chmod(0o700);write(b['activation_path'],b['activation'],private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    settle(probe,lambda s:s['readiness']=='ready')
    generic.write_text('# modified generic wrapper')
    state=probe.snapshot()
    assert state['readiness']=='checking' and state['checked_at_unix'] is None
    assert settle(probe,lambda s:s['readiness']!='checking')['readiness']=='ready'


def test_selected_and_configured_are_independent_and_no_latest_selection(tmp_path):
    b=bundle(tmp_path,name='old');other=b['nav']/'experiments/newer_unsealed';other.mkdir()
    write(other/'release.json',dict(schema=1,sealed_manifest='absent.json'))
    a=dict(b['activation'],release_root=str(other),release_manifest=str(other/'absent.json'))
    write(b['activation_path'],a,private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['selected_id']=='old' and state['selected_readiness']=='ready'
    assert state['configured_id']=='newer_unsealed' and state['readiness']=='invalid'
    assert state['reason_code']=='release_manifest_missing'


def test_strict_selector_manifest_hash_and_path(tmp_path):
    b=bundle(tmp_path)
    p=json.loads(b['selector'].read_text());p['release_manifest_sha256']='0'*64
    write(b['selector'],p)
    state=settle(module.MainlineReleaseProbe(b['nav'],b['activation_path']))
    assert state['selected_reason_code']=='release_manifest_changed'
    p['release_directory']='../outside';write(b['selector'],p)
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['selected_readiness']=='invalid' and state['selected_reason_code']=='selector_invalid'


@pytest.mark.parametrize('kind',['schema','purpose','permissions','session','entry_hash'])
def test_bad_activation_never_says_ready(tmp_path,kind):
    b=bundle(tmp_path);a=dict(b['activation'])
    if kind=='schema':a['schema_version']=2
    elif kind=='purpose':a['purpose']='execution_by_default'
    elif kind=='session':a['expected_sdk_session']=''
    elif kind=='entry_hash':a['entrypoint_sha256']='0'*64
    write(b['activation_path'],a,private=kind!='permissions')
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['motion_authorized'] is False


def test_missing_descriptor_is_not_treated_as_missing_activation(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    (b['release']/'release.json').unlink()
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='release_descriptor_invalid'


def test_missing_or_changed_file_and_descriptor_are_rejected(tmp_path):
    b=bundle(tmp_path)
    assert module.verify_release_files(b['nav'],b['release'],b['seal'])['verified_files']==4
    b['payload'].write_text('changed')
    with pytest.raises(module.ReleaseDiagnosticError,match='release_file_changed'):
        module.verify_release_files(b['nav'],b['release'],b['seal'])
    b['payload'].unlink()
    with pytest.raises(module.ReleaseDiagnosticError,match='release_file_missing'):
        module.verify_release_files(b['nav'],b['release'],b['seal'])


def test_frozen_startup_check_uses_explicit_env_no_global_mutation_or_source(tmp_path,monkeypatch):
    b=bundle(tmp_path,frozen=True);calls=[]
    monkeypatch.setenv('D1MAX_RMW_PREFIX','/fixture/actual-rmw')
    monkeypatch.setenv('D1MAX_NAV_ROOT','/unchanged/server/environment')
    def run(command,**kw):
        calls.append((command,kw))
        return subprocess.CompletedProcess(command,0,'{}','')
    monkeypatch.setattr(module.subprocess,'run',run)
    result=module.verify_release_files(b['nav'],b['release'],b['seal'])
    assert result['startup_closure_verified'] is True
    command,kw=calls[0]
    assert command[:2]==['/usr/bin/python3',str(b['tools']/'release/single_floor_entry_preflight.py')]
    assert command[2:]==['startup-check','--release',str(b['release'])]
    assert kw['env']['D1MAX_NAV_ROOT']==str(b['nav']) and kw['env']['D1MAX_RMW_PREFIX']=='/fixture/actual-rmw'
    assert kw['timeout']<=10 and 'shell' not in kw
    assert module.os.environ['D1MAX_NAV_ROOT']=='/unchanged/server/environment'


@pytest.mark.parametrize('fault',['missing_closure','checker_failure','checker_timeout'])
def test_new_candidate_cannot_bypass_startup_closure(tmp_path,monkeypatch,fault):
    b=bundle(tmp_path,frozen=True)
    if fault=='missing_closure':
        m=json.loads(b['seal'].read_text());del m['startup_closure'];write(b['seal'],m)
    def run(command,**kw):
        if fault=='checker_timeout':raise subprocess.TimeoutExpired(command,kw['timeout'])
        return subprocess.CompletedProcess(command,2,'','closure mismatch')
    monkeypatch.setattr(module.subprocess,'run',run)
    with pytest.raises(module.ReleaseDiagnosticError,match='startup_closure_missing|startup_closure_invalid|release_check_timeout'):
        module.verify_release_files(b['nav'],b['release'],b['seal'])


def test_polling_is_nonblocking_single_worker_and_no_unbounded_queue(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    entered=threading.Event();release=threading.Event();calls=[]
    def verify(*args,**kw):
        calls.append(args);entered.set();assert release.wait(2)
        return dict(manifest_sha256=digest(b['seal']),release_contract='legacy_schema3')
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],verifier=verify)
    assert probe.snapshot()['readiness']=='checking';assert entered.wait(1)
    begin=time.monotonic()
    for _ in range(40):assert probe.snapshot()['readiness']=='checking'
    assert time.monotonic()-begin<.2 and len(calls)==1
    release.set();state=settle(probe,lambda s:s['readiness']=='ready')
    assert len(calls)==1 and state['checked_at_unix'] is not None


def test_ttl_expiry_keeps_same_identity_result_during_one_slow_refresh(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    now=[10.];calls=[];gate=threading.Event();entered=threading.Event();gate.set()
    def verify(*args,**kw):
        calls.append(1);entered.set();assert gate.wait(2)
        return dict(manifest_sha256=digest(b['seal']))
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],ttl_s=1,
        verifier=verify,clock=lambda:now[0])
    original=settle(probe,lambda s:s['readiness']=='ready')
    entered.clear();gate.clear();now[0]=11.1
    try:
        state=probe.snapshot();assert entered.wait(1)
        worker=probe._worker
        assert state['readiness']=='ready' and state['selected_readiness']=='ready'
        assert state['checked_at_unix']==original['checked_at_unix']
        begin=time.monotonic()
        for _ in range(40):
            state=probe.snapshot()
            assert state['readiness']=='ready' and state['checked_at_unix']==original['checked_at_unix']
            assert state['sdk_connected'] is False and state['motion_authorized'] is False
        assert time.monotonic()-begin<.2 and len(calls)==2 and probe._worker is worker
    finally:
        gate.set()
        if probe._worker is not None:probe._worker.join(1)
    assert probe._worker is None
    state=probe.snapshot()
    assert state['readiness']=='ready' and state['checked_at_unix']>original['checked_at_unix']
    assert len(calls)==2


@pytest.mark.parametrize('kind',['selector','activation','descriptor','seal','entry'])
def test_identity_change_withdraws_ready_during_same_key_refresh(tmp_path,kind):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    now=[10.];calls=[];gate=threading.Event();entered=threading.Event();gate.set()
    def verify(*args,**kw):
        calls.append(1);entered.set();assert gate.wait(2)
        return dict(manifest_sha256=kw['expected_manifest_sha256'])
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],ttl_s=1,
        verifier=verify,clock=lambda:now[0])
    original=settle(probe,lambda s:s['readiness']=='ready')
    entered.clear();gate.clear();now[0]=11.1
    try:
        assert probe.snapshot()['readiness']=='ready';assert entered.wait(1)
        worker=probe._worker
        path={'selector':b['selector'],'activation':b['activation_path'],
              'descriptor':b['release']/'release.json','seal':b['seal'],'entry':b['entry']}[kind]
        if kind=='entry':path.write_text('# changed entry fixture')
        else:write(path,{**json.loads(path.read_text()),'fixture_revision':2},private=kind=='activation')
        for _ in range(20):
            state=probe.snapshot()
            assert state['readiness']==('invalid' if kind=='entry' else 'checking')
            assert state['checked_at_unix'] is None
            assert state['motion_authorized'] is False and state['sdk_connected'] is False
        assert len(calls)==2 and probe._worker is worker
    finally:
        gate.set()
        if probe._worker is not None:probe._worker.join(1)
    # The obsolete refresh must not publish its result for the new identity.
    assert probe._cached[2]['checked_at_unix']==original['checked_at_unix']
    assert probe._cached[0]!=probe._requested_key


def test_failed_refresh_replaces_same_identity_ready_and_cached_display_is_not_admission(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    now=[10.];gate=threading.Event();entered=threading.Event();gate.set()
    def verify(*args,**kw):
        entered.set();assert gate.wait(2)
        return module.verify_release_files(*args,**kw)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],ttl_s=1,
        verifier=verify,clock=lambda:now[0])
    original=settle(probe,lambda s:s['readiness']=='ready')
    b['payload'].write_text('changed payload, selection and launch identity unchanged')
    entered.clear();gate.clear();now[0]=11.1
    try:
        state=probe.snapshot();assert entered.wait(1)
        assert state['readiness']=='ready' and state['checked_at_unix']==original['checked_at_unix']
        # Start does this independent full byte check even while diagnostics refresh.
        with pytest.raises(module.ReleaseDiagnosticError,match='release_file_changed'):
            module.verify_release_files(b['nav'],b['release'],b['seal'])
    finally:
        gate.set()
        if probe._worker is not None:probe._worker.join(1)
    state=probe.snapshot()
    assert state['readiness']=='invalid' and state['selected_readiness']=='invalid'
    assert state['reason_code']=='release_file_changed'
    assert state['checked_at_unix']>original['checked_at_unix']
    assert state['motion_authorized'] is False and state['sdk_connected'] is False


def test_tool_change_invalidates_cached_readiness_immediately(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    settle(probe,lambda s:s['readiness']=='ready')
    b['checker'].write_text('# modified checker')
    state=probe.snapshot()
    assert state['readiness']=='checking' and state['checked_at_unix'] is None
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']=='invalid' and state['reason_code']=='release_file_changed'


def test_key_change_during_check_discards_old_ready_no_new_worker_until_retired(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    entered=threading.Event();release=threading.Event();calls=[]
    def verify(*args,**kw):
        calls.append(1);entered.set();assert release.wait(2)
        return dict(manifest_sha256=digest(b['seal']))
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],verifier=verify)
    probe.snapshot();assert entered.wait(1)
    b['checker'].write_text('# new key')
    for _ in range(20):assert probe.snapshot()['readiness']=='checking'
    assert len(calls)==1
    release.set();settle(probe,lambda s:s['readiness']=='ready')
    assert len(calls)==2


def test_execution_record_checksum_is_not_motion_acceptance(tmp_path):
    b=bundle(tmp_path);a=dict(b['activation'],purpose='execution')
    write(b['activation_path'],a,private=True)
    state=settle(module.MainlineReleaseProbe(b['nav'],b['activation_path']),lambda s:s['readiness']!='checking')
    assert state['readiness']=='invalid' and state['reason_code']=='physical_record_invalid'
    record=write(b['release']/'record.json',{'fixture_only':True})
    a.update(physical_acceptance_record=str(record),physical_acceptance_record_sha256=digest(record))
    write(b['activation_path'],a,private=True)
    state=settle(module.MainlineReleaseProbe(b['nav'],b['activation_path']),lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready' and state['motion_authorized'] is False


@pytest.mark.parametrize('reader',['_digest','_json'])
def test_file_readers_reject_fifo_before_blocking_the_only_worker(tmp_path,reader):
    fifo=tmp_path/'replaced-sealed-file';os.mkfifo(fifo)
    # A regressed blocking open is contained in this disposable local child;
    # no verifier/service/SDK process or real release is touched.
    code="""import sys
from backend import mainline_release as m
try:getattr(m,sys.argv[1])(sys.argv[2])
except m.ReleaseDiagnosticError as error:
 assert error.code=='release_invalid';sys.exit(0)
sys.exit(1)
"""
    result=subprocess.run([sys.executable,'-c',code,reader,str(fifo)],
        cwd=Path(__file__).resolve().parents[1],capture_output=True,text=True,timeout=2.)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize('kind',['outside_release','unsealed','physical_true','wrong_kind','stale_hash','valid'])
def test_runtime_profile_readiness_matches_formal_launch_admission(tmp_path,kind):
    b=bundle(tmp_path)
    profile={'record_kind':'runtime_motion_profile','physical_acceptance_verified':False}
    if kind=='physical_true':profile['physical_acceptance_verified']=True
    if kind=='wrong_kind':profile['record_kind']='physical_acceptance_record'
    record=write((tmp_path if kind=='outside_release' else b['release'])/'runtime.json',profile)
    manifest=json.loads(b['seal'].read_text())
    if kind!='unsealed':manifest['files'][str(record)]=digest(record)
    write(b['seal'],manifest)
    a={key:value for key,value in b['activation'].items() if key not in ('purpose','expected_sdk_session')}
    a.update(launch_policy='navigation',runtime_motion_profile=str(record),runtime_motion_profile_sha256=digest(record),
             release_manifest_sha256=digest(b['seal']))
    if kind=='stale_hash':a['runtime_motion_profile_sha256']='0'*64
    write(b['activation_path'],a,private=True)
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'])
    state=settle(probe,lambda s:s['readiness']!='checking')
    assert state['readiness']==('ready' if kind=='valid' else 'invalid')
    if kind!='valid':assert state['reason_code']=='physical_record_invalid'
    assert state['motion_authorized'] is False and state['sdk_connected'] is False
    if kind=='stale_hash':
        # Updating only the Web binding must refresh this same running probe:
        # no Web/SDK restart, profile edit, or movement authorization is needed.
        original_profile=record.read_bytes()
        a['runtime_motion_profile_sha256']=digest(record)
        write(b['activation_path'],a,private=True)
        state=settle(probe,lambda s:s['readiness']!='checking')
        assert state['readiness']=='ready' and state['reason_code']=='ready'
        assert record.read_bytes()==original_profile
        assert state['motion_authorized'] is False and state['sdk_connected'] is False


def test_planning_only_can_defer_current_data_session_binding_without_reading_sdk(tmp_path):
    b=bundle(tmp_path);a=dict(b['activation'],sdk_session_policy='bind_current_on_start')
    a.pop('expected_sdk_session');write(b['activation_path'],a,private=True)
    state=settle(module.MainlineReleaseProbe(b['nav'],b['activation_path']),lambda s:s['readiness']!='checking')
    assert state['readiness']=='ready' and state['sdk_connected'] is False
    a['purpose']='execution';write(b['activation_path'],a,private=True)
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='activation_invalid'


def test_web_activation_cannot_directly_bypass_unique_root_dispatcher(tmp_path):
    b=bundle(tmp_path,frozen=True)
    frozen_entry=b['tools']/'single_floor_entry.sh'
    a=dict(b['activation'],entrypoint=str(frozen_entry),entrypoint_sha256=digest(frozen_entry))
    write(b['activation_path'],a,private=True)
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='entrypoint_changed'


@pytest.mark.parametrize('kind',['other_root','generic_symlink','frozen_generic'])
def test_generic_name_does_not_allow_other_or_frozen_direct_entries(tmp_path,kind):
    b=bundle(tmp_path,frozen=True)
    if kind=='other_root':entry=write(b['nav']/'tools/other_entry.sh','# fixture')
    elif kind=='frozen_generic':entry=write(b['tools']/'navigation_entry.sh','# frozen generic fixture')
    else:
        entry=b['nav']/'tools/navigation_entry.sh';entry.symlink_to(b['entry'])
    entry.chmod(0o700)
    a=dict(b['activation'],entrypoint=str(entry),entrypoint_sha256=digest(entry))
    write(b['activation_path'],a,private=True)
    state=module.MainlineReleaseProbe(b['nav'],b['activation_path']).snapshot()
    assert state['readiness']=='invalid' and state['reason_code']=='entrypoint_changed'
