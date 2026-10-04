"""Read-only temporary fixtures: no services, ROS, SDK, or setup execution."""
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time

import pytest

from backend import mainline_release as module


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


def test_ttl_expiry_withdraws_ready_and_checks_again(tmp_path):
    b=bundle(tmp_path);write(b['activation_path'],b['activation'],private=True)
    now=[10.];calls=[];gate=threading.Event();gate.set()
    def verify(*args,**kw):
        calls.append(1);assert gate.wait(2)
        return dict(manifest_sha256=digest(b['seal']))
    probe=module.MainlineReleaseProbe(b['nav'],b['activation_path'],ttl_s=1,
        verifier=verify,clock=lambda:now[0])
    settle(probe,lambda s:s['readiness']=='ready');gate.clear();now[0]=11.1
    assert probe.snapshot()['readiness']=='checking'
    gate.set();settle(probe,lambda s:s['readiness']=='ready')
    assert len(calls)==2


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
