"""Pure deterministic external-runtime identities; no ROS or SDK context."""
from copy import deepcopy
import json
from pathlib import Path

import pytest
import yaml

from d1max_pct_planner import runtime_freeze as freeze
from d1max_pct_planner.compute_budget import DEFAULT,FROZEN_ENV


@pytest.fixture
def runtime(tmp_path,monkeypatch):
    config=tmp_path/'compute.yaml';config.write_text(yaml.safe_dump(DEFAULT))
    env={'D1MAX_PCT_COMPUTE_CONFIG':str(config)}
    packages={}
    for name,dist in freeze.PACKAGES.items():
        root=tmp_path/name;root.mkdir()
        origin=root/'__init__.py';origin.write_text('# fixture-only package\n')
        (root/'native.so').write_bytes(b'not-an-ELF-fixture')
        packages[name]=dict(root=str(root),origin=str(origin),version='fixture-1',distribution=dist)
    monkeypatch.setattr(freeze,'_package_identity',lambda name,dist:dict(packages[name]))
    return config,env,packages


def test_actual_budget_and_package_bytes_are_captured_without_processes(runtime,monkeypatch):
    _,env,_=runtime
    def forbidden(*a,**kw):raise AssertionError('capture spawned a process')
    monkeypatch.setattr(freeze.subprocess,'run',forbidden)
    saved=freeze.capture_runtime_inputs(env)
    assert saved['compute_budget']['effective']==DEFAULT
    assert set(saved['scientific']['packages'])==set(freeze.PACKAGES)
    assert freeze.verify_runtime_inputs(saved,env)==saved
    assert all(Path(p).is_absolute() for p in freeze.contract_files(saved))


def test_internal_frozen_value_must_equal_real_file(runtime):
    _,env,_=runtime
    saved=freeze.capture_runtime_inputs(env)
    equivalent=dict(env,**{FROZEN_ENV:json.dumps(DEFAULT)})
    assert freeze.verify_runtime_inputs(saved,equivalent)==saved
    bad=dict(DEFAULT,nice=6)
    with pytest.raises(ValueError,match='frozen_override_differs'):
        freeze.verify_runtime_inputs(saved,dict(env,**{FROZEN_ENV:json.dumps(bad)}))


def test_an_equal_but_unsealed_budget_file_path_is_rejected(runtime,tmp_path):
    config,env,_=runtime
    saved=freeze.capture_runtime_inputs(env)
    other=tmp_path/'unsealed.yaml';other.write_bytes(config.read_bytes())
    with pytest.raises(ValueError,match='runtime_changed_or_unsealed_override'):
        freeze.verify_runtime_inputs(saved,{'D1MAX_PCT_COMPUTE_CONFIG':str(other)})


def test_used_yaml_change_is_detected_even_if_effective_frozen_value_was_unchanged(runtime):
    config,env,_=runtime
    saved=freeze.capture_runtime_inputs(env)
    config.write_text(yaml.safe_dump(dict(DEFAULT,nice=6)))
    with pytest.raises(ValueError,match='frozen_override_differs'):
        freeze.verify_runtime_inputs(saved,dict(env,**{FROZEN_ENV:json.dumps(DEFAULT)}))
    with pytest.raises(ValueError,match='runtime_changed_or_unsealed_override'):
        freeze.verify_runtime_inputs(saved,env)


@pytest.mark.parametrize('change',['origin','version','python','native','added_python','interpreter'])
def test_actual_scientific_runtime_change_is_not_hidden_by_same_project_source(runtime,change):
    _,env,packages=runtime
    saved=freeze.capture_runtime_inputs(env)
    root=Path(packages['scipy']['root'])
    if change=='origin':
        origin=root/'alternate.py';origin.write_text('# new import\n');packages['scipy']['origin']=str(origin)
    elif change=='version':packages['scipy']['version']='fixture-2'
    elif change=='python':(root/'__init__.py').write_text('# altered solver\n')
    elif change=='native':(root/'native.so').write_bytes(b'altered-native-bytes')
    elif change=='added_python':(root/'new.py').write_text('# newly loadable module\n')
    else:saved['scientific']['interpreter']='/unsealed/interpreter'
    with pytest.raises(ValueError,match='scientific_runtime_changed'):
        freeze.verify_runtime_inputs(saved,env)


def test_loader_dependencies_preserve_absolute_paths_with_spaces(tmp_path,monkeypatch):
    extension=tmp_path/'selected.so';extension.write_bytes(b'\x7fELFfixture')
    lib=tmp_path/'dir with space/dependency.so';lib.parent.mkdir();lib.write_bytes(b'external lib')
    from types import SimpleNamespace
    calls=[]
    def ldd(command,**kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0,stdout=f'\tlib.so => {lib} (0x1)\n')
    monkeypatch.setattr(freeze.subprocess,'run',ldd)
    result=freeze.scientific_loader_dependencies({'files':{str(extension):freeze.sha(extension)}})
    assert result=={str(lib):freeze.sha(lib)}
    assert calls==[['ldd',str(extension)]]


@pytest.mark.parametrize('output,code',[('lib.so => not found',0),('',1)])
def test_unresolved_scientific_loader_is_rejected(tmp_path,monkeypatch,output,code):
    extension=tmp_path/'selected.so';extension.write_bytes(b'\x7fELFfixture')
    from types import SimpleNamespace
    monkeypatch.setattr(freeze.subprocess,'run',lambda *a,**kw:SimpleNamespace(returncode=code,stdout=output))
    with pytest.raises(ValueError,match='dependency_unresolved'):
        freeze.scientific_loader_dependencies({'files':{str(extension):freeze.sha(extension)}})
