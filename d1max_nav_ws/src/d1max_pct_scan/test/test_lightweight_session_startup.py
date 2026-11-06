"""Session/display contracts only: no ROS context, process or SDK connection."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from d1max_pct_scan import single_floor_session as entry


@pytest.fixture
def display_session(tmp_path,monkeypatch):
    release=tmp_path/'release'
    sources=[]
    for module,component in (('d1max_pct_scan','application'),('d1max_pct_planner','application'),
                             ('d1max_planning_interfaces','interfaces'),
                             ('d1max_navigation_bt_interfaces','interfaces')):
        path=release/component/'install'/module/'lib'/module/'__init__.py'
        path.parent.mkdir(parents=True);path.write_text('# display fixture\n')
        sources.append(path)
    monkeypatch.setattr(entry.paths,'release_root',lambda:release)
    monkeypatch.setattr(entry.importlib.util,'find_spec',lambda module:
        SimpleNamespace(origin=str(next(p for p in sources if p.parent.name==module))))
    directory=tmp_path/'session';directory.mkdir()
    config=directory/'global_planning.rviz';config.write_text('fixture: rviz\n')
    terrain=release/'terrain.pcd';terrain.write_text('fixture: map\n')
    session=dict(id='display-fixture',pipeline_contract='single_floor_v3',interface_version=3,
        view_contract='single_window_layout_v1',transport_mode='live',
        parameter_hashes={config.name:entry.sha(config)},
        input_hashes={str(terrain):entry.sha(terrain)})
    (directory/'session.json').write_text(json.dumps(session))
    sealed=dict(schema=3,session_sha256=entry.sha(directory/'session.json'),types={'display_types':3},
        files={str(p):entry.sha(p) for p in sources})
    (directory/'runtime_bundle.json').write_text(json.dumps(sealed))
    def forbidden(*args,**kwargs):raise AssertionError('display rediscovered execution dependencies')
    monkeypatch.setattr(entry,'verify',forbidden)
    monkeypatch.setattr(entry.subprocess,'run',forbidden)
    monkeypatch.setattr(entry.subprocess,'Popen',forbidden)
    monkeypatch.setattr('d1max_pct_scan.interface_preflight.validate_execution_interfaces',lambda:{'display_types':3})
    return directory,session,config,terrain,sources


def test_display_checks_exact_binding_without_rebuilding_algorithm_runtime(display_session):
    directory,session,*_=display_session
    assert entry.verify_view(directory)==session


@pytest.mark.parametrize('which',['session','parameter','input','module','bundle'])
def test_display_still_rejects_changed_inputs(display_session,which):
    directory,_,config,terrain,sources=display_session
    target={'session':directory/'session.json','parameter':config,'input':terrain,
            'module':sources[0],'bundle':directory/'runtime_bundle.json'}[which]
    if which=='session':
        value=json.loads(target.read_text());value['id']='another';target.write_text(json.dumps(value))
    elif which=='bundle':
        value=json.loads(target.read_text());value['session_sha256']='0'*64;target.write_text(json.dumps(value))
    else:target.write_text('changed\n')
    with pytest.raises(ValueError,match='changed|not_sealed'):
        entry.verify_view(directory)


def test_display_rejects_development_import(display_session,monkeypatch):
    directory,*_=display_session
    source=directory/'__init__.py';source.write_text('# mutable source\n')
    monkeypatch.setattr(entry.importlib.util,'find_spec',lambda _:SimpleNamespace(origin=str(source)))
    with pytest.raises(ValueError,match='unreleased_view_module'):
        entry.verify_view(directory)


def test_display_rejects_changed_interfaces_without_algorithm_discovery(display_session,monkeypatch):
    directory,*_=display_session
    monkeypatch.setattr('d1max_pct_scan.interface_preflight.validate_execution_interfaces',lambda:{'display_types':2})
    with pytest.raises(ValueError,match='view_interface_contract_changed'):
        entry.verify_view(directory)


@pytest.mark.parametrize('change',['added','deleted'])
def test_display_checks_module_inventory(display_session,change):
    directory,_,_,_,sources=display_session
    if change=='added':sources[0].with_name('unexpected.py').write_text('# unexpected\n')
    else:sources[2].unlink()
    with pytest.raises((ValueError,FileNotFoundError),match='changed_view_module_inventory|No such file'):
        entry.verify_view(directory)


def test_prepare_and_seal_share_one_invocation_and_start_no_process(tmp_path,monkeypatch,capsys):
    events=[]
    monkeypatch.setattr(entry.sys,'argv',['navigation_session','prepare','--seal','--session',str(tmp_path)])
    monkeypatch.setattr(entry,'default_session_purpose',lambda:'execution')
    monkeypatch.setattr(entry,'prepare',lambda directory,**options:
        events.append(('prepare',directory,options)) or dict(id='one-invocation'))
    monkeypatch.setattr(entry,'verify',lambda directory,**options:
        events.append(('seal',directory,options)) or (dict(id='one-invocation'),{}))
    monkeypatch.setattr(entry,'run',lambda *_:pytest.fail('prepare started core'))
    entry.main()
    assert [e[0] for e in events]==['prepare','seal']
    assert events[1][2]==dict(seal_runtime=True)
    assert json.loads(capsys.readouterr().out)['sealed'] is True


def test_seal_flag_cannot_change_other_commands(tmp_path,monkeypatch):
    monkeypatch.setattr(entry.sys,'argv',['navigation_session','run','--seal','--session',str(tmp_path)])
    with pytest.raises(SystemExit) as error:entry.main()
    assert error.value.code==2
