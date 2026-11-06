import importlib.util
import json
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('freeze_localization',Path(__file__).with_name('freeze_localization_runtime.py'))
freeze=importlib.util.module_from_spec(spec);spec.loader.exec_module(freeze)


def test_same_byte_runtime_copy_dereferences_build_symlinks_and_records_actual_source(tmp_path):
    nav=tmp_path/'nav';release=tmp_path/'release';release.mkdir()
    (release/'release.json').write_text(json.dumps(dict(sealed_manifest='seal.json')))
    built=tmp_path/'built-elf';built.write_bytes(b'actual-existing-ELF-bytes')
    for name in ('faster_lio','livox_ros_driver2'):
        source=nav/'install'/name;source.mkdir(parents=True)
        (source/'library.so').symlink_to(built)
        (source/'package.xml').write_text('<package/>')
    result=freeze.freeze(nav,release)
    assert result['copied_files']==4 and result['rebuilt_from_source'] is False
    root=Path(result['prefix'])
    for name in ('faster_lio','livox_ros_driver2'):
        assert (root/name/'library.so').read_bytes()==built.read_bytes()
        assert not (root/name/'library.so').is_symlink()
    provenance=json.loads((root.parent/'runtime_copy_provenance.json').read_text())
    assert provenance['physical_acceptance'] is False
    assert provenance['files'][str(root/'faster_lio/library.so')]['source_path']==str(built)
    assert provenance['files'][str(root/'faster_lio/library.so')]['sha256']==freeze.sha(built)
    assert str(nav) not in (root/'local_setup.bash').read_text()
    with pytest.raises(ValueError,match='overwrite'):freeze.freeze(nav,release)
    (release/'seal.json').write_text('{}')
    with pytest.raises(ValueError,match='sealed_release'):freeze.freeze(nav,release)


def test_native_rebuild_updates_only_its_actual_bytes_and_retains_copy_provenance(tmp_path):
    nav=tmp_path/'nav';release=tmp_path/'release';release.mkdir()
    (release/'release.json').write_text(json.dumps(dict(sealed_manifest='seal.json')))
    for name in ('faster_lio','livox_ros_driver2'):
        source=nav/'install'/name;source.mkdir(parents=True)
        (source/'library.so').write_bytes(b'old-bytes')
    source=nav/'src/faster_lio';source.mkdir(parents=True)
    (source/'laser_mapping.cc').write_text('finite weak geometry is recoverable')
    freeze.freeze(nav,release)
    built=tmp_path/'built';built.mkdir()
    (built/'library.so').write_bytes(b'new-native-bytes')
    target=release/'localization_dependencies/install/faster_lio/library.so'
    with pytest.raises(ValueError,match='bytes_mismatch'):
        freeze.record_rebuilt_package(nav,release,built)
    target.write_bytes((built/'library.so').read_bytes())
    result=freeze.record_rebuilt_package(nav,release,built)
    assert result['verified_rebuilt_files']==1 and result['physical_acceptance'] is False
    proof=json.loads((release/'localization_dependencies/runtime_copy_provenance.json').read_text())
    assert proof['rebuilt_packages']==['faster_lio'] and proof['physical_acceptance'] is False
    assert proof['files'][str(target)]['source_path']==str(built/'library.so')
    livox=release/'localization_dependencies/install/livox_ros_driver2/library.so'
    assert proof['files'][str(livox)]['source_path']==str(nav/'install/livox_ros_driver2/library.so')
    assert proof['rebuilds']['faster_lio']['source_files']['laser_mapping.cc']==freeze.sha(source/'laser_mapping.cc')
    (release/'seal.json').write_text('{}')
    with pytest.raises(ValueError,match='sealed_release'):
        freeze.record_rebuilt_package(nav,release,built)
