"""Passive copy/rebind of production map contracts, not an acceptance map."""
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/d1max_pct_scan/test'))
from test_navigation_map_package import conditioned_package
from rebase_conditioned_package import rebase, array_identity
from d1max_pct_scan.source_route import digest_file


def copy_package(tmp_path,fixture):
    release=tmp_path/'release'
    release.mkdir()
    (release/'release.json').write_text(json.dumps(dict(schema=1,sealed_manifest='release_manifest.json')))
    target=release/'map/crossfloor'
    shutil.copytree(fixture['directory'],target)
    return release,target


def test_only_deployment_paths_change_and_original_map_is_untouched(tmp_path,conditioned_package):
    fixture=conditioned_package
    before={str(p):digest_file(p) for p in fixture['directory'].rglob('*') if p.is_file()}
    with np.load(fixture['tomogram'],allow_pickle=False) as archive:
        geometry={key:array_identity(archive[key]) for key in archive.files
                  if key not in ('source_pcd','source_processing_manifest')}
    release,target=copy_package(tmp_path,fixture)
    result=rebase(release,target,fixture['directory'])
    assert result['geometry_changed'] is False
    assert result['execution_eligibility_changed'] is False
    assert result['capabilities']['floor_ids']==['floor1','floor2']
    assert result['capabilities']['execution_geometry_eligible'] is False
    assert result['unchanged_arrays']==geometry
    with np.load(target/'pct/tomogram.npz',allow_pickle=False) as archive:
        assert str(archive['source_pcd'].item())==str(target/'processed_map.pcd')
        assert str(archive['source_processing_manifest'].item())==str(target/'manifest.json')
        assert {key:array_identity(archive[key]) for key in geometry}==geometry
    assert {p:digest_file(p) for p in before}==before
    assert result['external_runtime_dependencies']


def test_existing_seal_cannot_be_rewritten(tmp_path,conditioned_package):
    release,target=copy_package(tmp_path,conditioned_package)
    (release/'release_manifest.json').write_text('{}')
    before=digest_file(target/'pct/tomogram.npz')
    with pytest.raises(ValueError,match='sealed_release'):
        rebase(release,target,conditioned_package['directory'])
    assert digest_file(target/'pct/tomogram.npz')==before


def test_different_copied_map_is_not_silently_rebound(tmp_path,conditioned_package):
    release,target=copy_package(tmp_path,conditioned_package)
    (target/'route.yaml').write_text('wrong: map\n')
    before=digest_file(target/'pct/tomogram.npz')
    with pytest.raises(ValueError,match='copied_map_bytes_differ'):
        rebase(release,target,conditioned_package['directory'])
    assert digest_file(target/'pct/tomogram.npz')==before
