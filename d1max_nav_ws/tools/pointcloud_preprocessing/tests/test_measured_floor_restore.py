"""Real source identity and fail-closed derived-height lineage contracts."""
import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from pointcloud_preprocessing import measured_floor_restore as restore
from pointcloud_preprocessing.pcd_io import read_pcd
from pointcloud_preprocessing.runner import sha256


@pytest.fixture
def sample(tmp_path, monkeypatch):
    dtype = np.dtype([(key, '<f4') for key in ('x', 'y', 'z', 'intensity')])
    records = np.array([(0., 0., -.50, 17.)], dtype=dtype)
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              'COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n').encode()
    source = tmp_path/'source.pcd'
    source.write_bytes(header+records.tobytes())
    run = tmp_path/'run'
    (run/'Scans').mkdir(parents=True)
    scans = []
    for frame in range(5):
        path = run/'Scans'/f'{frame:06d}.pcd'
        path.write_bytes(header+records.tobytes())
        scans.append({'frame':frame, 'scan_path':str(path), 'scan_sha256':sha256(path)})
    poses_path = run/'optimized_poses.txt'
    np.savetxt(poses_path,np.tile(np.eye(4)[:3].reshape(1,-1),(5,1)))
    floor = tmp_path/'floor'
    (floor/'audit').mkdir(parents=True)
    audit = floor/'audit/floor_evidence.npz'
    np.savez_compressed(audit, placeholder=np.array([1]))
    manifest_path = floor/'manifest.json'
    manifest_path.write_text(json.dumps({'source_sha256':sha256(source),'trajectory_path':str(poses_path)}))
    parameters = {'reference_z_m':-.65}
    monkeypatch.setattr(restore,'_floor_field',lambda *args: ({},parameters,[],(0,4)))
    monkeypatch.setattr(restore,'_field',lambda xy,*args: np.full(len(xy),-.5))
    rows=[{'frame':i,'scan_point_index':0,'sample_xyz':[0,0,-.5],'field_z':-.5,'derived_z':-.65}
          for i in range(5)]
    evidence=[{'cell':[2,2],'cell_xy':[0,0],'sample_xyz':[0,0,-.5],'derived_xyz':[0,0,-.65],
               'derived_z':-.65,'field_z':-.5,'source_frame':2,'source_point_index':0,
               'support_frames':list(range(5)),'derived_height_range':[-.65,-.65],
               'independent_observations':rows}]
    sources={'source_pcd':str(source),'source_sha256':sha256(source),
             'conditioned_manifest':str(manifest_path),'conditioned_manifest_sha256':sha256(manifest_path),
             'floor_evidence':str(audit),'floor_evidence_sha256':sha256(audit),
             'optimized_poses':str(poses_path),'optimized_poses_sha256':sha256(poses_path),
             'parameters':parameters,'restored_cells':1,'scans':scans,
             'rule':{'minimum_frames':5,'max_height_range_m':.08,'max_field_residual_m':.055,
                     'snap_to_plane':False,'trajectory_clearing':False}}
    evidence_path,sources_path=tmp_path/'evidence.json',tmp_path/'sources.json'
    def run_test(value=None, provenance=None):
        evidence_path.write_text(json.dumps(evidence if value is None else value))
        sources_path.write_text(json.dumps(sources if provenance is None else provenance))
        return restore.restore_conditioned_records(read_pcd(source),evidence_path,sources_path,manifest_path)
    return run_test,evidence,sources,monkeypatch


def test_source_return_and_intensity_are_preserved_with_verified_derived_z(sample):
    run,evidence,sources,_=sample
    records,verified,hashes=run()
    assert records['z'][0] == np.float32(-.65)
    assert records['intensity'][0] == np.float32(17.)
    assert verified[0]['source_frame_id'] == 2
    assert len(verified[0]['verified_support']) == 5
    assert all(item['scan_path'] in hashes for item in sources['scans'])
    assert 'source_frame_id' not in evidence[0]


def test_claimed_derived_height_cannot_replace_measured_field(sample):
    run,evidence,_,_=sample
    modified=copy.deepcopy(evidence)
    modified[0]['derived_xyz'][2]=-.60
    modified[0]['derived_z']=-.60
    with pytest.raises(ValueError,match='Selected conditioned return'):
        run(modified)


def test_all_supporting_returns_are_revalidated_not_only_selected_return(sample):
    run,evidence,_,_=sample
    modified=copy.deepcopy(evidence)
    modified[0]['independent_observations'][4]['sample_xyz'][2]=-.49
    with pytest.raises(ValueError,match='Support observation'):
        run(modified)


def test_five_claimed_frames_cannot_be_duplicates(sample):
    run,evidence,_,_=sample
    modified=copy.deepcopy(evidence)
    modified[0]['independent_observations'][4]['frame']=3
    modified[0]['support_frames'][4]=3
    with pytest.raises(ValueError,match='independent measured support'):
        run(modified)


def test_changed_support_scan_is_rejected_by_hash(sample):
    run,_,sources,_=sample
    path=Path(sources['scans'][4]['scan_path'])
    path.write_bytes(path.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):
        run()


def test_protected_staircase_is_never_conditioned(sample):
    run,_,_,monkeypatch=sample
    monkeypatch.setattr(restore,'_floor_field',lambda *args: (
        {},{'reference_z_m':-.65},[{'min':[-1,-1,-1],'max':[1,1,1]}],(0,4)))
    with pytest.raises(ValueError,match='protected staircase'):
        run()


def test_invented_cell_support_cannot_be_shifted_to_another_grid_position(sample):
    run,evidence,_,_=sample
    modified=copy.deepcopy(evidence)
    modified[0]['cell_xy']=[.1,0]
    with pytest.raises(ValueError,match='declared 10 cm cell'):
        run(modified)
