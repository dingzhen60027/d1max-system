"""Ground-only regressions and byte-exact provenance checks on the real result."""
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from extract_ground import read_source, select_ground, write_xyzi


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'config/traversability/sc_pgo_0904_ground_only.yaml'


class GroundExtractionTests(unittest.TestCase):
    def test_binary_xyzi_roundtrip(self):
        records=np.array([[1.234,-2.8,3.91,84.7],[8.25,0,-5.13,0]],dtype='<f4')
        with tempfile.TemporaryDirectory(prefix='d1max-ground-test-') as temp:
            path=Path(temp)/'test.pcd';write_xyzi(path,records)
            np.testing.assert_array_equal(read_source(path),records)

    def test_two_floors_tread_wall_and_ceiling(self):
        cfg=yaml.safe_load(CONFIG.read_text())
        cfg['floors'][0]['plane']=[0,0,1,0]
        cfg['floors'][1]['plane']=[0,0,1,-4]
        cfg['stairs']['xy_min']=[-.2,-.2];cfg['stairs']['xy_max']=[.6,.6]
        cfg['cleanup'].update(minimum_floor_component_cells=1,minimum_stair_component_cells=1)
        x,y=np.meshgrid(np.linspace(0,.4,9),np.linspace(0,.4,9))
        square=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        xyz=np.concatenate([square+[0,0,z] for z in [0,4,2,6,0]])
        normals=np.tile([0.,0.,1.],(len(xyz),1));normals[-len(square):]=[1.,0.,0.]
        # Three poses also exercise a shorter-than-eight nearest-neighbour input.
        poses=np.array([[.2,.2,.5],[.2,.2,4.5],[.2,.2,2.5]])
        region,offset=select_ground(xyz,normals,poses,cfg)
        self.assertAlmostEqual(offset,.5)
        for k,expected in enumerate([0,1,2,-1,-1]):
            self.assertTrue(np.all(region[k*len(square):(k+1)*len(square)]==expected))

    def test_saved_result_is_original_measured_subset(self):
        cfg=yaml.safe_load(CONFIG.read_text())
        out=(CONFIG.parent/cfg['output']['directory']).resolve()
        report=json.loads((out/'report.json').read_text())
        source=Path(cfg['source']['pcd']);records=read_source(source)
        result=read_source(out/'ground.pcd')
        with np.load(out/'ground_points.npz') as bundle:
            ids=bundle['source_indices'];regions=bundle['region']
            self.assertEqual(len(np.unique(ids)),len(ids))
            np.testing.assert_array_equal(result,records[ids])
            np.testing.assert_array_equal(bundle['xyz'],result[:,:3])
            np.testing.assert_array_equal(bundle['intensity'],result[:,3])
            self.assertEqual(bundle['frame'].item(),'map')
            for k,layer in enumerate(report['layers']):
                self.assertGreater(layer['points'],0)
                part=result[regions==k]
                self.assertEqual(len(part),layer['points'])
                np.testing.assert_array_equal(read_source(out/f"{layer['id']}_ground.pcd"),part)
                binary=np.fromfile(out/layer['file'],dtype='<f4').reshape(-1,6)
                np.testing.assert_array_equal(binary[:,:3],part[:,:3])
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),report['source_sha256'])
        self.assertEqual(report['source_sha256'],'5d709a5d152fd214cbc18c4412d08b5e5652df1d3567252e62b863b8048039eb')
        self.assertEqual(report['ground_points'],len(result))
        self.assertFalse(report['robot_model']);self.assertFalse(report['costmap'])
        self.assertFalse(report['coordinate_change']);self.assertEqual(report['synthetic_points'],0)
        self.assertNotIn('robot',cfg);self.assertFalse(cfg['cleanup']['hole_filling'])


if __name__=='__main__':unittest.main()
