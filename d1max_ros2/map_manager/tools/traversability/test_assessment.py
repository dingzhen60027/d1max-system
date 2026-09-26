import copy
import hashlib
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml

from build_map import BLOCKED, CONDITIONAL, FREE, candidate_graph, rectangular_heading_masks

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT/'config/traversability/sc_pgo_0904_width_aware.yaml'
RESULT = ROOT/'data/traversability/sc_pgo_20260904_143406_width_aware'


class AssessmentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=yaml.safe_load(CONFIG.read_text())

    def test_graph_cannot_jump_floors_or_cut_corners(self):
        cfg=copy.deepcopy(self.cfg)
        cfg['tomography']['resolution_m']=.15
        cfg['validation']['minimum_seeded_component_area_m2']=0
        xyz=np.array([[0,0,0],[.15,0,0],[0,.15,4],[.15,.15,4],[.30,.15,0]], np.float32)
        nodes={'xyz':xyz,'cell':np.array([[0,0],[1,0],[0,1],[1,1],[2,1]]),
               'state':np.full(5,FREE,np.uint8),'cost':np.ones(5),'region':np.array([0,0,1,1,0])}
        poses=xyz+np.array([0,0,.5])
        edges,_=candidate_graph(nodes,poses,cfg)
        actual={tuple(x) for x in edges.tolist()}
        self.assertEqual(actual,{(0,1),(2,3)})

    def test_narrow_corridor_allows_lengthwise_not_crosswise(self):
        cfg=copy.deepcopy(self.cfg);cfg['tomography']['resolution_m']=.05
        obstacle=np.zeros((81,81),bool);obstacle[32,:]=True;obstacle[48,:]=True
        observed=np.ones_like(obstacle)
        headings,support,_=rectangular_heading_masks(obstacle,observed,cfg)
        bits=int(headings[40,40]);bins=cfg['robot']['heading_bins_half_turn']
        self.assertTrue(bits & (1 << (bins//2)), 'A 0.8 m corridor must allow lengthwise passage')
        self.assertFalse(bits & 1, 'A long body must not be allowed to turn sideways in the corridor')
        self.assertNotEqual(int(support[40,40]),0)
        obstacle[37,:]=True;obstacle[43,:]=True
        headings,_,_=rectangular_heading_masks(obstacle,observed,cfg)
        self.assertEqual(int(headings[40,40]),0,'A 0.3 m opening must stay blocked')

    def test_completed_artifact_invariants(self):
        import json
        r=json.loads((RESULT/'report.json').read_text())
        with np.load(RESULT/'traversability.npz',allow_pickle=False) as d:
            self.assertEqual(len(d['xyz']),r['nodes'])
            self.assertTrue(np.isfinite(d['xyz']).all())
            self.assertEqual(set(d['region']),{0,1,2})
            self.assertFalse(np.any((d['region']==2)&(d['state']==FREE)))
            green=d['state']==FREE
            self.assertTrue(np.isfinite(d['ceiling'][green]).all())
            self.assertTrue(np.all(d['ceiling'][green]-d['xyz'][green,2] >= self.cfg['robot']['standing_height_m']+self.cfg['robot']['vertical_margin_m']-1e-5))
            self.assertTrue(d['heading_checked'][green].all())
            self.assertTrue((d['heading_mask'][green] != 0).all())
            self.assertTrue((d['observed_heading_mask'][green] != 0).all())
            self.assertTrue(np.all(d['observed_support_ratio'][green] >= self.cfg['robot']['minimum_observed_support_ratio']-1e-5))
            self.assertTrue(d['seed_connected'][green].all())
            self.assertTrue(np.isinf(d['cost'][d['state']<CONDITIONAL]).all())
            for region in [0,1]:
                ids=d['region']==region
                self.assertGreater(np.sum(green & ids),100)
                self.assertEqual(len(np.unique(d['cell'][ids],axis=0)),np.sum(ids))
            edges=d['candidate_edges']
            self.assertTrue((d['state'][edges]>=CONDITIONAL).all())
            self.assertTrue(np.all(np.abs(d['cell'][edges[:,0]]-d['cell'][edges[:,1]]).sum(1)==1))
            self.assertTrue(np.all(np.abs(d['xyz'][edges[:,0],2]-d['xyz'][edges[:,1],2])<=self.cfg['robot']['step_candidate_max_m']+1e-5))
            pcd=o3d.io.read_point_cloud(str(RESULT/'traversability.pcd'))
            self.assertTrue(pcd.has_colors())
            self.assertEqual(len(pcd.points),len(d['xyz']))
            self.assertTrue(np.allclose(np.asarray(pcd.points),d['xyz'],atol=1e-5))
        digest=hashlib.sha256(Path(self.cfg['source']['pcd']).read_bytes()).hexdigest()
        self.assertEqual(digest,r['source_sha256'])


if __name__=='__main__':unittest.main()
