import unittest
import numpy as np
from smooth_ground_path import segment_clear


class SupercoverTests(unittest.TestCase):
    def test_clear(self):
        self.assertTrue(segment_clear([.5,.5],[4.5,4.5],np.ones((5,5),bool),np.zeros(2),1.))

    def test_crossing_obstacle_and_unknown_boundary(self):
        f=np.ones((5,5),bool);f[2,2]=False
        self.assertFalse(segment_clear([.5,.5],[4.5,4.5],f,np.zeros(2),1.))
        self.assertFalse(segment_clear([-.1,.5],[1.5,.5],f,np.zeros(2),1.))

    def test_cannot_touch_blocked_corner_or_edge(self):
        f=np.ones((4,4),bool);f[1,0]=False
        self.assertFalse(segment_clear([.5,.5],[2.5,2.5],f,np.zeros(2),1.))
        self.assertFalse(segment_clear([1.,.2],[1.,.8],f,np.zeros(2),1.))


if __name__=='__main__':unittest.main()
