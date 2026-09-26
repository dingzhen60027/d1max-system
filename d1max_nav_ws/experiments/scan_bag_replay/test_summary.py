"""Read-only audit must detect changing a measured initial velocity to zero."""
import unittest

from summarize_runs import boundary_audit


def curve(velocity):
    # Cubic Bezier on [0, 1], exactly p(0)=0 and p'(0)=velocity.
    return {'id':1,'order':3,'knots':[0,0,0,0,1,1,1,1],
            'points':[[0,0,0],[velocity/3,0,0],[.1,0,0],[.2,0,0]],
            'native_start_stamp_ns':2200000000}


class SummaryTests(unittest.TestCase):
    def test_exact_source_pose_not_latest_publication_sample(self):
        body=[{'stamp_ns':1000000000,'position':[0,0,0],'orientation':[0,0,0,1],'linear':[-.01,0,0]},
              {'stamp_ns':1100000000,'position':[-.001,0,0],'orientation':[0,0,0,1],'linear':[-.02,0,0]}]
        result=boundary_audit([curve(-.01)],body,1000000000)
        self.assertEqual(result['matched_position_count'],1)
        self.assertEqual(result['changed_velocity_curves'],[])
        self.assertAlmostEqual(result['max_boundary_source_age_s'],.2)

    def test_measured_reverse_velocity_cannot_be_silently_zeroed(self):
        body=[{'stamp_ns':1000000000,'position':[0,0,0],'orientation':[0,0,0,1],'linear':[-.01,0,0]}]
        result=boundary_audit([curve(0)],body,1000000000)
        self.assertEqual(result['changed_velocity_curves'][0]['id'],1)
        self.assertAlmostEqual(result['max_velocity_error_mps'],.01)

    def test_unmatched_pose_is_inconclusive_not_success(self):
        body=[{'stamp_ns':1000000000,'position':[10,0,0],'orientation':[0,0,0,1],'linear':[0,0,0]}]
        result=boundary_audit([curve(0)],body,1000000000)
        self.assertEqual(result['unmatched_curve_ids'],[1])
        self.assertIsNone(result['max_velocity_error_mps'])


if __name__=='__main__':unittest.main()
