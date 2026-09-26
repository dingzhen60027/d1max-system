"""Pure evidence-check fixtures; no ROS, network, SDK or native processes."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    'offline_live_chain_smoke', Path(__file__).with_name('offline_live_chain_smoke.py'))
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)


class ReferenceSelectionEvidenceTest(unittest.TestCase):
    surface = [[0., 0., -.55], [1., 0., -.55], [1., 2., -.55]]

    def evidence(self):
        return dict(progress_arc=.5, target_arc=2., reference_length=3.,
                    planning_start=[.5, .2, 0.], projection=[.5, 0., 0.],
                    local_target=[1., 1., 0.],
                    selected_reference=[[.5, .2, 0.], [.5, 0., 0.],
                                        [1., 0., 0.], [1., 1., 0.]])

    def validate(self, debug, horizon=1.5):
        return smoke.validate_reference_selection(
            debug, self.surface, [0., 0., 0.], .55, horizon)

    def test_horizon_slice_preserves_corner_and_projection(self):
        result = self.validate(self.evidence())
        self.assertFalse(result['target_is_final'])
        self.assertEqual(result['selected_points'], 4)
        self.assertIn('reference_length_m', json.loads(json.dumps(result, allow_nan=False)))

    def test_short_reference_stops_at_final_target(self):
        debug = self.evidence()
        debug.update(target_arc=3., local_target=[1., 2., 0.])
        debug['selected_reference'][-1] = [1., 2., 0.]
        self.assertTrue(self.validate(debug, horizon=6.)['target_is_final'])

    def test_an_arbitrary_endpoint_is_not_native_selection(self):
        debug = self.evidence()
        debug['local_target'] = [1., 1.1, 0.]
        with self.assertRaises(AssertionError):
            self.validate(debug)

    def test_shortcut_around_corner_is_rejected(self):
        debug = self.evidence()
        del debug['selected_reference'][2]
        with self.assertRaises(AssertionError):
            self.validate(debug)

    def test_ground_body_height_mixup_is_rejected(self):
        debug = deepcopy(self.evidence())
        debug['projection'][2] = -.55
        with self.assertRaises(AssertionError):
            self.validate(debug)

    def test_reported_arc_cannot_override_actual_reference(self):
        debug = self.evidence()
        debug['reference_length'] = 5.
        with self.assertRaises(AssertionError):
            self.validate(debug)


if __name__ == '__main__':
    unittest.main()
