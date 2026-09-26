import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from fit_lidar_central_rotation import helpers, interval_mean, integrate, profile_fit, fit_huber, robust_score


class IndependentFitTest(unittest.TestCase):
    def test_fixed_and_free_same_objective(self):
        rng = np.random.default_rng(7)
        x = rng.normal(size=(60, 3))*.2
        r = Rotation.from_euler("xyz", [13, 24, -17], degrees=True).as_matrix()
        y = x@r.T + [.002, -.005, .001] + rng.normal(size=x.shape)*.003
        y[2] += [.4, -.5, .1]
        free, bfree, _ = fit_huber(x, y)
        _, bfixed, _ = fit_huber(x, y, fixed_rotation=r)
        self.assertLessEqual(robust_score(x, y, free, bfree), robust_score(x, y, r, bfixed)+1e-10)

    def test_direction_and_conjugacy(self):
        rng = np.random.default_rng(24)
        x = rng.normal(size=(100, 3))*.15
        r = Rotation.from_euler("xyz", [80, 12, -25], degrees=True).as_matrix()
        b = np.array([.003, -.001, .002])
        fitted, intercept, _ = helpers.fit_rotation(x, x@r.T+b)
        self.assertLess(helpers.rotation_angle(fitted@r.T), 1e-5)
        np.testing.assert_allclose(intercept, b, atol=1e-10)
        delta = Rotation.from_rotvec(x[0]*.1).as_matrix()
        np.testing.assert_allclose(Rotation.from_matrix(r@delta@r.T).as_rotvec(), r@x[0]*.1)

    def test_gap_guard_and_exact_constant_integration(self):
        t = np.arange(0., 1., .005)
        v = np.tile([.1, -.2, .3], (len(t), 1))
        np.testing.assert_allclose(interval_mean(t, v, .101, .202, .020), v[0])
        r = integrate(t, v, .101, .202, np.zeros(3), .020)
        np.testing.assert_allclose(r.as_rotvec(), v[0]*.101, atol=1e-12)
        keep = (t < .15) | (t > .19)
        self.assertTrue(np.isnan(interval_mean(t[keep], v[keep], .101, .202, .020)).all())

    def test_lag_sign_and_profile(self):
        t = np.arange(-.2, 2.21, .002)
        true_lag = .014
        def f(z):
            return np.column_stack([.1*np.sin(8*z), .2*np.cos(5*z), .15*np.sin(13*z+.4)])
        r = Rotation.from_euler("xyz", [15, -72, 39], degrees=True).as_matrix()
        bias = np.array([.002, -.003, .001])
        lidar = f(t)
        gyro = f(t-true_lag)@r.T+bias
        intervals = np.column_stack([np.linspace(0, 1.9, 30), np.linspace(.06, 1.96, 30)])
        x = np.array([interval_mean(t, lidar, a, b, .01) for a, b in intervals])
        lags = np.arange(-30, 31, 2)
        ys = np.array([[interval_mean(t, gyro, a+lag*.001, b+lag*.001, .01)
                        for a, b in intervals] for lag in lags])
        best, (estimated, b, _), _ = profile_fit(x, ys, lags, np.ones(len(x), bool))
        self.assertEqual(lags[best], 14)
        self.assertLess(helpers.rotation_angle(estimated@r.T), .01)
        np.testing.assert_allclose(b, bias, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
