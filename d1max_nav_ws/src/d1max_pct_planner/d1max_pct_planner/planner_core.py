import pickle
import sys
from pathlib import Path

import numpy as np


class TomogramPlanner:
    def __init__(self, vendor_root, use_quintic=True, max_heading_rate=10.0):
        planner_root = Path(vendor_root) / 'planner'
        sys.path.insert(0, str(planner_root))
        sys.path.insert(0, str(planner_root / 'lib'))
        from lib import a_star, ele_planner, traj_opt
        self.a_star = a_star
        self.ele_planner = ele_planner
        self.traj_opt = traj_opt
        self.use_quintic = use_quintic
        self.max_heading_rate = max_heading_rate

    def load(self, path):
        with open(path, 'rb') as stream:
            payload = pickle.load(stream)
        tomo = np.asarray(payload['data'], dtype=np.float32)
        self.resolution = float(payload['resolution'])
        self.center = np.asarray(payload['center'], dtype=np.float64)
        self.slice_h0 = float(payload['slice_h0'])
        self.slice_dh = float(payload['slice_dh'])
        self.map_dim = [tomo.shape[2], tomo.shape[3]]
        self.offset = np.array([self.map_dim[0] // 2, self.map_dim[1] // 2], dtype=np.int32)
        trav, trav_gx, trav_gy, elev_g, elev_c = tomo
        elev_g = np.nan_to_num(elev_g, nan=-100.0)
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])
        gateway_up = np.zeros_like(trav, dtype=bool)
        gateway_up[:-1] = (diff_t < -8.0) & (diff_g < 0.1) & (elev_g[1:] > -99.0)
        gateway_down = np.zeros_like(trav, dtype=bool)
        gateway_down[1:] = (diff_t > 8.0) & (diff_g < 0.1) & (elev_g[:-1] > -99.0)
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_down] = -2

        self.planner = self.ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic)
        self.planner.init_map(
            20, 15, self.resolution, tomo.shape[1], 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.float64),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.float64),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.float64),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.float64),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.float64))

    def _position_index(self, xy):
        idx = np.rint((np.asarray(xy) - self.center) / self.resolution).astype(np.int32) + self.offset
        return np.array([idx[1], idx[0]], dtype=np.int32)

    def plan(self, start_xy, goal_xy, start_layer=0, goal_layer=0):
        start = np.zeros(3, dtype=np.int32)
        goal = np.zeros(3, dtype=np.int32)
        start[0], goal[0] = int(start_layer), int(goal_layer)
        start[1:], goal[1:] = self._position_index(start_xy), self._position_index(goal_xy)
        if not self.planner.plan(start, goal, True):
            return None
        path_finder = self.planner.get_path_finder()
        if len(path_finder.get_result_matrix()) == 0:
            return None
        optimizer = (self.planner.get_trajectory_optimizer_wnoj() if self.use_quintic
                     else self.planner.get_trajectory_optimizer())
        trajectory = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()
        y_index = trajectory.shape[-1] // 2
        grid = np.stack((trajectory[:, 0], trajectory[:, y_index],
                         heights / self.resolution), axis=1)
        offset = np.array([self.map_dim[1] // 2, self.map_dim[0] // 2, 0])
        grid_center = np.array([self.center[1], self.center[0], 0.5])
        mapped = (grid - offset) * self.resolution + grid_center
        return np.stack((mapped[:, 1], mapped[:, 0], mapped[:, 2]), axis=1)
