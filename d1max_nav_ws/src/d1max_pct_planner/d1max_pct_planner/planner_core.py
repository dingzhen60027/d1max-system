import pickle
import sys
from pathlib import Path

import numpy as np

from .paths import expand, vendor_checkout


def validate_native_parameters(astar_cost_weight=.2, optimizer_cost_margin=15.0):
    """Bound native cost parameters; these do not change the hard cutoff 20.

    Upstream A* discards step_cost below 5, so its legacy weight .2 makes all
    ordinary traversable costs (<=20) contribute zero. A weight such as 1.0
    activates the existing cost preference; it does not admit blocked cells.
    GPMP's optimizer_cost_margin is a cost-unit penalty onset, NOT metres.
    """
    bounds = {'astar_cost_weight': (astar_cost_weight, 0., 10., False, True),
              'optimizer_cost_margin': (optimizer_cost_margin, 0., 20., True, False)}
    for name, (value, low, high, include_low, include_high) in bounds.items():
        if (isinstance(value, (bool, np.bool_))
                or not isinstance(value, (int, float, np.integer, np.floating))
                or not np.isfinite(value)
                or (value < low if include_low else value <= low)
                or (value > high if include_high else value >= high)):
            bracket = ('[' if include_low else '(') + str(low) + ', ' + str(high) + (']' if include_high else ')')
            raise ValueError(f'{name} must be a finite number in {bracket}')
    return {'astar_cost_weight': float(astar_cost_weight),
            'optimizer_cost_margin': float(optimizer_cost_margin)}


def validate_sample_interval(value=10):
    """Native GPMP support spacing in A* path indices, not physical metres."""
    if (isinstance(value, (bool, np.bool_))
            or not isinstance(value, (int, np.integer)) or not 1 <= value <= 100):
        raise ValueError('optimizer_sample_interval must be an integer in [1, 100]')
    return int(value)


class TomogramPlanner:
    def __init__(self, vendor_root, use_quintic=True, max_heading_rate=10.0,
                 ground_z=False, astar_cost_weight=.2, optimizer_cost_margin=15.0,
                 optimizer_sample_interval=10):
        costs = validate_native_parameters(astar_cost_weight, optimizer_cost_margin)
        self.optimizer_sample_interval = validate_sample_interval(optimizer_sample_interval)
        self.astar_cost_weight = costs['astar_cost_weight']
        self.optimizer_cost_margin = costs['optimizer_cost_margin']
        self.native_parameters = {**costs, 'astar_cost_threshold': 20.0,
                                  'optimizer_cost_margin_units': 'cost_not_metres',
                                  'optimizer_sample_interval': self.optimizer_sample_interval,
                                  'optimizer_sample_interval_units': 'astar_path_indices',
                                  'max_heading_rate': float(max_heading_rate),
                                  'use_quintic': bool(use_quintic)}
        # An empty vendor_root means the configured/workspace PCT checkout.
        planner_root = Path(expand(vendor_root) or vendor_checkout()).expanduser().resolve() / 'planner'
        sys.path.insert(0, str(planner_root))
        sys.path.insert(0, str(planner_root / 'lib'))
        from lib import a_star, ele_planner, traj_opt
        self.a_star = a_star
        self.ele_planner = ele_planner
        self.traj_opt = traj_opt
        self.use_quintic = use_quintic
        self.max_heading_rate = max_heading_rate
        self.ground_z = bool(ground_z)

    def load(self, path):
        # Pickle is executable: accept only local, trusted map-builder output.
        with open(path, 'rb') as stream:
            payload = pickle.load(stream)
        self.load_payload(payload)

    def load_payload(self, payload):
        tomo = np.asarray(payload['data'], dtype=np.float32)
        if tomo.ndim != 4 or tomo.shape[0] != 5 or min(tomo.shape[1:]) < 1:
            raise ValueError('Invalid PCT tomogram: expected [5, layers, x, y]')
        self.resolution = float(payload['resolution'])
        self.center = np.asarray(payload['center'], dtype=np.float64)
        if not np.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError('PCT resolution must be finite and positive')
        if self.center.shape != (2,) or not np.isfinite(self.center).all():
            raise ValueError('PCT center must contain finite x/y')
        self.slice_h0 = float(payload['slice_h0'])
        self.slice_dh = float(payload['slice_dh'])
        self.map_dim = [tomo.shape[2], tomo.shape[3]]
        self.offset = np.array([self.map_dim[0] // 2, self.map_dim[1] // 2], dtype=np.int32)
        trav, trav_gx, trav_gy, elev_g, elev_c = tomo
        self.ground = elev_g.copy()
        self.cost = trav.copy()
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
            20, self.optimizer_cost_margin, self.resolution, tomo.shape[1], self.astar_cost_weight,
            trav.reshape(-1, trav.shape[-1]).astype(np.float64),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.float64),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.float64),
            # Keep all six inputs float64 so pybind selects its non-copying
            # strided-Ref overload immediately. Values remain exactly 0/±2.
            gateway.reshape(-1, gateway.shape[-1]).astype(np.float64),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.float64),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.float64))
        if self.optimizer_sample_interval != 10:
            setter = getattr(self.planner, 'set_optimizer_sample_interval', None)
            if setter is None:
                raise RuntimeError('Native PCT must be rebuilt for optimizer_sample_interval')
            setter(self.optimizer_sample_interval)

    def _position_index(self, xy):
        if np.asarray(xy).shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('PCT endpoints must contain finite x/y')
        idx = np.rint((np.asarray(xy) - self.center) / self.resolution).astype(np.int32) + self.offset
        return np.array([idx[1], idx[0]], dtype=np.int32)

    def plan_astar(self, start_xy, goal_xy, start_layer, goal_layer):
        """Explicit native A* strategy, not a fallback from failed GPMP."""
        start = np.r_[int(start_layer), self._position_index(start_xy)].astype(np.int32)
        goal = np.r_[int(goal_layer), self._position_index(goal_xy)].astype(np.int32)
        for index in (start, goal):
            layer, y, x = index
            if (not 0 <= layer < len(self.ground) or not 0 <= x < self.map_dim[0]
                    or not 0 <= y < self.map_dim[1] or not np.isfinite(self.ground[layer,x,y])
                    or self.cost[layer,x,y] > 20):
                raise ValueError('Native A-star endpoint blocked, unsupported or outside map')
        if not self.planner.plan(start, goal, False):
            return None
        grid = np.asarray(self.planner.get_path_finder().get_result_matrix())
        if grid.ndim != 2 or grid.shape[1] != 3 or not len(grid):
            return None
        layers = grid[:,0].astype(int)
        indices = grid[:,1:3].astype(int)
        xy = self.center + (indices-self.offset)*self.resolution
        return {'path': np.c_[xy,self.ground[layers,indices[:,0],indices[:,1]]], 'layer_ids':layers}

    def plan(self, start_xy, goal_xy, start_layer=0, goal_layer=0, return_details=False):
        start = np.zeros(3, dtype=np.int32)
        goal = np.zeros(3, dtype=np.int32)
        start[0], goal[0] = int(start_layer), int(goal_layer)
        start[1:], goal[1:] = self._position_index(start_xy), self._position_index(goal_xy)
        # The upstream extension indexes directly; bounds checks here avoid a
        # native crash rather than relying on a Python exception afterwards.
        for label, index in [('start', start), ('goal', goal)]:
            layer, y, x = index
            if not (0 <= layer < len(self.ground) and 0 <= x < self.map_dim[0]
                    and 0 <= y < self.map_dim[1]):
                raise ValueError(f'PCT {label} is outside the tomogram')
            if not np.isfinite(self.ground[layer, x, y]) or self.cost[layer, x, y] > 20:
                raise ValueError(f'PCT {label} is blocked or lacks ground support')
        if np.array_equal(start, goal):
            # Upstream has an empty path.front() for a start==goal search.
            result = np.array([[*start_xy, self.ground[start[0], start[2], start[1]]],
                               [*goal_xy, self.ground[goal[0], goal[2], goal[1]]]], dtype=float)
            if return_details:
                return {'path': result, 'layer_ids': np.array([start_layer, goal_layer], dtype=int),
                        'native_states': None, 'native_sample_dt': None}
            return result
        # Native Plan owns the short-path guard. One search supplies both GPMP
        # and timing metadata; asking for details must not repeat A*.
        if not hasattr(self.planner, 'get_search_result_size'):
            raise RuntimeError('Native PCT must be rebuilt for single-search planning')
        if not self.planner.plan(start, goal, True):
            return None
        search_size = self.planner.get_search_result_size()
        if search_size == 0:
            return None
        optimizer = (self.planner.get_trajectory_optimizer_wnoj() if self.use_quintic
                     else self.planner.get_trajectory_optimizer())
        trajectory = optimizer.get_result_matrix()
        layers = np.asarray(optimizer.get_layers()).reshape(-1)
        heights = optimizer.get_heights()
        y_index = trajectory.shape[-1] // 2
        grid = np.stack((trajectory[:, 0], trajectory[:, y_index],
                         heights / self.resolution), axis=1)
        offset = np.array([self.map_dim[1] // 2, self.map_dim[0] // 2, 0])
        grid_center = np.array([self.center[1], self.center[0], 0.5])
        mapped = (grid - offset) * self.resolution + grid_center
        result = np.stack((mapped[:, 1], mapped[:, 0], mapped[:, 2]), axis=1)
        if not np.isfinite(result).all():
            raise RuntimeError('Upstream PCT returned a non-finite trajectory')
        if (layers.shape != (len(result),) or not np.isfinite(layers).all()
                or not np.equal(layers, np.rint(layers)).all()):
            raise RuntimeError('Upstream PCT returned invalid trajectory layer identities')
        if self.ground_z:
            # Native quintic heights include a 0.1 m reference height and the
            # historical wrapper adds another 0.5 m. SCAN navi_mode=3 adds its
            # own body height, so the online contract uses measured ground Z.
            for i, xy in enumerate(result[:, :2]):
                y, x = self._position_index(xy)
                layer = int(np.asarray(layers).reshape(-1)[i])
                if not (0 <= layer < len(self.ground) and 0 <= x < self.map_dim[0]
                        and 0 <= y < self.map_dim[1]):
                    raise RuntimeError('Upstream PCT trajectory leaves the tomogram')
                result[i, 2] = self.ground[layer, x, y]
            if not np.isfinite(result).all():
                raise RuntimeError('Upstream PCT trajectory has unsupported ground')
        if return_details:
            # Exact timing used by the upstream quintic GenerateTrajectory:
            # T_tmp=(input_path.size()-1)/3 and uniform interpolated samples.
            duration = (search_size - 1) / 3.0
            return {'path': result, 'layer_ids': layers.astype(int),
                    'native_states': np.asarray(trajectory, dtype=float).copy(),
                    'native_sample_dt': duration / (len(result) - 1) if len(result) > 1 else None}
        return result
