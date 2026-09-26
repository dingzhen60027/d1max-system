"""Native PCT worker protocol with post-optimization validation."""
import time

import numpy as np

from .measured_grid import MeasuredGrid
from .planner_core import TomogramPlanner


def plan_checked(planner, grid, start_xy, goal_xy, max_ground_step_m=0.15):
    start_z = grid.validate_point(start_xy)
    goal_z = grid.validate_point(goal_xy)
    started = time.monotonic()
    path = planner.plan(start_xy, goal_xy)
    if path is None or len(path) == 0:
        raise ValueError('Native PCT search/GPMP failed to find a route')
    # Native A* omits the start node, and rounds endpoints onto grid centers.
    # Include the exact requested endpoints, checking those connectors too.
    path = np.vstack([[*start_xy, start_z], path, [*goal_xy, goal_z]])
    keep = np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-8]
    path = path[keep]
    if len(path) == 1:
        path = np.repeat(path, 2, axis=0)
    checks = grid.validate_path(path, max_ground_step_m)
    return {
        'path': path.tolist(), 'length_m': float(np.linalg.norm(
            np.diff(path, axis=0), axis=1).sum()),
        'elapsed_s': time.monotonic() - started,
        'algorithm': 'upstream PCT OfflineElePlanner + GPMP quintic',
        'source_grid_sha256': grid.sha256,
        'native_parameters': getattr(planner, 'native_parameters', None),
        'height_semantics': 'ground_z', **checks,
    }


def worker_main(settings, requests, results):
    """Runs in a separate process: native bindings need not release the GIL."""
    try:
        grid = MeasuredGrid(settings['planning_grid'], settings['cost_margin_m'],
                            settings['minimum_clearance_m'], settings['optimization_guard_cells'])
        planner = TomogramPlanner(settings['vendor_root'], use_quintic=True,
                                  max_heading_rate=settings['max_heading_rate'], ground_z=True,
                                  astar_cost_weight=settings.get('astar_cost_weight', .2),
                                  optimizer_cost_margin=settings.get('optimizer_cost_margin', 15.0))
        planner.load_payload(grid.payload())
        results.put({'kind': 'ready', 'source_grid_sha256': grid.sha256,
                     'native_parameters': planner.native_parameters,
                     'free_cells': int(grid.free.sum()), 'backend': 'native_pct_gpmp'})
    except Exception as exc:
        results.put({'kind': 'initialization_failed', 'error': str(exc)})
        return
    while True:
        request = requests.get()
        if request is None:
            return
        generation = request['generation']
        try:
            result = plan_checked(planner, grid, request['start_xy'], request['goal_xy'],
                                  settings['max_ground_step_m'])
            results.put({'kind': 'planned', 'generation': generation, **result})
        except Exception as exc:
            results.put({'kind': 'failed', 'generation': generation, 'error': str(exc)})
