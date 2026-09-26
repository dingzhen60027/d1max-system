"""Rebuild the tomogram in memory per candidate config and run the real native
PCT planner on the stair legs and the full four-leg route. Offline only.
"""
import copy
import os
import subprocess
import sys

import numpy as np
import open3d as o3d
import yaml

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
ROUTE_CFG = '/home/dndx/d1max_nav_ws/src/d1max_pct_planner/config/crossfloor_route_20260923.yaml'
PCD = ('/home/dndx/智元四足机器人D1 Max二次开发文档资料包v0.1.0/d1max_ros2/map_manager/data/processed/'
       '20260923_141005_sc-pgo-0923-crossfloor-structure-v1_2c7218/processed_map.pcd')

if __name__ == '__main__' and os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

from d1max_pct_planner.cpu_tomography import tomogram_from_points  # noqa: E402
from d1max_pct_planner.crossfloor_route import plan_crossfloor  # noqa: E402
from d1max_pct_planner.tomogram_map import TomogramMap  # noqa: E402

# Expected measured ground heights at the recorded portals (map frame).
EXPECTED = {'start': -0.68, 'entry': -0.62, 'landing': 1.40, 'exit': 3.17, 'goal': 3.15}
TRAV = {'kernel_size': 7, 'interval_min': 0.55, 'interval_free': 0.70, 'slope_max_rad': 0.40,
        'step_max': 0.17, 'standable_ratio': 0.20, 'cost_barrier': 50.0,
        'safe_margin': 0.20, 'inflation': 0.20}
WIDE = ([-48.0, -12.0, -3.0], [35.0, 62.0, 15.5])
CROP = ([-40.0, -4.0, -3.0], [26.0, 60.0, 9.0])

VARIANTS = [
    ('A current r.20 slope.40 m.20/i.20', 0.20, 0.50, WIDE, {}),
    ('B r.20 slope.60 m.20/i.20', 0.20, 0.50, WIDE, {'slope_max_rad': 0.60}),
    ('C r.20 slope.60 m.10/i.10', 0.20, 0.50, WIDE, {'slope_max_rad': 0.60, 'safe_margin': .10, 'inflation': .10}),
    ('D r.10 slope.40 m.20/i.20', 0.10, 0.50, CROP, {}),
    ('E r.10 slope.40 m.10/i.15', 0.10, 0.50, CROP, {'safe_margin': .10, 'inflation': .15}),
    ('F r.10 slope.40 m.10/i.10', 0.10, 0.50, CROP, {'safe_margin': .10, 'inflation': .10}),
    ('G r.10 slope.40 m.10/i.15 dh.25', 0.10, 0.25, CROP, {'safe_margin': .10, 'inflation': .15}),
    # Stair incline atan(0.14/0.30) = 0.437 rad exceeds slope 0.40: raise it.
    ('H r.10 slope.55 m.10/i.10', 0.10, 0.50, CROP, {'slope_max_rad': .55, 'safe_margin': .10, 'inflation': .10}),
    ('I r.10 slope.60 m.10/i.10', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10}),
    ('J r.10 slope.65 m.10/i.10', 0.10, 0.50, CROP, {'slope_max_rad': .65, 'safe_margin': .10, 'inflation': .10}),
    ('K r.10 slope.60 m.20/i.20', 0.10, 0.50, CROP, {'slope_max_rad': .60}),
    ('L r.10 slope.60 m.15/i.15', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .15, 'inflation': .15}),
    ('M r.15 slope.60 m.15/i.15', 0.15, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .15, 'inflation': .15}),
    # Sparse ghost points (operator/dynamic) at the stair foot: radius outlier removal.
    ('N r.10 slope.55 m.10/i.10 ror.15/8', 0.10, 0.50, CROP, {'slope_max_rad': .55, 'safe_margin': .10, 'inflation': .10, 'ror': (0.15, 8)}),
    ('O r.10 slope.60 m.10/i.10 ror.15/8', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'ror': (0.15, 8)}),
    ('P r.10 slope.60 m.15/i.15 ror.15/8', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .15, 'inflation': .15, 'ror': (0.15, 8)}),
    ('Q r.10 slope.40 m.10/i.10 ror.15/8', 0.10, 0.50, CROP, {'safe_margin': .10, 'inflation': .10, 'ror': (0.15, 8)}),
    # Free-space (ray-cast) removal of transients from visibility_clean.py votes.
    ('R r.10 slope.60 m.10/i.10 vis10', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.12)}),
    ('S r.10 slope.60 m.15/i.15 vis10', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .15, 'inflation': .15, 'vis': (10, 20, 0.12)}),
    ('T r.10 slope.55 m.10/i.10 vis10', 0.10, 0.50, CROP, {'slope_max_rad': .55, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.12)}),
    ('U r.10 slope.60 m.20/i.20 vis10', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'vis': (10, 20, 0.12)}),
    # R stalls at the stair head: a z=3.24 ghost (F123/H3) survives the 0.12 m protection.
    ('V r.10 slope.60 m.10/i.10 vis10p06', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.06)}),
    ('W r.10 slope.60 m.10/i.10 vis10p0', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.0)}),
    ('X r.10 slope.60 m.10/i.10 vis6p06', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (6, 20, 0.06)}),
    # Same-surface protection: ellipsoid 0.12 m horizontal x 0.05 m vertical.
    ('Y r.10 slope.60 m.10/i.10 vis10e', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.12, 0.05)}),
    ('Z r.10 slope.60 m.10/i.10 vis10e03', 0.10, 0.50, CROP, {'slope_max_rad': .60, 'safe_margin': .10, 'inflation': .10, 'vis': (10, 20, 0.12, 0.03)}),
]


def ror(points, radius, min_nb):
    """Radius outlier removal (quick experiment only)."""
    from scipy.spatial import cKDTree
    counts = cKDTree(points).query_ball_point(points, radius, return_length=True) - 1
    keep = counts >= min_nb
    print(f'   ror r={radius} min={min_nb}: removed {int((~keep).sum())} of {len(points)}', flush=True)
    return points[keep]


def visibility_keep(points, ratio, fmin, protect, protect_dz=None):
    """Keep-mask from visibility_clean.py votes (index-aligned with PCD).

    A candidate is kept if a persistent point lies within `protect` m; with
    `protect_dz` the neighbourhood is an ellipsoid (protect, protect, protect_dz)
    so only the same surface protects it, not a floor below a hovering ghost.
    """
    from scipy.spatial import cKDTree
    v = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'visibility_votes.npz'))
    free, hit, roi = v['free'], v['hit'], v['in_roi']
    assert len(free) == len(points)
    remove = roi & (free >= ratio * np.maximum(hit, 1)) & (free >= fmin)
    if protect > 0:
        persistent = roi & (hit >= 10) & (free < 3 * hit)
        scale = np.array([1.0, 1.0, protect / protect_dz if protect_dz else 1.0])
        tree = cKDTree(points[persistent] * scale)
        near = tree.query_ball_point(points[remove] * scale, protect, return_length=True) > 0
        remove[np.flatnonzero(remove)[near]] = False
    print(f'   visibility F>={ratio}H F>={fmin} protect {protect}/{protect_dz}: removed {int(remove.sum())}',
          flush=True)
    return ~remove


def build(points, res, dh, roi, overrides):
    low, high = np.asarray(roi[0]), np.asarray(roi[1])
    pts = points[np.all((points >= low) & (points <= high), axis=1)]
    overrides = dict(overrides)
    if 'ror' in overrides:
        pts = ror(pts, *overrides.pop('ror'))
    cfg = {'resolution': res, 'slice_dh': dh, 'ground_height': -3.0, 'simplify_layers': False,
           'traversability': {**TRAV, **overrides}}
    payload, _ = tomogram_from_points(pts, cfg)
    payload['frame_id'] = 'd1max_loc_map'
    return TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=0.17)


def snap_anchor(tomo, xy, z_expected, search_m=0.4):
    """Nearest valid cell (XY within search_m) whose measured ground is near z_expected."""
    best = None
    ix, iy = tomo.index(xy)
    r = int(np.ceil(search_m / tomo.resolution))
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            cell = (ix + dx, iy + dy)
            if not tomo.contains(cell):
                continue
            for layer in range(tomo.layers):
                if not tomo.valid[(layer, *cell)]:
                    continue
                g = float(tomo.ground[(layer, *cell)])
                if abs(g - z_expected) > 0.15:
                    continue
                d = tomo.resolution * np.hypot(dx, dy)
                key = (round(d, 6), abs(g - z_expected), layer)
                if d <= search_m and (best is None or key < best[0]):
                    best = (key, tomo.world(cell), g, layer)
    if best is None:
        return None
    _, wxy, g, layer = best
    return {'xyz': [float(wxy[0]), float(wxy[1]), g], 'layer_id': int(layer), 'moved_m': best[0][0]}


def main():
    points = np.asarray(o3d.io.read_point_cloud(PCD).points, dtype=np.float32)
    base = yaml.safe_load(open(ROUTE_CFG))
    only = sys.argv[1:]
    for name, res, dh, roi, overrides in VARIANTS:
        if only and name.split()[0] not in only:
            continue
        overrides = dict(overrides)
        anchor_xy = overrides.pop('anchor_xy', {})
        pts = points
        if 'vis' in overrides:
            pts = points[visibility_keep(points, *overrides.pop('vis'))]
        tomo = build(pts, res, dh, roi, overrides)
        config = copy.deepcopy(base)
        line = f'{name:34s} shape={tomo.data.shape[1:]}'
        anchors = {}
        for key, item in base['anchors'].items():
            snapped = snap_anchor(tomo, anchor_xy.get(key, item['xyz'][:2]), EXPECTED[key])
            if snapped is None:
                anchors = None
                line += f'  anchor {key} has no valid cell'
                break
            anchors[key] = snapped
        if anchors is None:
            print(line, flush=True)
            continue
        moved = max(a['moved_m'] for a in anchors.values())
        config['anchors'] = {k: {'xyz': a['xyz'], 'layer_id': a['layer_id']} for k, a in anchors.items()}
        try:
            result = plan_crossfloor(config, tomo)
            prof = result['stair_profile']
            line += (f'  ROUTE OK length={result["length_m"]:.1f}m transitions={len(result["layer_transitions"])}'
                     f' stair_max_grade={prof["max_edge_grade"]:.2f} anchor_moved<={moved:.2f}m')
        except Exception as exc:
            code = getattr(exc, 'code', type(exc).__name__)
            details = getattr(exc, 'details', {})
            line += f'  FAIL {code}: {exc} {details.get("cause_code", "")} anchor_moved<={moved:.2f}m'
            inner = exc.__cause__
            if inner is not None and getattr(inner, 'details', None):
                d = dict(inner.details)
                cells = d.pop('cells', None)
                if cells:
                    d['cells_xy'] = [tomo.world(tuple(c)).round(2).tolist() for c in cells]
                line += f'\n      cause_details={d}'
        print(line, flush=True)


if __name__ == '__main__':
    main()
