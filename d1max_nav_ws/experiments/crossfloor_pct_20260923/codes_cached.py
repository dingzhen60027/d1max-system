"""Per-cell rule codes on a cached variant tomogram (cache_<V>.npz from curve_diag.py).
 . flat  c crossable  , / x inflated>20  I interval barrier  S slope barrier  ' ' no ground
 h valid-mask reject (headroom/step) despite cost<=20; '#' '@' '!' recorded track on free/inflated/barrier
usage: codes_cached.py VARIANT layers lo_x,lo_y hi_x,hi_y"""
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
from d1max_pct_planner.cpu_tomography import height_gradients, traversal_cost  # noqa: E402
from d1max_pct_planner.tomogram_map import TomogramMap  # noqa: E402

os.environ['SWEEP_CHILD'] = '1'  # import only; no native re-exec
import native_sweep as ns  # noqa: E402

variant = sys.argv[1]
layers = [int(v) for v in sys.argv[2].split(',')]
lo, hi = (np.array([float(v) for v in s.split(',')]) for s in sys.argv[3:5])
name, res, dh, roi, overrides = next(v for v in ns.VARIANTS if v[0].split()[0] == variant)
trav = {**ns.TRAV, **{k: v for k, v in overrides.items() if k != 'ror'}}
z = np.load(f'cache_{variant}.npz')
payload = {k: z[k] for k in z.files}
payload.update(resolution=float(payload['resolution']), slice_h0=float(payload['slice_h0']),
               slice_dh=float(payload['slice_dh']), frame_id='d1max_loc_map')
tomo = TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=0.17)
ground = np.nan_to_num(z['data'][3], nan=-1e6)
ceiling = np.nan_to_num(z['data'][4], nan=1e6)
raw = traversal_cost(ground, ceiling, res, trav)
gs, gm = height_gradients(ground)
stand = (1.2 * res * np.tan(trav['slope_max_rad'])) ** 2
poses = np.loadtxt(ns.PCD.replace(ns.PCD, '/home/dndx/d1max_nav_ws/maps/runs/20260923_132601_150357_central_imu_faster_lio_sc_pgo_zenoh/sc_pgo/optimized_poses.txt'))[:, [3, 7, 11]]
track = {tuple(tomo.index(p[:2])) for p in poses[700:960]}
i0, i1 = tomo.index(lo), tomo.index(hi)
for l in layers:
    inf = z['data'][0][l]
    flat = gs[l] <= stand
    nflat = ndimage.convolve((gs[l] < stand).astype(int), np.ones((7, 7), int), mode='constant')
    cross = (~flat) & (gm[l] <= trav['step_max'] ** 2) & (nflat >= int(trav['standable_ratio'] * 49) - 1)
    interval = ceiling[l] - ground[l]
    print(f'\nL{l} slice_top={tomo.slice_h0 + l * dh:+.2f}  rows x from {lo[0]} ; cols y from {lo[1]} step {res}')
    for i in range(i0[0], i1[0] + 1):
        row = []
        for j in range(i0[1], i1[1] + 1):
            if ground[l, i, j] < -1e5:
                ch = ' '
            elif interval[i, j] < trav['interval_min']:
                ch = 'I'
            elif flat[i, j]:
                ch = '.'
            elif cross[i, j]:
                ch = 'c'
            else:
                ch = 'S'
            if inf[i, j] > 20 and ch in '.c':
                ch = {'.': ',', 'c': 'x'}[ch]
            elif ch in '.c' and not tomo.valid[l, i, j]:
                ch = 'h'
            if (i, j) in track:
                ch = '#' if ch in '.c' else ('@' if ch in ',xh' else '!')
            row.append(ch)
        print(f'{tomo.world((i, 0))[0]:6.1f} ' + ''.join(row))
