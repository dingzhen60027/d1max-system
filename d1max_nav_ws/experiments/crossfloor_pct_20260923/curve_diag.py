"""Run the native stair legs on a cached variant tomogram and render the A*
search path vs the GPMP quintic curve over the masked cost, to see where the
optimised curve leaves the admissible surface. Offline only.

usage: curve_diag.py VARIANT LEG [LEG ...] (variants/legs as in native_sweep.py)
"""
import copy
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, '/home/dndx/d1max_nav_ws/src/d1max_pct_planner')
VENDOR = '/home/dndx/d1max_nav_ws/src/pct_planner_vendor'
if os.environ.get('SWEEP_CHILD') != '1':
    from d1max_pct_planner.native_runtime import prepare_native_environment
    env = prepare_native_environment(VENDOR)
    env['SWEEP_CHILD'] = '1'
    raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:]], env=env).returncode)

import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

import native_sweep as ns  # noqa: E402
from d1max_pct_planner.crossfloor_route import LEGS, _masked_tomogram, validate_config  # noqa: E402
from d1max_pct_planner.tomogram_map import TomogramMap  # noqa: E402
from d1max_pct_planner.tomogram_route import TomogramRoute, expand_native_curve  # noqa: E402


def cached_tomogram(variant):
    name, res, dh, roi, overrides = next(v for v in ns.VARIANTS if v[0].split()[0] == variant)
    path = f'cache_{variant}.npz'
    if not os.path.exists(path):
        import open3d as o3d
        points = np.asarray(o3d.io.read_point_cloud(ns.PCD).points, dtype=np.float32)
        overrides = dict(overrides)
        overrides.pop('anchor_xy', None)
        if 'vis' in overrides:
            points = points[ns.visibility_keep(points, *overrides.pop('vis'))]
        tomo = ns.build(points, res, dh, roi, overrides)
        np.savez_compressed(path, data=tomo.data, resolution=tomo.resolution, center=tomo.center,
                            slice_h0=tomo.slice_h0, slice_dh=tomo.slice_dh,
                            selected_source_layers=tomo.source_layers)
    z = np.load(path)
    payload = {k: z[k] for k in z.files}
    payload.update(resolution=float(payload['resolution']), slice_h0=float(payload['slice_h0']),
                   slice_dh=float(payload['slice_dh']), frame_id='d1max_loc_map')
    return name, TomogramMap(payload, unknown_ceiling_policy='allow_unobserved', max_ground_step_m=0.17)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    overrides = {a[2:].split('=')[0]: [float(v) for v in a.split('=')[1].split(',')]
                 for a in sys.argv[1:] if a.startswith('--')}
    variant, legs = args[0], args[1:]
    name, tomo = cached_tomogram(variant)
    base = yaml.safe_load(open(ns.ROUTE_CFG))
    config = copy.deepcopy(base)
    config['anchors'] = {}
    for key, item in base['anchors'].items():
        a = ns.snap_anchor(tomo, overrides.get(key, item['xyz'][:2]), ns.EXPECTED[key])
        print(f'anchor {key}: {np.round(a["xyz"], 3).tolist()} L{a["layer_id"]}')
        config['anchors'][key] = {'xyz': a['xyz'], 'layer_id': a['layer_id']}
    settings = validate_config(config)
    for leg, a_name, b_name in LEGS:
        if leg not in legs:
            continue
        masked = _masked_tomogram(tomo, leg, settings)
        p = settings['planning']
        route = TomogramRoute(masked, VENDOR, max_heading_rate=p['max_heading_rate'],
                              astar_cost_weight=p['astar_cost_weight'],
                              optimizer_cost_margin=p['optimizer_cost_margin'])
        a, b = settings['anchors'][a_name], settings['anchors'][b_name]
        native = route.planner.plan(a['xyz'][:2], b['xyz'][:2], a['layer_id'], b['layer_id'],
                                    return_details=True)
        if native is None:
            print(f'{leg}: A* failed')
            continue
        search = np.asarray(route.planner.planner.get_path_finder().get_result_matrix())
        # search rows are (layer, native_x=world_y idx, native_y=world_x idx)? print raw head to confirm.
        print(f'{leg}: A* nodes={len(search)} first rows={search[:2].tolist()} last={search[-1].tolist()}')
        curve = np.asarray(native['path'])
        try:
            expand_native_curve(native, masked)
            print(f'{leg}: curve cells OK')
            bad = None
        except Exception as exc:
            bad = exc.details
            print(f'{leg}: curve FAIL {exc.code} {bad}')
        try:
            route.plan(a['xyz'], b['xyz'], a['layer_id'], b['layer_id'])
            print(f'{leg}: full TomogramRoute.plan OK')
        except Exception as exc:
            print(f'{leg}: full plan FAIL {getattr(exc, "code", exc)} {getattr(exc, "details", "")}')
        # Render cost of every layer the leg uses.
        layers_used = sorted(set(native['layer_ids'].tolist()))
        lo, hi = np.array([-33.0, 46.8]), np.array([-27.5, 56.2])
        i0, i1 = masked.index(lo), masked.index(hi)
        fig, axes = plt.subplots(1, len(layers_used), figsize=(5 * len(layers_used), 8), squeeze=False)
        extent = [masked.world(i0)[1] - masked.resolution / 2, masked.world(i1)[1] + masked.resolution / 2,
                  masked.world(i1)[0] + masked.resolution / 2, masked.world(i0)[0] - masked.resolution / 2]
        for ax, layer in zip(axes[0], layers_used):
            cost = masked.cost[layer, i0[0]:i1[0] + 1, i0[1]:i1[1] + 1].astype(float)
            cost = np.where(masked.valid[layer, i0[0]:i1[0] + 1, i0[1]:i1[1] + 1], cost, np.nan)
            ax.imshow(cost, extent=extent, cmap='RdYlGn_r', vmin=0, vmax=25, interpolation='nearest')
            sel = native['layer_ids'] == layer
            ax.plot(curve[:, 1], curve[:, 0], 'b-', lw=0.8, label='GPMP curve')
            ax.plot(curve[sel, 1], curve[sel, 0], 'bo', ms=2)
            ax.plot([a['xyz'][1], b['xyz'][1]], [a['xyz'][0], b['xyz'][0]], 'k*', ms=10)
            if bad and 'cell_xy' in bad and bad.get('layer_id') == layer:
                w = masked.world(bad['cell_xy'])
                ax.plot(w[1], w[0], 'mx', ms=14, mew=3)
            ax.set_title(f'{leg} L{layer} valid cost (grey=invalid)')
        fig.suptitle(f'{name}  (x down, y right)')
        fig.tight_layout()
        tag = ''.join(f'_{k}{v[0]:.1f}_{v[1]:.1f}' for k, v in overrides.items())
        fig.savefig(f'curve_{variant}_{leg}{tag}.png', dpi=90)


if __name__ == '__main__':
    main()
