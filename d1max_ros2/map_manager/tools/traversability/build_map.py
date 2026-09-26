"""Offline, configuration-driven multilevel geometric traversability assessment.

Coordinates remain in the source map frame. No SDK, ROS, or robot control calls.
The stair graph is a candidate graph, never an automatically executable plan.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve

UNKNOWN, BLOCKED, CONDITIONAL, FREE = 0, 1, 2, 3
COLORS = np.array([[111, 125, 148], [238, 91, 97], [250, 182, 56], [43, 207, 143]], np.uint8)


def log(message):
    print(message, flush=True)


def load_input(cfg):
    src = Path(cfg['source']['pcd']).resolve(strict=True)
    cloud = o3d.io.read_point_cloud(str(src))
    points = np.asarray(cloud.points)
    if len(points) != cfg['source']['expected_points']:
        raise ValueError('Source point count does not match the selected map')
    bounds = np.asarray(cfg['preprocess']['bounds'])
    selected = np.isfinite(points).all(1) & (points >= bounds[0]).all(1) & (points <= bounds[1]).all(1)
    cloud = cloud.select_by_index(np.flatnonzero(selected))
    cloud = cloud.voxel_down_sample(cfg['preprocess']['voxel_m'])
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=cfg['preprocess']['support_normal_radius_m'], max_nn=30))
    points = np.asarray(cloud.points).astype(np.float32)
    support_normals = np.abs(np.asarray(cloud.normals)[:, 2]) >= cfg['preprocess']['support_normal_min_abs_z']
    poses = np.loadtxt(cfg['source']['poses']).reshape(-1, 3, 4)[:, :, 3]
    return src, cloud, points, poses, int(selected.sum()), support_normals


def plane_height(plane, x, y):
    a, b, c, d = plane
    return -(a*x + b*y + d)/c


def shifted(array, dx, dy, fill):
    out = np.full_like(array, fill)
    nx, ny = array.shape
    out[max(0, -dx):min(nx, nx-dx), max(0, -dy):min(ny, ny-dy)] = array[max(0, dx):min(nx, nx+dx), max(0, dy):min(ny, ny+dy)]
    return out


def build_surfaces(points, cfg, support_normals=None):
    tc, rc = cfg['tomography'], cfg['robot']
    res = tc['resolution_m']
    origin = np.floor(points.min(0)[:2]/res)*res
    shape = tuple((np.ceil((points.max(0)[:2]-origin)/res).astype(int)+2).tolist())
    ix, iy = np.floor((points[:, :2]-origin)/res).astype(np.int32).T
    flat_id = ix*shape[1] + iy
    xx, yy = np.meshgrid(origin[0]+(np.arange(shape[0])+.5)*res, origin[1]+(np.arange(shape[1])+.5)*res, indexing='ij')
    refs = [plane_height(f['plane'], xx, yy) for f in tc['floors']]
    roi = tc['connector_roi']
    stairs_xy = (xx >= roi['xy_min'][0]) & (xx <= roi['xy_max'][0]) & (yy >= roi['xy_min'][1]) & (yy <= roi['xy_max'][1])
    minimum_clearance = rc['standing_height_m'] + rc['vertical_margin_m']
    if support_normals is None:
        support_normals = np.ones(len(points), bool)
    ceilings, grounds, costs, states, regions, supports = [], [], [], [], [], []
    for height in np.arange(tc['z_min_m'], tc['z_max_m']+.001, tc['slice_spacing_m']):
        ground = np.full(shape, -np.inf, np.float32)
        ceiling = np.full(shape, np.inf, np.float32)
        below = points[:, 2] <= height
        support_below = below & support_normals
        np.maximum.at(ground.ravel(), flat_id[support_below], points[support_below, 2])
        np.minimum.at(ceiling.ravel(), flat_id[~below], points[~below, 2])
        observed = np.isfinite(ground)
        region = np.full(shape, -1, np.int8)
        for k, floor in enumerate(tc['floors']):
            region[observed & (np.abs(ground-refs[k]) <= floor['tolerance_m'])] = k
        connector = observed & stairs_xy & (ground >= refs[0]-.15) & (ground <= refs[1]+.15)
        region[connector] = 2
        surface = observed & (region >= 0)
        neighbour_count = ndimage.convolve(surface.astype(np.int16), np.ones((3, 3), np.int16), mode='constant')
        max_step = np.zeros(shape, np.float32)
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            adjacent = shifted(ground, dx, dy, np.nan)
            adjacent_ok = shifted(surface, dx, dy, False)
            delta = np.zeros(shape, np.float32)
            valid = surface & adjacent_ok
            delta[valid] = np.abs(ground[valid]-adjacent[valid])
            max_step = np.maximum(max_step, delta)
        gap = ceiling-ground
        # No ceiling return is retained as unverified clearance, not fabricated.
        ceiling_observed = np.isfinite(ceiling)
        terrain_ok = max_step <= rc['step_candidate_max_m']
        if 'candidate_slope_max_deg' in rc:
            slope_limit = res*math.tan(math.radians(rc['candidate_slope_max_deg'])) + rc['ground_noise_tolerance_m']
            # A smooth slope and a discrete tread are different support models.
            terrain_ok = (max_step <= slope_limit) | ((region == 2) & (max_step <= rc['step_candidate_max_m']))
        geom_valid = surface & (gap >= minimum_clearance) & terrain_ok
        enough_support = neighbour_count >= rc['minimum_center_support_neighbours']
        flat_limit = res*math.tan(math.radians(rc['flat_slope_deg']))+rc['ground_noise_tolerance_m']
        state = np.full(shape, UNKNOWN, np.uint8)
        state[surface & ~geom_valid] = BLOCKED
        state[geom_valid] = CONDITIONAL
        state[geom_valid & enough_support & ceiling_observed & (max_step <= flat_limit) & (region != 2)] = FREE
        cost = np.full(shape, np.inf, np.float32)
        cost[geom_valid] = 1 + 8*np.minimum(max_step[geom_valid]/max(flat_limit, 1e-3), 3)
        cost[geom_valid & (region == 2)] += 25
        g = np.where(surface, ground, np.nan)
        ceilings.append(np.where(ceiling_observed, ceiling, np.nan))
        grounds.append(g); costs.append(cost); states.append(state); regions.append(region); supports.append(enough_support)
    arrays = dict(ground=np.stack(grounds), ceiling=np.stack(ceilings), cost=np.stack(costs), state=np.stack(states), region=np.stack(regions), support=np.stack(supports))
    # One node for each observed support surface, not one duplicate per slice.
    level, x, y = np.where(np.isfinite(arrays['ground']))
    z = arrays['ground'][level, x, y]
    keys = np.column_stack((x, y, np.rint(z/tc['surface_merge_m']).astype(np.int32)))
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    # Prefer the slice with the greatest observed clearance/correct state.
    ranking = arrays['state'][level, x, y].astype(float)*1000 + np.nan_to_num(arrays['ceiling'][level, x, y]-z, nan=-1, posinf=0)
    order = np.lexsort((-ranking, inverse))
    first = order[np.r_[True, np.diff(inverse[order]) != 0]]
    l, x, y, z = level[first], x[first], y[first], z[first]
    nodes = dict(xyz=np.column_stack((origin[0]+(x+.5)*res, origin[1]+(y+.5)*res, z)).astype(np.float32),
                 cell=np.column_stack((x, y)).astype(np.int32), slice=l.astype(np.int16),
                 state=arrays['state'][l, x, y], cost=arrays['cost'][l, x, y], region=arrays['region'][l, x, y],
                 ceiling=arrays['ceiling'][l, x, y], support=arrays['support'][l, x, y])
    log(f'Grid {shape}, slices {len(grounds)}, unique observed surfaces {len(z)}')
    # On each main floor export ONE physical support per XY cell. A vertical
    # spread is retained as uncertainty instead of layered overlapping colours.
    region_key = nodes['region'].astype(np.int32)
    zkey = np.where(region_key == 2, np.rint(nodes['xyz'][:, 2]/tc['surface_merge_m']).astype(np.int32), 0)
    keys = np.column_stack((nodes['cell'], region_key, zkey))
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    priority = nodes['state'].astype(float)*1000
    for k, floor in enumerate(tc['floors']):
        ids = nodes['region'] == k
        priority[ids] -= np.abs(nodes['xyz'][ids, 2]-plane_height(floor['plane'], nodes['xyz'][ids, 0], nodes['xyz'][ids, 1]))
    order = np.lexsort((-priority, inv))
    chosen = order[np.r_[True, np.diff(inv[order]) != 0]]
    lo = np.full(inv.max()+1, np.inf); hi = np.full_like(lo, -np.inf)
    np.minimum.at(lo, inv, nodes['xyz'][:, 2]); np.maximum.at(hi, inv, nodes['xyz'][:, 2])
    spread = (hi-lo)[inv[chosen]]
    nodes = {k:v[chosen] for k,v in nodes.items()}
    nodes['surface_spread'] = spread.astype(np.float32)
    ambiguous = (spread > tc['ambiguous_floor_surface_spread_m']) & (nodes['region'] != 2) & (nodes['state'] == FREE)
    nodes['state'][ambiguous] = CONDITIONAL
    log(f'Deduplicated floor cells / stair supports: {len(chosen)}')
    return nodes, arrays, origin, shape


def candidate_graph(nodes, poses, cfg):
    res = cfg['tomography']['resolution_m']
    step = cfg['robot']['step_candidate_max_m']
    possible = np.flatnonzero(nodes['state'] >= CONDITIONAL)
    xyz = nodes['xyz'][possible]
    tree = cKDTree(xyz)
    pairs = tree.query_pairs(math.sqrt(2*res**2+step**2)+1e-5, output_type='ndarray')
    dc = np.abs(nodes['cell'][possible[pairs[:, 0]]] - nodes['cell'][possible[pairs[:, 1]]])
    dz = np.abs(xyz[pairs[:, 0], 2]-xyz[pairs[:, 1], 2])
    # Cardinal neighbours only: no corner cutting, no vertical jumps at same XY.
    good = (dc.sum(1) == 1) & (dc.max(1) == 1) & (dz <= step)
    pairs = pairs[good]
    graph = sparse.coo_matrix((np.ones(len(pairs)*2, np.uint8), (np.r_[pairs[:, 0], pairs[:, 1]], np.r_[pairs[:, 1], pairs[:, 0]])), shape=(len(xyz), len(xyz))).tocsr()
    ncomponents, component = connected_components(graph, directed=False)
    seed_pos = poses.copy(); seed_pos[:, 2] -= cfg['robot']['seed_sensor_height_m']
    dist, nearest = tree.query(seed_pos)
    valid_seed = dist <= cfg['robot']['seed_distance_m']
    sizes = np.bincount(component)
    selected = np.unique(component[nearest[valid_seed]])
    selected = selected[sizes[selected]*res**2 >= cfg['validation']['minimum_seeded_component_area_m2']]
    reachable = np.isin(component, selected)
    retained = np.zeros(len(nodes['xyz']), bool); retained[possible[reachable]] = True
    component_all = np.full(len(nodes['xyz']), -1, np.int32); component_all[possible] = component
    nodes['component'] = component_all
    nodes['seed_connected'] = retained
    nodes['state'][(nodes['state'] >= CONDITIONAL) & ~retained] = UNKNOWN
    nodes['cost'][~retained] = np.inf
    candidate_edges = possible[pairs[reachable[pairs].all(1)]]
    both_floors = []
    for comp in selected:
        regions = set(nodes['region'][possible[component == comp]].tolist())
        if 0 in regions and 1 in regions:
            both_floors.append(int(comp))
    log(f'Candidate graph: {ncomponents} components, {len(selected)} seeded; raw cross-floor components {len(both_floors)}')
    return candidate_edges, dict(total_components=int(ncomponents), seeded_components=len(selected), trajectory_seed_fraction=float(valid_seed.mean()), geometric_cross_floor_components=both_floors)


def rectangular_heading_masks(obstacle, observed, cfg):
    """Collision-free headings for a full length/width rectangle, modulo pi.

    Pixel extent is included in the kernel. Width alone is not used to approve a
    bend, and a successful heading does not imply arbitrary in-place rotation.
    """
    rc, res = cfg['robot'], cfg['tomography']['resolution_m']
    bins = int(rc['heading_bins_half_turn'])
    if not 1 <= bins <= 31:
        raise ValueError('heading_bins_half_turn must be between 1 and 31')
    half_l = rc['length_m']/2 + rc['horizontal_margin_m']
    half_w = rc['width_m']/2 + rc['horizontal_margin_m']
    radius = math.hypot(half_l, half_w)+res
    r = int(math.ceil(radius/res)); dx, dy = np.mgrid[-r:r+1, -r:r+1]
    headings = np.zeros(obstacle.shape, np.uint32)
    support_headings = np.zeros_like(headings)
    max_coverage = np.zeros(obstacle.shape, np.float32)
    for index, yaw in enumerate(np.linspace(0, np.pi, bins, endpoint=False)):
        c, s = math.cos(yaw), math.sin(yaw)
        # Conservative overlap of the rotated body and an occupied square cell.
        pixel = res*.5*(abs(c)+abs(s))
        k = ((np.abs(dx*res*c+dy*res*s) <= half_l+pixel) &
             (np.abs(-dx*res*s+dy*res*c) <= half_w+pixel)).astype(np.float32)
        collision = fftconvolve(obstacle.astype(np.float32), k, mode='same') > .5
        coverage = np.clip(fftconvolve(observed.astype(np.float32), k, mode='same')/k.sum(), 0, 1)
        clear = ~collision
        # Outside cropped grid is not certified space.
        inside = fftconvolve(np.ones(obstacle.shape, np.float32), k, mode='same') >= k.sum()-.5
        clear &= inside
        bit = np.uint32(1 << index)
        headings[clear] |= bit
        supported = clear & (coverage >= rc['minimum_observed_support_ratio'])
        support_headings[supported] |= bit
        max_coverage = np.maximum(max_coverage, np.where(clear, coverage, 0))
    return headings, support_headings, max_coverage


def assess_footprint(nodes, points, origin, shape, cfg):
    """Check obstacles relative to each local support surface, not a global Z crop.

    Floor maps use a conservative circumscribed footprint for arbitrary heading.
    Stairs remain conditional: a planar footprint does not certify stair gait.
    """
    rc, tc = cfg['robot'], cfg['tomography']
    res = tc['resolution_m']
    radius = math.hypot(rc['length_m']/2, rc['width_m']/2)+rc['horizontal_margin_m']
    oriented = rc.get('footprint_mode') == 'oriented_rectangle'
    half_width = rc['width_m']/2 + rc['horizontal_margin_m']
    min_gap = rc['standing_height_m']+rc['vertical_margin_m']
    cells = np.floor((points[:, :2]-origin)/res).astype(int)
    flat_ids = cells[:, 0]*shape[1]+cells[:, 1]
    xx, yy = np.meshgrid(origin[0]+(np.arange(shape[0])+.5)*res, origin[1]+(np.arange(shape[1])+.5)*res, indexing='ij')
    r = int(math.ceil(radius/res)); dx, dy = np.mgrid[-r:r+1, -r:r+1]
    kernel = ((dx*res)**2+(dy*res)**2 <= radius**2).astype(np.float32)
    coverage_all = np.zeros(len(nodes['xyz']), np.float32)
    clearance_all = np.zeros(len(nodes['xyz']), np.float32)
    nodes['heading_mask'] = np.zeros(len(nodes['xyz']), np.uint32)
    nodes['observed_heading_mask'] = np.zeros(len(nodes['xyz']), np.uint32)
    nodes['heading_checked'] = np.zeros(len(nodes['xyz']), bool)
    for region, floor in enumerate(tc['floors']):
        ground_ref = plane_height(floor['plane'], xx, yy)
        relative = points[:, 2] - ground_ref[cells[:, 0], cells[:, 1]]
        # Exclude small floor measurement noise, not furniture/walls/rails.
        obstacle_points = (relative >= 0.14) & (relative <= min_gap+0.10)
        obstacle = np.zeros(shape, bool)
        obstacle.ravel()[flat_ids[obstacle_points]] = True
        observed = np.zeros(shape, bool)
        ground_points = np.abs(relative) <= floor['tolerance_m']
        observed.ravel()[flat_ids[ground_points]] = True
        distance = ndimage.distance_transform_edt(~obstacle)*res
        if oriented:
            heading_mask, observed_heading_mask, coverage = rectangular_heading_masks(obstacle, observed, cfg)
        else:
            coverage = ndimage.convolve(observed.astype(np.float32), kernel, mode='constant')/kernel.sum()
        # Distance-to-cell must account for a finite obstacle cell, not just centre.
        clearance = np.maximum(0, distance-res/math.sqrt(2))
        mask = nodes['region'] == region
        ids = np.flatnonzero(mask); x, y = nodes['cell'][ids].T
        coverage_all[ids] = coverage[x, y]; clearance_all[ids] = clearance[x, y]
        if oriented:
            nodes['heading_mask'][ids] = heading_mask[x, y]
            nodes['observed_heading_mask'][ids] = observed_heading_mask[x, y]
            nodes['heading_checked'][ids] = True
            collide = heading_mask[x, y] == 0
        else:
            collide = clearance[x, y] < radius
        nodes['state'][ids[collide]] = BLOCKED
        unverified = (observed_heading_mask[x, y] == 0) if oriented else coverage[x, y] < rc['minimum_observed_support_ratio']
        nodes['state'][ids[unverified & ~collide & (nodes['state'][ids] == FREE)]] = CONDITIONAL
        valid = nodes['state'][ids] >= CONDITIONAL
        inflation_radius = half_width if oriented else radius
        nodes['cost'][ids[valid]] += 5*np.maximum(0, (inflation_radius+.5-clearance[x[valid], y[valid]])/.5)
    # Stair candidate surfaces get their measured clearance, but no green label.
    stair = nodes['region'] == 2
    nodes['state'][stair & (nodes['state'] == FREE)] = CONDITIONAL
    nodes['cost'][nodes['state'] < CONDITIONAL] = np.inf
    nodes['footprint_clearance'] = clearance_all
    nodes['observed_support_ratio'] = coverage_all
    return radius


def final_graph_report(nodes, edges):
    active = np.flatnonzero(nodes['state'] >= CONDITIONAL)
    remap = np.full(len(nodes['xyz']), -1, np.int32); remap[active] = np.arange(len(active))
    pair = remap[edges]
    graph = sparse.coo_matrix((np.ones(len(pair)*2, np.uint8),
                              (np.r_[pair[:, 0], pair[:, 1]], np.r_[pair[:, 1], pair[:, 0]])),
                             shape=(len(active), len(active))).tocsr()
    count, labels = connected_components(graph, directed=False)
    both = []
    for comp in range(count):
        regions = set(nodes['region'][active[labels == comp]].tolist())
        if {0, 1}.issubset(regions): both.append(int(comp))
    nodes['final_component'] = np.full(len(nodes['xyz']), -1, np.int32)
    nodes['final_component'][active] = labels
    return {'final_components':int(count), 'final_cross_floor_components':both,
            'stair_passage_verified':False, 'automatically_executable':False}


def write_cloud(path, xyz, colors):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(np.asarray(colors, float)/255)
    if not o3d.io.write_point_cloud(str(path), cloud, write_ascii=False, compressed=False):
        raise RuntimeError(f'Failed to write {path}')


def export_plots(out, nodes, poses, cfg):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    plt.style.use('dark_background')
    fig, axes = plt.subplots(1, 2, figsize=(13, 13), facecolor='#101722')
    for region, ax in enumerate(axes):
        ids = np.flatnonzero((nodes['region'] == region) | (nodes['region'] == 2))
        p = nodes['xyz'][ids]
        ax.scatter(p[:, 0], p[:, 1], c=COLORS[nodes['state'][ids]]/255, marker='s', s=5, linewidths=0)
        ax.set_aspect('equal'); ax.set_title(['LOWER FLOOR', 'UPPER FLOOR'][region], fontsize=17, pad=16)
        ax.set_xlim(-12, 23); ax.set_ylim(-7, 53); ax.set_xlabel('X / m'); ax.set_ylabel('Y / m')
        ax.set_facecolor('#101722'); ax.grid(alpha=.08)
    legend = [Patch(color=COLORS[i]/255, label=text) for i, text in [(3, 'Geometric clearance'), (2, 'Conditional / stairs'), (1, 'Blocked / safety margin'), (0, 'Unverified')]]
    fig.legend(handles=legend, loc='lower center', ncol=4, frameon=False, fontsize=11)
    fig.suptitle(f'SC-PGO 09-04 14:34 | D1 Max | {cfg["tomography"]["resolution_m"]:.2f} m cells', fontsize=18)
    fig.subplots_adjust(bottom=.08, top=.92, wspace=.18)
    fig.savefig(out/'two_floors_overview.png', dpi=160, facecolor=fig.get_facecolor()); plt.close(fig)


def export_outputs(out, cfg_path, cfg, nodes, arrays, edges, origin, shape, source_cloud, poses, stats):
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy2(cfg_path, out/'pipeline.yaml')
    np.savez_compressed(out/'traversability.npz', **nodes, candidate_edges=edges, origin_xy=origin, resolution=cfg['tomography']['resolution_m'], shape_xy=shape,
                        status_codes=np.array(['unknown', 'blocked', 'conditional', 'free']), frame=np.array(cfg['source']['frame']),
                        heading_angles_rad=np.linspace(0,np.pi,cfg['robot'].get('heading_bins_half_turn',1),endpoint=False),
                        footprint_mode=np.array(cfg['robot'].get('footprint_mode','circumscribed_circle')))
    # Full multilevel maps are retained for inspection; these are not Nav2 OccupancyGrid.
    # Preliminary slice costs precede footprint checking. Only traversability.npz
    # contains final assessment; do not expose preliminary costs as final plans.
    np.savez_compressed(out/'height_layers.npz', ground=arrays['ground'], ceiling=arrays['ceiling'], origin_xy=origin,
                        resolution=cfg['tomography']['resolution_m'], slice_spacing=cfg['tomography']['slice_spacing_m'],
                        slice_h0=cfg['tomography']['z_min_m'])
    for ext in ['ply', 'pcd']:
        write_cloud(out/f'traversability.{ext}', nodes['xyz'], COLORS[nodes['state']])
    viewer_sets = []
    for region, name in enumerate(['lower', 'upper', 'stairs']):
        mask = nodes['region'] == region
        write_cloud(out/f'{name}.ply', nodes['xyz'][mask], COLORS[nodes['state'][mask]])
        xyz = nodes['xyz'][mask]
        # Interleaved float32 XYZ + RGB(0..1), compact and easy to inspect.
        packed = np.column_stack((xyz, COLORS[nodes['state'][mask]].astype(float)/255)).astype('<f4')
        packed.tofile(out/f'{name}.bin')
        viewer_sets.append({'id':name, 'label':['下层','上层','楼梯'][region], 'file':f'{name}.bin', 'points':int(mask.sum()),
                            'bounds':[xyz.min(0).tolist(), xyz.max(0).tolist()] if len(xyz) else None,
                            'counts':{label:int(np.sum(nodes['state'][mask]==i)) for i,label in enumerate(['unknown','blocked','conditional','free'])}})
    background = source_cloud.voxel_down_sample(cfg['output']['source_preview_voxel_m'])
    xyz = np.asarray(background.points)
    np.column_stack((xyz, np.full(xyz.shape, .39))).astype('<f4').tofile(out/'source.bin')
    poses.astype('<f4').tofile(out/'recorded_trajectory.bin')
    stats.update(layers=viewer_sets, source_preview_points=len(xyz), recorded_trajectory_points=len(poses), source=cfg['source'],
                 robot=cfg['robot'], resolution_m=cfg['tomography']['resolution_m'], source_unchanged=True,
                 pipeline_file='pipeline.yaml', statuses={str(i):label for i,label in enumerate(['未确认','禁止/安全距离','条件通行/楼梯待验证','几何通行'])},
                 warnings=['几何候选地图，不是已验证的自主导航安全地图。', '绿色表示存在满足包络的朝向，不代表任何朝向都能通过或可以原地转身。' if cfg['robot'].get('footprint_mode') == 'oriented_rectangle' else '绿色使用全朝向外接圆包络，窄通道可能被保守排除。',
                           '本次 SC-PGO 日志接受回环为 0，原始地图倾斜和漂移未被此处理修正。',
                           '楼梯保持条件通行；未验证 D1 实机上下楼步态、摩擦、载荷及俯仰包络。', '灰色区域和没有支撑观测的区域不作为自由空间；未进行补洞。',
                           '上下层是相对高度名称，实际楼号未假定。'])
    (out/'report.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf8')
    export_plots(out, nodes, poses, cfg)
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    start = time.perf_counter()
    cfg_path = args.config.resolve(strict=True)
    cfg = yaml.safe_load(cfg_path.read_text())
    if cfg['validation']['allow_unknown_as_free'] or cfg['validation']['allow_hole_filling']:
        raise ValueError('This assessment does not permit unknown-as-free or synthetic hole filling')
    out = (args.output or cfg_path.parent/cfg['output']['directory']).resolve()
    source, cloud, points, poses, cropped, support_normals = load_input(cfg)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    log(f'Input {cfg["source"]["expected_points"]}; cropped {cropped}; voxel {len(points)}')
    nodes, arrays, origin, shape = build_surfaces(points, cfg, support_normals)
    edges, graph_report = candidate_graph(nodes, poses, cfg)
    radius = assess_footprint(nodes, points, origin, shape, cfg)
    edge_ok = (nodes['state'][edges] >= CONDITIONAL).all(1)
    if cfg['robot'].get('footprint_mode') == 'oriented_rectangle':
        both_checked = nodes['heading_checked'][edges].all(1)
        shared_heading = (nodes['heading_mask'][edges[:,0]] & nodes['heading_mask'][edges[:,1]]) != 0
        edge_ok &= ~both_checked | shared_heading
    edges = edges[edge_ok]
    graph_report.update(final_graph_report(nodes, edges))
    auto = (nodes['state'] == FREE)
    stats = dict(name=cfg['name'], source_sha256=digest, input_points=cfg['source']['expected_points'], retained_source_points=cropped,
                 voxel_points=len(points), nodes=len(nodes['xyz']), candidate_edges=len(edges), conservative_body_radius_m=radius,
                 footprint_mode=cfg['robot'].get('footprint_mode','circumscribed_circle'),
                 nominal_passage_width_m=cfg['robot']['width_m']+2*cfg['robot']['horizontal_margin_m'],
                 geometric_green_area_m2=float(auto.sum()*cfg['tomography']['resolution_m']**2), graph=graph_report,
                 elapsed_seconds=round(time.perf_counter()-start, 2), generated_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    stats = export_outputs(out, cfg_path, cfg, nodes, arrays, edges, origin, shape, cloud, poses, stats)
    if digest != hashlib.sha256(source.read_bytes()).hexdigest():
        raise RuntimeError('Source changed during processing')
    log(json.dumps({k:v for k,v in stats.items() if k not in ['source','robot','statuses']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
