"""File/array-only coarse relocalization, independent of ROS and navigation.

FPFH retrieves overlapping 3-D submaps; RANSAC proposes rigid transforms and
point-to-plane ICP refines them. A proposal is NOT a localization proof. The
live owner must subsequently obtain the existing matcher's fresh confirmations.
Native calls run in a disposable worker process, never in a ROS callback.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np


@dataclass(frozen=True)
class RegistrationConfig:
    backend: str = 'fpfh_ransac'
    voxel_m: float = .4
    fine_voxel_m: float = .2
    tile_xy_m: float = 36.
    tile_z_m: float = 6.
    stride_xy_m: float = 12.
    stride_z_m: float = 3.
    scan_radius_m: float = 12.
    min_points: int = 200
    max_scan_points: int = 60000
    max_map_points: int = 250000
    max_tiles: int = 256
    max_index_points: int = 1200000
    candidates: int = 8
    exhaustive_tiles: bool = True
    hypotheses_per_tile: int = 2
    ransac_iterations: int = 20000
    min_inlier_ratio: float = .60
    max_rmse_m: float = .20
    min_information_ratio: float = .0005
    ambiguity_margin: float = .06
    distinct_position_m: float = 1.
    distinct_rotation_rad: float = .25
    search_timeout_s: float = 25.
    index_timeout_s: float = 120.
    threads: int = 2

    def __post_init__(self):
        if self.backend != 'fpfh_ransac':
            raise ValueError('unknown_global_registration_backend')
        for key, value in asdict(self).items():
            if key == 'backend':
                continue
            if key == 'exhaustive_tiles':
                if type(value) is not bool:raise ValueError('exhaustive_tiles_must_be_bool')
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_global_registration_parameter:' + key)
        if not (.1 <= self.voxel_m <= 1. and .05 <= self.fine_voxel_m <= self.voxel_m
                and self.stride_xy_m <= self.tile_xy_m / 2
                and self.stride_z_m <= self.tile_z_m / 2
                and .4 <= self.min_inlier_ratio <= 1 and self.max_rmse_m <= .3
                and self.ambiguity_margin <= .3 and self.search_timeout_s <= 60
                and self.index_timeout_s <= 180 and self.threads <= 4
                and self.max_index_points <= 2000000 and self.max_map_points <= 500000
                and self.max_tiles <= 512 and self.candidates <= 16 and self.hypotheses_per_tile <= 4):
            raise ValueError('unbounded_global_registration_configuration')
        for key in ('min_points', 'max_scan_points', 'max_map_points', 'max_tiles',
                    'max_index_points', 'candidates', 'hypotheses_per_tile', 'ransac_iterations', 'threads'):
            if type(getattr(self, key)) is not int:
                raise ValueError('integer_required:' + key)


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def cloud(points, viewpoint=None):
    import open3d as o3d
    value = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, dtype=float)))
    if viewpoint is not None:
        value.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.2, max_nn=30))
        value.orient_normals_towards_camera_location(np.asarray(viewpoint, dtype=float))
    return value


def feature(points, viewpoint, voxel):
    import open3d as o3d
    value = cloud(points)
    value.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2.5 * voxel, max_nn=30))
    value.orient_normals_towards_camera_location(np.asarray(viewpoint, dtype=float))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        value, o3d.geometry.KDTreeSearchParamHybrid(radius=5 * voxel, max_nn=100))
    return value, np.asarray(fpfh.data).T.copy()


def normalized(features):
    return features / np.maximum(np.linalg.norm(features, axis=1, keepdims=True), 1.e-12)


def select_candidate(candidates, config):
    """Merge the same place, but never merge different floors or orientations."""
    clusters = []
    for item in sorted(candidates, key=lambda v: v['score'], reverse=True):
        transform = np.asarray(item['transform'])
        if not (transform.shape == (4, 4) and np.isfinite(transform).all()):
            continue
        duplicate = False
        for old in clusters:
            previous = np.asarray(old['transform'])
            angle = math.acos(np.clip((np.trace(transform[:3, :3] @ previous[:3, :3].T) - 1) / 2, -1, 1))
            if (np.linalg.norm(transform[:3, 3] - previous[:3, 3]) < config.distinct_position_m
                    and angle < config.distinct_rotation_rad):
                duplicate = True
                break
        if not duplicate:
            clusters.append(item)
    result = dict(accepted=False, reason='no_geometric_match', candidates=clusters[:16])
    if not clusters:
        return result
    best = clusters[0]
    if (best['inlier_ratio'] < config.min_inlier_ratio or best['rmse_m'] > config.max_rmse_m
            or best['information_ratio'] < config.min_information_ratio):
        result['reason'] = 'insufficient_geometric_evidence'
        return result
    gap = best['score'] - clusters[1]['score'] if len(clusters) > 1 else None
    result['score_margin'] = gap
    if gap is not None and gap < config.ambiguity_margin:
        result['reason'] = 'ambiguous_place_or_floor'
        return result
    result.update(accepted=True, reason='coarse_candidate_requires_confirmation', candidate=best)
    return result


class GlobalMapIndex:
    def __init__(self, map_path, cache_dir, config=RegistrationConfig()):
        import open3d as o3d
        self.config = config
        self.map_sha256 = file_sha256(map_path)
        self.config_sha256 = hashlib.sha256(json.dumps(asdict(config), sort_keys=True).encode()).hexdigest()
        self.cache_key = hashlib.sha256((self.map_sha256 + self.config_sha256 + o3d.__version__ + ':index-v2').encode()).hexdigest()
        cache = Path(cache_dir) / (self.cache_key + '.npz')
        self.cache_hit = False
        if cache.exists():
            try:
                with np.load(cache, allow_pickle=False) as data:
                    if str(data['key']) != self.cache_key:
                        raise ValueError('cache_identity_mismatch')
                    self.points = data['points'].copy()
                    self.tile_points = data['tile_points'].copy()
                    self.features = data['features'].copy()
                    self.offsets = data['offsets'].copy()
                    self.centers = data['centers'].copy()
                self.validate()
                self.cache_hit = True
            except (OSError, ValueError, KeyError):
                self.build(map_path)
        else:
            self.build(map_path)
        self.validate()
        if not self.cache_hit:
            cache.parent.mkdir(parents=True, exist_ok=True)
            temporary = cache.with_name(cache.name + f'.{os.getpid()}.tmp')
            with temporary.open('wb') as stream:
                np.savez_compressed(stream, key=self.cache_key, points=self.points,
                    tile_points=self.tile_points, features=self.features, offsets=self.offsets, centers=self.centers)
            os.replace(temporary, cache)
        from scipy.spatial import cKDTree
        self.map_tree = cKDTree(self.points)

    def validate(self):
        c = self.config
        if (not 0 < len(self.points) <= c.max_map_points or self.points.shape[1:] != (3,)
                or not 0 < len(self.centers) <= c.max_tiles or self.centers.shape[1:] != (3,)
                or self.offsets.shape != (len(self.centers) + 1,)
                or self.offsets[0] != 0 or np.any(np.diff(self.offsets) < c.min_points)
                or self.offsets[-1] != len(self.tile_points)
                or len(self.tile_points) > c.max_index_points
                or self.tile_points.shape[1:] != (3,) or self.features.shape != (len(self.tile_points), 33)
                or not all(np.isfinite(a).all() for a in (self.points, self.tile_points, self.features, self.centers))):
            raise ValueError('invalid_or_oversized_global_index')

    def build(self, map_path):
        import open3d as o3d
        c = self.config
        original = o3d.io.read_point_cloud(str(map_path), remove_nan_points=True, remove_infinite_points=True)
        if file_sha256(map_path)!=self.map_sha256:
            raise ValueError('localization_map_changed_while_loading')
        if not len(original.points):
            raise ValueError('empty_localization_map')
        # No height flattening, estimated floor, or fake map->odom transform.
        self.points = np.asarray(original.voxel_down_sample(c.fine_voxel_m).points).copy()
        if len(self.points) > c.max_map_points:
            raise ValueError('localization_map_exceeds_index_budget_adjust_voxel_explicitly')
        coarse = np.asarray(original.voxel_down_sample(c.voxel_m).points)
        low = coarse.min(axis=0)
        steps = np.array([c.stride_xy_m, c.stride_xy_m, c.stride_z_m])
        half = np.array([c.tile_xy_m / 2, c.tile_xy_m / 2, c.tile_z_m / 2])
        # Place windows over occupied core cells, not every empty cell inside
        # an outlier-expanded bounding box. Every source point has a core cell;
        # this is NOT truncating the map or labeling empty cells as occupied.
        core_cells = np.unique(np.floor((coarse - low) / steps).astype(np.int64), axis=0)
        if len(core_cells) > c.max_tiles:
            raise ValueError('map_extent_exceeds_bounded_tile_budget')
        points, features, centers, offsets = [], [], [], [0]
        for cell in core_cells:
            center = low + (cell + .5) * steps
            subset = coarse[np.all(np.abs(coarse - center) <= half, axis=1)]
            if len(subset) < c.min_points:
                continue  # Too little geometry for even one valid query.
            if offsets[-1] + len(subset) > c.max_index_points:
                raise ValueError('global_index_budget_exceeded_no_silent_map_truncation')
            _, descriptors = feature(subset, center, c.voxel_m)
            points.append(subset); features.append(descriptors); centers.append(center)
            offsets.append(offsets[-1] + len(subset))
        if not centers:
            raise ValueError('no_usable_global_submaps')
        self.tile_points = np.concatenate(points)
        self.features = np.concatenate(features)
        self.centers = np.asarray(centers)
        self.offsets = np.asarray(offsets, dtype=np.int64)

    def search(self, points, request_id='offline'):
        import open3d as o3d
        from scipy.spatial.distance import cdist
        started = time.monotonic()
        c = self.config
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) > c.max_scan_points:
            raise ValueError('invalid_or_oversized_query')
        points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) <= c.scan_radius_m)]
        source = cloud(points).voxel_down_sample(c.voxel_m)
        if len(source.points) < c.min_points:
            return dict(accepted=False, reason='insufficient_scan_geometry', request_id=request_id)
        source, descriptors = feature(np.asarray(source.points), (0, 0, 0), c.voxel_m)
        query = normalized(descriptors)[np.linspace(0, len(descriptors) - 1, min(96, len(descriptors)), dtype=int)]
        retrieval = []
        for i, (begin, end) in enumerate(zip(self.offsets[:-1], self.offsets[1:])):
            target = normalized(self.features[begin:end])
            sample = target[np.linspace(0, len(target) - 1, min(384, len(target)), dtype=int)]
            distance = cdist(query, sample).min(axis=1)
            retrieval.append((float(np.mean(distance)), i))
        # Sparse live scans have descriptor statistics unlike dense map crops.
        # Retrieval orders work, but cannot silently exclude another floor or
        # the true place. Startup admission defaults to ALL bounded submaps.
        shortlist = [i for _, i in sorted(retrieval)]
        if not c.exhaustive_tiles:shortlist = shortlist[:c.candidates]
        source_features = o3d.pipelines.registration.Feature()
        source_features.data = descriptors.T.copy()
        fine_source = cloud(points).voxel_down_sample(c.fine_voxel_m)
        proposals = [];coarse_diagnostics=[]
        for tile in shortlist:
            begin, end = self.offsets[tile:tile + 2]
            target = cloud(self.tile_points[begin:end], self.centers[tile])
            target_features = o3d.pipelines.registration.Feature()
            target_features.data = self.features[begin:end].T.copy()
            for trial in range(c.hypotheses_per_tile):
                if time.monotonic() - started >= c.search_timeout_s:
                    return dict(accepted=False, reason='global_search_deadline', request_id=request_id)
                coarse = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                    source, target, source_features, target_features, True, c.voxel_m * 1.5,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
                    [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(.9),
                     o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(c.voxel_m * 1.5)],
                    o3d.pipelines.registration.RANSACConvergenceCriteria(c.ransac_iterations, .999),
                    seed=tile * 17 + trial)
                # Open3D 0.14's feature RANSAC fitness is correspondence
                # consensus, NOT source-cloud overlap. Sparse scan vs dense
                # map can have few feature matches yet a good rigid proposal.
                # Evaluate geometric overlap separately before ICP; final
                # admission thresholds below remain unchanged.
                geometry = o3d.pipelines.registration.evaluate_registration(source,target,.8,coarse.transformation)
                coarse_diagnostics.append(dict(tile=tile,trial=trial,
                    feature_consensus=float(coarse.fitness),geometric_overlap=float(geometry.fitness)))
                if len(coarse.correspondence_set)<3 or geometry.fitness < .25:
                    continue
                # Refine against ORIGINAL map XYZ, not a truncated retrieval tile.
                mapped = np.asarray(fine_source.points) @ coarse.transformation[:3, :3].T + coarse.transformation[:3, 3]
                low, high = mapped.min(axis=0) - 1., mapped.max(axis=0) + 1.
                local = self.points[np.all((self.points >= low) & (self.points <= high), axis=1)]
                if len(local) < c.min_points:
                    continue
                fine_target = cloud(local, coarse.transformation[:3, 3])
                transform = coarse.transformation.copy()
                for distance in (.8, .35):
                    refined = o3d.pipelines.registration.registration_icp(fine_source, fine_target, distance, transform,
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
                    transform = refined.transformation
                transformed = np.asarray(fine_source.points) @ transform[:3, :3].T + transform[:3, 3]
                distance, indices = self.map_tree.query(transformed, workers=c.threads)
                mask = distance <= .35
                ratio = float(mask.mean())
                rmse = float(np.sqrt(np.mean(distance[mask] ** 2))) if mask.any() else 1.e6
                # Scale-free rank check; one plane cannot establish a 6-D pose.
                if mask.sum() >= c.min_points:
                    normals = np.asarray(fine_target.normals)
                    from scipy.spatial import cKDTree
                    _, normal_indices = cKDTree(local).query(transformed[mask], workers=c.threads)
                    n = normals[normal_indices]
                    q = transformed[mask] - transform[:3, 3]
                    q /= max(1., float(np.sqrt(np.mean(np.sum(q * q, axis=1)))))
                    jacobian = np.concatenate([n, np.cross(q, n)], axis=1)
                    eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian / len(jacobian))
                    information = float(max(0., eigenvalues[0]) / max(1.e-12, eigenvalues[-1]))
                else:
                    information = 0.
                proposals.append(dict(tile=tile, trial=trial, transform=transform.tolist(),
                    inlier_ratio=ratio, rmse_m=rmse, information_ratio=information,
                    score=ratio * math.exp(-rmse / .2)))
        result = select_candidate(proposals, c)
        result.update(request_id=request_id, map_sha256=self.map_sha256, config_sha256=self.config_sha256,
            elapsed_s=time.monotonic() - started, evaluated_tiles=shortlist, indexed_tiles=len(self.centers),
            retrieval_is_heuristic=True, global_uniqueness_proven=False,
            exhaustive_submap_search=c.exhaustive_tiles,coarse_diagnostics=coarse_diagnostics)
        if not c.exhaustive_tiles:result.update(accepted=False,reason='subset_search_not_initialization_eligible')
        if result['elapsed_s'] > c.search_timeout_s:
            result.update(accepted=False, reason='global_search_deadline')
        return result
