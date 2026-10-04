"""Original-coordinate geometry for one floor named by its map package. No estimated inverse or TF."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial import cKDTree

from .source_route import SOURCE_FRAME, SourceRouteBuilder, SourceSupport, digest_file


def _floor_name(value):
    return isinstance(value,str) and 0<len(value)<=64 and value.isidentifier()


class SourceIdentityBridge:
    source_frame = planning_frame = SOURCE_FRAME
    projection_kind = 'source_identity'
    limits = None

    @classmethod
    def from_artifacts(cls, path):
        path = Path(path).resolve(strict=True)
        manifest = json.loads(path.read_text())
        if (manifest.get('schema') != 'd1max.source_identity_floor/v1'
                or manifest.get('status') != 'complete' or manifest.get('xyz_modified') is not False
                or manifest.get('geometry_operation') != 'source_identity'
                or manifest.get('frame_id') != SOURCE_FRAME or not _floor_name(manifest.get('floor_id'))
                or manifest.get('source_coordinate_error_m') != 0.):
            raise ValueError('source_identity_manifest_invalid')
        for filename, digest in ((manifest['source_path'],manifest['source_sha256']),
                (path.parent/manifest['output_file'],manifest['output_sha256']),
                (path.parent/manifest['source_indices_file'],manifest['source_indices_sha256'])):
            if digest_file(filename) != digest:
                raise ValueError('source_identity_artifact_changed')
        with np.load(path.parent/manifest['source_indices_file'],allow_pickle=False) as archive:
            floor = manifest['floor_id']
            support = archive[floor+'_support_xyz'].copy()
            ids=archive[floor+'_support_source_indices'].copy()
            retained=archive['selected_source_indices'].copy()
        from .pointcloud_helpers.pcd_io import read_pcd
        source=read_pcd(manifest['source_path'])
        output=read_pcd(path.parent/manifest['output_file'])
        for indices in (ids,retained):
            if indices.ndim!=1 or indices.dtype.kind not in 'iu' or np.any(indices<0) or np.any(indices>=len(source.xyz)):
                raise ValueError('source_identity_original_indices_invalid')
        if (not np.array_equal(source.xyz[ids],support)
                or not np.array_equal(source.records[retained],output.records)):
            raise ValueError('source_identity_original_records_were_modified')
        result = cls(support,manifest['forbidden_stair_xy'],floor_id=floor)
        result.manifest, result.manifest_path = manifest,path
        return result

    def __init__(self, support, forbidden_xy, *, floor_id):
        if not _floor_name(floor_id):
            raise ValueError('source_identity_floor_id_invalid')
        self.floor_id = floor_id
        self.support = np.asarray(support,dtype=float)
        if (self.support.ndim != 2 or self.support.shape[1] != 3 or not len(self.support)
                or not np.isfinite(self.support).all()):
            raise ValueError('source_identity_support_invalid')
        self.tree = cKDTree(self.support[:,:2])
        self.forbidden = np.asarray(forbidden_xy,dtype=float)
        if self.forbidden.shape != (2,2) or np.any(self.forbidden[0]>=self.forbidden[1]):
            raise ValueError('source_identity_stair_exclusion_invalid')
        self.floors = {floor_id:self}
        self.reference_z_m = float(np.median(self.support[:,2]))  # UI hint only
        self.input_z_band = (float(self.support[:,2].min())-.15,float(self.support[:,2].max())+.15)
        self.protected_regions = [dict(min=[*self.forbidden[0],-10000.],max=[*self.forbidden[1],10000.])]

    def query(self, xy, limits=None):
        xy = np.asarray(xy,dtype=float).reshape(-1,2)
        if not np.isfinite(xy).all() or np.any(np.all((xy>=self.forbidden[0])&(xy<=self.forbidden[1]),axis=1)):
            raise ValueError('source_identity_stair_or_invalid_query')
        distance,ids = self.tree.query(xy,k=min(8,len(self.support)),workers=1)
        distance,ids = np.asarray(distance).reshape(len(xy),-1),np.asarray(ids).reshape(len(xy),-1)
        z = []
        for d,i in zip(distance,ids):
            selected = self.support[i[d<=.15],2]
            if not len(selected) or np.ptp(selected)>.17:
                raise ValueError('source_identity_ground_unobserved_or_ambiguous')
            z.append(float(np.median(selected)))
        return np.asarray(z),distance[:,0]

    def project_live_pose_to_ground(self, body, floor_id, *, body_height_interval_m):
        if floor_id != self.floor_id:
            raise ValueError('floor_not_in_map_package')
        body = np.asarray(body,dtype=float)
        z,d = self.query(body[:2].reshape(1,2))
        height = float(body[2]-z[0])
        if not body_height_interval_m[0] <= height <= body_height_interval_m[1]:
            raise ValueError('source_identity_body_height_outside_observed_support')
        point = np.r_[body[:2],z[0]]
        return SimpleNamespace(xyz=point,diagnostics=dict(source_ground_xyz=point.tolist(),
            estimated_body_height_m=height,support_distance_m=float(d[0]),projection_kind='source_identity'))

    def to_localization_ground(self, points, labels):
        points = np.asarray(points,dtype=float)
        if len(labels)!=len(points) or any(v!=self.floor_id for v in labels):
            raise ValueError('source_identity_floor_ownership_required')
        if np.any(np.all((points[:,:2]>=self.forbidden[0])&(points[:,:2]<=self.forbidden[1]),axis=1)):
            raise ValueError('source_identity_route_enters_stair_exclusion')
        return SimpleNamespace(xyz=points.copy(),diagnostics=dict(projection_kind='source_identity',
            coordinate_transfer_error_bound_m=0.,nonrigid=False))

    to_planning_ground = to_localization_ground


def identity_builder(manifest_path,tomogram,bridge):
    manifest = bridge.manifest
    if (tomogram.provenance.get('source_processing_manifest_sha256') != digest_file(manifest_path)
            or tomogram.provenance.get('original_source_sha256') != manifest['source_sha256']
            or tomogram.provenance.get('geometry_operation') != 'source_identity'):
        raise ValueError('source_identity_tomogram_binding_invalid')
    return SourceRouteBuilder(bridge=bridge,source_map_sha256=manifest['source_sha256'],
        conditioning_sha256=digest_file(manifest_path),tomogram_sha256=tomogram.sha256,
        support=SourceSupport({bridge.floor_id:bridge.support},manifest['source_sha256']),tomogram=tomogram)


def validate_package(map_directory, *, bridge=None):
    """Validate the exact map the graph will open, without ROS/native loading.

    Hash equality alone does not prevent a copied route from opening a different
    release's tomography or support manifest. Make those deployment edges local
    and explicit before preparing any node parameters; never repair them here.
    """
    import yaml
    from d1max_pct_planner.paths import expand_tree
    from d1max_pct_planner.tomogram_map import TomogramMap
    from d1max_pct_planner.singlefloor_route import load_config
    from .live_map_geometry import validate_map_binding
    directory = Path(map_directory).resolve(strict=True)
    manifest_path = directory/'manifest.json'
    bridge = bridge or SourceIdentityBridge.from_artifacts(manifest_path)
    raw = expand_tree(yaml.safe_load((directory/'route.yaml').read_text()))
    tomogram = TomogramMap(directory/'tomogram.npz',
        minimum_headroom_m=raw.get('minimum_headroom_m'),
        unknown_ceiling_policy=raw.get('unknown_ceiling_policy', 'allow_unobserved'),
        max_ground_step_m=raw['limits']['max_ground_step_m'])
    _, settings = load_config(directory/'route.yaml', tomogram)
    validate_map_binding(manifest_path, bridge, settings, source_frame=SOURCE_FRAME)
    if settings.get('floor_id') != bridge.floor_id:
        raise ValueError('source_identity_route_floor_mismatch')
    for key, expected in (('source_pcd', directory/bridge.manifest['output_file']),
                          ('source_processing_manifest', manifest_path)):
        if Path(tomogram.provenance.get(key, '')).resolve() != expected.resolve():
            raise ValueError('source_identity_package_references_foreign_artifact:'+key)
    identity_builder(manifest_path, tomogram, bridge)
    return dict(route=settings, tomogram=tomogram, bridge=bridge)
