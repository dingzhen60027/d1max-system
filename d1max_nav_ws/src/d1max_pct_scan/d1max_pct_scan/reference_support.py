"""Literal original-map returns for reference preparation, never authority.

Conditioned PCT points and ground-bridge estimates are not measured returns.
Reuse the production SourceRouteBuilder's hashed source/evidence selection and
keep its floor/stair ownership; only the measured map-to-odom anchor may later
transform these points. Loading support does not certify a route or a gait.
"""
import json
import hashlib
from collections import OrderedDict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .source_identity import SourceIdentityBridge
from .source_route import SourceRouteBuilder, digest_file


class ConditionedReferenceSupport:
    def __init__(self, manifest, source_support, tomogram=None, *, structured_source_points=None,
                 stair_model=None, stair_model_sha256=''):
        self.manifest = manifest
        self.source_support = source_support
        if stair_model is not None and (
                stair_model.frame_id != 'd1max_loc_map'
                or stair_model.record['source_map_sha256'] != source_support.source_hash
                or len(stair_model_sha256) != 64
                or any(c not in '0123456789abcdef' for c in stair_model_sha256)):
            raise ValueError('reference_stair_model_identity_mismatch')
        # An explicit planning model is not a row in any observed-point tree,
        # not contact qualification, and not raw original-riser evidence.
        self.stair_model, self.stair_model_sha256 = stair_model, stair_model_sha256
        self.points, self.xy_trees = {}, {}
        for label, tree in source_support.trees.items():
            # These are copies of literal source PCD rows selected by the
            # builder's verified floor evidence, never conditioned-map XYZ.
            points = tree.data
            points.flags.writeable = False
            self.points[label] = points
            self.xy_trees[label] = cKDTree(points[:, :2], copy_data=True)
        self.stair_index = None
        self.stair_contact_source = frozenset()
        if tomogram is not None and 'stairs' in self.points:
            from .stair_support import StairSupportIndex
            self.stair_index = StairSupportIndex(self.points['stairs'], tomogram,
                support_radius_m=source_support.radius,
                stair_mapping=source_support.stair_mapping,
                structured_source_points=structured_source_points)
            if structured_source_points is not None:
                # Keep the stronger, package-checked contact classification
                # separate from the legacy PCT-valid support union. Membership
                # is literal source XYZ, never a nearest point or fitted grid.
                self.stair_contact_source = (frozenset(map(tuple, structured_source_points)) &
                    frozenset(map(tuple, self.stair_index.points)))

    def contact_indices(self, source, *, source_map_sha256, floor_id, segment_kind,
                        required_mode, regions=()):
        """Index the FINAL selected raw rows before their one rigid transform.

        Transition ownership is resolved before this final selection. Qualify
        only stair-owned literal rows, never ordinary-floor context.
        Platform points receive no inferred qualification: they must themselves
        belong to the independent literal qualified set.
        """
        if (not self.stair_contact_source or source_map_sha256 != self.source_support.source_hash
                or source_map_sha256 != self.manifest['source_sha256']):
            return []
        source = np.asarray(source, dtype=float).reshape((-1, 3))
        allowed = np.zeros(len(source), dtype=bool)
        def stair_region(floor, kind, mode):
            return (mode == 'stair' and floor in ('stairs', 'stair_lower', 'stair_upper')
                    and kind in ('stair_up', 'stair_down', 'landing'))
        if regions:
            for region in regions:
                if stair_region(region['floor_id'], region['segment_kind'], region['required_mode']):
                    allowed[region['begin_index']:region['end_index']] = True
        elif stair_region(floor_id, segment_kind, required_mode):
            allowed[:] = True
        return [i for i, point in enumerate(source)
                if allowed[i] and tuple(point) in self.stair_contact_source]

    def nearby(self, center, radius, floor_id):
        if floor_id not in ('floor1', 'floor2', 'stairs', 'stair_lower', 'stair_upper'):
            raise ValueError('reference_support_unknown_floor')
        tree = self.xy_trees.get(floor_id)
        if tree is None:
            return np.empty((0, 3), dtype=float)
        return self.points[floor_id][tree.query_ball_point(center, radius)]

    def classified_ground(self, *, source_map_sha256, floor_id, segment_kind, required_mode):
        # Floor evidence classifies these literal rows as observed floor
        # support. Protected stair returns have no equivalent classification.
        # This says nothing about route eligibility, collision or locomotion.
        return (source_map_sha256 == self.source_support.source_hash
            and source_map_sha256 == self.manifest['source_sha256']
            and floor_id in ('floor1', 'floor2') and floor_id in self.points
            and segment_kind in ('floor', 'landing') and required_mode == 'general')

    def stair_surface(self, ground_reference, *, source_map_sha256, floor_id,
                      segment_kind, required_mode):
        from .stair_support import StairSupportSelection
        if (source_map_sha256 != self.source_support.source_hash
                or source_map_sha256 != self.manifest['source_sha256']
                or floor_id not in ('stairs', 'stair_lower', 'stair_upper')
                or segment_kind not in ('stair_up', 'stair_down', 'landing')
                or required_mode != 'stair' or self.stair_index is None):
            return StairSupportSelection(np.empty((0, 3)), False,
                                         'source_support_not_classified_for_segment')
        # Native stair legs include the approach/exit on their adjacent floor.
        # Reuse that floor's ALREADY classified original rows and XYZ tree;
        # do not compare the floor's non-rigidly corrected PCT Z with raw Z.
        floor={'stair_lower':'floor1','stair_upper':'floor2'}.get(floor_id)
        return self.stair_index.select(ground_reference,
            classified_floor_tree=self.source_support.trees.get(floor))


def load_reference_support(manifest_path, *, expected_source_map_sha256,
                           expected_conditioning_sha256, expected_tomogram_sha256):
    """Use the original identity loader, or audited multi-floor raw support."""
    path = Path(manifest_path).resolve(strict=True)
    manifest = json.loads(path.read_text())
    if manifest.get('geometry_operation') == 'source_identity':
        # Preserve the established original-record validation and point index.
        result = SourceIdentityBridge.from_artifacts(path)
    else:
        if digest_file(path) != expected_conditioning_sha256:
            raise ValueError('reference_support_conditioning_identity_mismatch')
        from .navigation_map_package import validate_package
        package = validate_package(path.parent, purpose='planning_only')
        if package['tomogram'].sha256 != expected_tomogram_sha256:
            raise ValueError('reference_support_tomogram_identity_mismatch')
        builder = SourceRouteBuilder.from_artifacts(path, package['tomogram'], package['bridge'])
        result = ConditionedReferenceSupport(package['manifest'], builder.support, package['tomogram'],
            structured_source_points=package.get('structured_stair_source_points'),
            stair_model=package.get('stair_model'), stair_model_sha256=package.get('stair_model_sha256', ''))
    if result.manifest['source_sha256'] != expected_source_map_sha256:
        raise ValueError('reference_support_map_identity_mismatch')
    return result


def parametric_stair_packet(provider, *, source_hash, segment, contexts, inverse):
    """Transform model poses once; local parameters are unchanged observations-free geometry.

    The explicit segment associations are asset metadata. They do not reuse
    literal support owners or infer a stair from the SDK's operating mode.
    This channel is independent of the success of nearby-point selection.
    """
    if not isinstance(provider, ConditionedReferenceSupport) or provider.stair_model is None:
        return '', [], [], None
    model = provider.stair_model
    if (source_hash != provider.source_support.source_hash
            or source_hash != provider.manifest['source_sha256']
            or source_hash != model.record['source_map_sha256']):
        raise ValueError('reference_stair_model_source_mismatch')
    owners = [c for c in (contexts or (segment,)) if c[3] == 'stair'
              and c[2] in ('stair_up', 'stair_down', 'landing')]
    selected = [(surface, next((c[0] for c in owners if c[0] in surface['segment_ids']), None))
                for surface in model.surfaces]
    selected = [(surface, sid) for surface, sid in selected if sid is not None]
    if not selected:
        return '', [], [], None
    from d1max_planning_interfaces.msg import StairSurfaceModel, StairBoundaryModel
    from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
    from scipy.spatial.transform import Rotation
    from .control_frame_contract import Rigid
    def pose(position, quaternion):
        transformed = inverse.compose(Rigid(tuple(map(float, position)), tuple(map(float, quaternion))))
        return Pose(position=Point(x=transformed.xyz[0], y=transformed.xyz[1], z=transformed.xyz[2]),
            orientation=Quaternion(x=transformed.xyzw[0], y=transformed.xyzw[1],
                                   z=transformed.xyzw[2], w=transformed.xyzw[3]))
    surfaces = []
    source_geometry = []
    for surface, sid in selected:
        frame = surface['frame']
        rotation = Rotation.from_matrix(np.column_stack(
            [frame[k] for k in ('axis_u', 'axis_v', 'axis_up')])).as_quat()
        surfaces.append(StairSurfaceModel(surface_id=surface['surface_id'], segment_id=sid,
            kind=surface['kind'], pose=pose(frame['origin_xyz'], rotation),
            polygon=[Point(x=float(p[0]), y=float(p[1]), z=0.) for p in surface['polygon_xy']],
            support_grade=float(surface['support_grade']),
            support_intercept_m=float(surface['support_intercept_m']),
            contact_tolerance_m=float(surface['contact_tolerance_m']),
            height_association_m=float(surface['ground_replacement_band_m']),
            tread_stations_m=list(map(float, surface['tread_stations_m'])),
            tread_heights_m=list(map(float, surface['tread_heights_m']))))
        source_geometry.append(dict(surface_id=surface['surface_id'], segment_id=sid))
    obstacles = [StairBoundaryModel(structure_id=box['structure_id'],
        pose=pose(box['pose']['position'], box['pose']['orientation_xyzw']),
        half_extents=Vector3(x=float(box['half_extents_xyz'][0]), y=float(box['half_extents_xyz'][1]),
                            z=float(box['half_extents_xyz'][2]))) for box in model.obstacles]
    identity = dict(model_sha256=provider.stair_model_sha256, surfaces=source_geometry,
                    obstacles=[b['structure_id'] for b in model.obstacles],
                    model_points_are_observations=False)
    return provider.stair_model_sha256, surfaces, obstacles, identity


def nearby_source_returns(provider, center, radius, floor_id):
    if isinstance(provider, SourceIdentityBridge):
        # The existing single-floor radius/order/selection are unchanged.
        return provider.support[provider.tree.query_ball_point(center, radius)]
    return provider.nearby(center, radius, floor_id)


def _polyline_distance_squared(points, path):
    """Exact 3D segment distances with a bounded midpoint-tree search.

    Full immutable route segments, not the moving window, define ownership.
    The initial segment gives an upper bound; any better segment's midpoint
    must be within that distance plus the maximum segment half-length.
    """
    path=np.asarray(path,dtype=float).reshape((-1,3))
    if len(path)<2 or not np.isfinite(path).all():
        raise ValueError('transition_source_owner_geometry_invalid')
    starts=path[:-1];delta=np.diff(path,axis=0)
    squared=np.einsum('ij,ij->i',delta,delta)
    tree=cKDTree((starts+path[1:])*.5)
    nearest=tree.query(points,workers=1)[1]
    def distances(rows,edges):
        offset=points[rows]-starts[edges]
        ratio=np.divide(np.einsum('ij,ij->i',offset,delta[edges]),squared[edges],
                        out=np.zeros(len(edges)),where=squared[edges]>0.)
        residual=offset-np.clip(ratio,0.,1.)[:,None]*delta[edges]
        return np.einsum('ij,ij->i',residual,residual)
    result=distances(np.arange(len(points)),nearest)
    radius=np.sqrt(result)+.5*np.sqrt(squared.max(initial=0.))+1e-12
    # Bound temporary arrays even for overlapping/degenerate long routes.
    for begin in range(0,len(points),128):
        neighbors=tree.query_ball_point(points[begin:begin+128],radius[begin:begin+128],workers=1)
        counts=np.asarray([len(v) for v in neighbors])
        rows=np.repeat(np.arange(begin,begin+len(neighbors)),counts)
        edges=np.concatenate(neighbors).astype(np.int64)
        np.minimum.at(result,rows,distances(rows,edges))
    return result


def _unique_transition_owners(provider, sources, contexts, geometries, *, original_tree=None):
    """One original return, one terrain owner, before per-region quotas.

    Selection neighborhoods are context, not ownership domains. Reusing their
    labels made the same original XYZ both flights and disabled native contact
    matching. Partition the unchanged union against complete segment geometry.
    Eligibility still comes from the independently classified source indexes;
    geometry proximity cannot relabel a wall, unknown point or another floor.
    """
    points=np.unique(np.concatenate(sources,axis=0),axis=0)
    if not len(points):return [points.copy() for _ in contexts]
    represented=contexts
    identity=contexts[0][8] if len(contexts[0])>8 else ()
    # Formal callbacks freeze the complete route ownership domain once. The
    # contexts-only form remains for legacy direct callers without that domain;
    # it cannot promise stability across changes of their supplied domain.
    if len(contexts[0])>7:
        contexts=contexts[0][7]
        geometries=[context[6] for context in contexts]
    all_points=points
    keys=list(map(tuple,points))
    # A map provider has one preparation worker (not a shared thread cache).
    # Content-address the whole frozen domain, task/route identity and original
    # map, not numpy addresses: views can share a mutable base and copied routes
    # can have identical content. Reanchor does not alter physical ownership.
    immutable=all(not np.asarray(path).flags.writeable for path in geometries)
    digest=hashlib.sha256()
    digest.update(str(provider.source_support.source_hash).encode())
    digest.update(b'original-riser-candidates' if original_tree is not None else b'classified-ground')
    if original_tree is not None:digest.update(provider.source_support.raw_stair_evidence_sha256.encode())
    digest.update(json.dumps(identity,separators=(',',':')).encode())
    for context,path in zip(contexts,geometries):
        digest.update(json.dumps(context[:6],separators=(',',':')).encode())
        digest.update(np.asarray(path,dtype='<f8').tobytes())
    signature=digest.digest()
    cache_name='_riser_owner_cache' if original_tree is not None else '_transition_owner_cache'
    cache=getattr(provider,cache_name,None)
    if not immutable or not identity:cache=None
    elif cache is None or cache[0]!=signature:
        cache=(signature,OrderedDict())
        setattr(provider,cache_name,cache)
    known={} if cache is None else cache[1]
    missing=[i for i,key in enumerate(keys) if key not in known]
    # Per-call results may exceed the cache cap; they never expand its bound.
    labels=[known.get(key) for key in keys]
    points=points[missing]
    best=np.full(len(points),np.inf);owners=np.full(len(points),-1,dtype=int)
    # Physical floor/landing wins an exact portal tie. The final lexical tie
    # is deterministic under route reversal, not whichever window came first.
    order=sorted(range(len(contexts)),key=lambda i:(contexts[i][3]!='general',
        contexts[i][2]!='landing',contexts[i][1],contexts[i][0]))
    for number in order:
        _,floor,kind,mode=contexts[number][:4]
        trees=[]
        if original_tree is not None:
            # Raw original observations have NO terrain qualification. All
            # frozen physical segments participate only in owner assignment.
            trees.append(original_tree)
        elif mode=='stair' and provider.stair_index is not None:
            trees.append(provider.stair_index.tree)
            floor={'stair_lower':'floor1','stair_upper':'floor2'}.get(floor)
        if original_tree is None and floor is not None:trees.append(provider.source_support.trees.get(floor))
        allowed=np.zeros(len(points),dtype=bool)
        for tree in trees:
            if tree is not None:allowed|=tree.query(points,workers=1)[0]==0.
        distance=np.full(len(points),np.inf)
        if allowed.any():distance[allowed]=_polyline_distance_squared(points[allowed],geometries[number])
        better=allowed & (distance<best-1e-12)
        best[better]=distance[better];owners[better]=number
    for index,owner in zip(missing,owners):
        labels[index]=contexts[owner][0] if owner>=0 else None
    if cache is not None:
        for key,label in zip(keys,labels):
            known[key]=label
            known.move_to_end(key)
            if len(known)>20000:known.popitem(last=False)
    # Off-window context remains in the static map; do not relabel it as the
    # current segment merely to keep the publication cloud large.
    labels=np.asarray(labels,dtype=object)
    return [all_points[labels==context[0]] for context in represented]


def owned_stair_source_returns(provider, source, context, domain, identity=()):
    """Clip single-stair context to the same owners used by mixed windows."""
    if not domain or len(domain)==1:return source
    full=next(item for item in domain if item[0]==context[0])
    return _unique_transition_owners(provider,[source],[(*full,domain,identity)],[full[6]])[0]


def original_stair_riser_candidates(provider, ground, *, source_hash, contexts=(),
                                    context=(), domain=(), identity=()):
    """Bounded original-map observations, NOT ground or riser qualification.

    Return independent literal points with final packet-region owner indices.
    Native matching must still prove the vertical surface against adjacent
    qualified treads. Walls/railings in this raw set gain no permission here.
    """
    empty=(np.empty((0,3),dtype=float),[])
    if not isinstance(provider,ConditionedReferenceSupport):return empty
    tree=getattr(provider.source_support,'raw_stair_evidence',None)
    if (tree is None or source_hash!=provider.source_support.source_hash
            or source_hash!=provider.manifest['source_sha256']):return empty
    if contexts:
        represented=tuple(contexts)
        if len(represented[0])<=8:return empty
        domain=represented[0][7];identity=represented[0][8]
    else:
        if not domain or not identity:return empty
        full=next((c for c in domain if c[0]==context[0]),None)
        if full is None:return empty
        represented=((*full,domain,identity),)
    eligible=[i for i,c in enumerate(represented) if c[3]=='stair' and c[2] in ('stair_up','stair_down')]
    if not eligible:return empty
    ground=np.asarray(ground,dtype=float).reshape((-1,3))
    if len(ground)<2 or not np.isfinite(ground).all():return empty
    # One bounded neighborhood around the actual short reference, followed
    # by exact XYZ segment distance. Never gather the whole stair ROI per tick.
    center=(ground.min(axis=0)+ground.max(axis=0))*.5
    radius=float(np.linalg.norm(ground-center,axis=1).max())+1.5
    candidates=tree.data[tree.query_ball_point(center,radius)]
    if len(candidates):candidates=candidates[_polyline_distance_squared(candidates,ground)<=1.5**2]
    if not len(candidates):return empty
    parts=_unique_transition_owners(provider,[candidates],represented,
        [c[6] for c in represented],original_tree=tree)
    points=[];owners=[];quota=20000//len(eligible)
    for number in eligible:
        part=parts[number]
        if len(part)>quota:
            # Index-strided sampling tears holes in a densely observed face.
            # Keep the original returns nearest this short reference first,
            # using their stable source rows to break equal-distance ties.
            # This changes only over-quota sets; no new point or qualification
            # is created, and the existing per-owner/total bounds still apply.
            distance=_polyline_distance_squared(part,ground)
            original_index=tree.query(part,workers=1)[1]
            selected=np.lexsort((original_index,distance))[:quota]
            part=part[np.sort(selected)]
        points.append(part);owners.extend([number]*len(part))
    return (np.concatenate(points,axis=0),owners) if points else empty


def canonical_riser_points(provider, points):
    """Byte-identical canonical XYZ encoding with a bounded immutable-row LRU.

    Only strings are cached, never mutable ROS message objects. The provider
    is owned by the single preparation worker; source/lineage changes isolate
    the cache. Per-call order and signed-zero bytes remain exact.
    """
    source=provider.source_support
    identity=(source.source_hash,source.raw_stair_evidence_sha256)
    cache=getattr(provider,'_riser_encoding_cache',None)
    if cache is None or cache[0]!=identity:
        cache=(identity,OrderedDict());provider._riser_encoding_cache=cache
    known=cache[1];rows=[]
    for point in np.asarray(points,dtype='<f8'):
        key=point.tobytes()  # +0 and -0 have different canonical JSON bytes.
        encoded=known.get(key)
        if encoded is None:
            encoded=json.dumps(point.tolist(),separators=(',',':'),allow_nan=False)
            known[key]=encoded
        known.move_to_end(key)
        if len(known)>20000:known.popitem(last=False)
        rows.append(encoded)
    return '['+','.join(rows)+']'


def transition_source_returns(provider, ground, reference, contexts, source_hash):
    """Associate an adjacent stair window without relabelling future terrain.

    `contexts` is frozen with the proposal, not looked up in the live task.
    Returned offsets label literal PCD rows; reference interpolation is used
    only to select those rows. Ordinary floor and single-stair packets never
    call this function.
    """
    if not isinstance(provider, ConditionedReferenceSupport):
        raise ValueError('transition_requires_classified_source_support')
    ground=np.asarray(ground,dtype=float).reshape((-1,3))
    ids=list(reference.point_segment_ids)
    kinds=list(reference.point_segment_kinds)
    modes=list(reference.point_required_modes)
    if (not 2<=len(contexts)<=16 or len(ground)!=len(ids) or
            len(kinds)!=len(ids) or len(modes)!=len(ids) or not np.isfinite(ground).all()):
        raise ValueError('transition_source_context_invalid')
    ordered=list(dict.fromkeys(ids))
    if ordered!=[c[0] for c in contexts] or not any(c[3]=='stair' for c in contexts):
        raise ValueError('transition_source_context_order_invalid')
    sources=[];geometries=[];regions=[];offset=0;verified=True;last=-1
    # Bounded packet without dropping a small entry/exit region under a large
    # floor region. Keep selected original rows, never voxel-centre synthesis.
    quota=20000//len(contexts)
    for context in contexts:
        sid,floor,kind,mode=context[:4]
        indexes=np.flatnonzero(np.asarray(ids)==sid)
        if (len(indexes)==0 or indexes[0]!=last+1 or
                np.any(np.diff(indexes)!=1) or
                any(kinds[i]!=kind or modes[i]!=mode for i in indexes)):
            raise ValueError('transition_source_context_noncontiguous')
        last=int(indexes[-1])
        # One adjoining sample keeps support at the exact portal visible to
        # both literal source regions, while retaining each source label.
        region_ground=ground[max(0,int(indexes[0])-1):min(len(ground),last+2)]
        # Production passes a read-only view of the complete source segment.
        # Older direct callers can still provide their entire reference here;
        # never derive a TF or a measured point from this selection geometry.
        geometries.append(context[6] if len(context)>6 else region_ground)
        if mode=='stair':
            selection=provider.stair_surface(region_ground,source_map_sha256=source_hash,
                floor_id=floor,segment_kind=kind,required_mode=mode)
            source=selection.points;verified=verified and selection.verified
        else:
            classified=provider.classified_ground(source_map_sha256=source_hash,
                floor_id=floor,segment_kind=kind,required_mode=mode)
            verified=verified and classified
            center=np.mean(region_ground[:,:2],axis=0)
            radius=float(np.max(np.linalg.norm(region_ground[:,:2]-center,axis=1)))+1.5
            source=provider.nearby(center,radius,floor) if classified else np.empty((0,3))
        sources.append(source)
    sources=_unique_transition_owners(provider,sources,contexts,geometries)
    selected=[]
    for context,source in zip(contexts,sources):
        sid,floor,kind,mode=context[:4]
        if len(source)>quota:
            _,unique=np.unique(np.floor(source/.05).astype(np.int64),axis=0,return_index=True)
            source=source[np.sort(unique)]
        if len(source)>quota:source=source[np.linspace(0,len(source)-1,quota,dtype=int)]
        verified=verified and len(source)>=2
        regions.append(dict(segment_id=sid,floor_id=floor,segment_kind=kind,required_mode=mode,
                            begin_index=offset,end_index=offset+len(source)))
        selected.append(source);offset+=len(source)
    return (np.concatenate(selected,axis=0),regions,bool(verified),
            'original_transition_surface_returns_associated' if verified else
            'transition_source_support_incomplete')
