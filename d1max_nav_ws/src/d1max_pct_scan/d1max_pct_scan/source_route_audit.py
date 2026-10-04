"""Read-only two-direction audit of an existing, validated PCT route artifact.

No ROS initialization, map edits, worker process or execution authorization.
The legacy audit is geometrically revalidated before explicitly constructing v2
snapshots; this CLI cannot silently migrate a live old-schema action/session.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .live_global_contract import labels_for_result
from .source_route import SourceRouteBuilder


def reverse_audit_route(result):
    result = deepcopy(result)
    n = len(result['path'])
    for key in ('path', 'layer_ids', 'source_layer_ids', 'edge_legs'):
        result[key] = result[key][::-1]
    result['segments'] = [{**segment,
        'first_index':n-1-segment['last_index'], 'last_index':n-1-segment['first_index'],
        'from':segment['to'], 'to':segment['from']} for segment in result['segments'][::-1]]
    result['anchors']['start'],result['anchors']['goal'] = result['anchors']['goal'],result['anchors']['start']
    rename = {'start':'goal','goal':'start'}
    for segment in result['segments']:
        for key in ('from','to'):
            segment[key] = rename.get(segment[key],segment[key])
    result['layer_transitions'] = [{**event,'edge':n-2-event['edge'],
        'from_layer':event['to_layer'],'to_layer':event['from_layer']}
        for event in result['layer_transitions'][::-1]]
    result['direction'] = 'upper_to_lower'
    return result


def audit(manifest, route_config, route_audit):
    from .pointcloud_helpers.ground_path_bridge import GroundPathBridge
    from d1max_pct_planner.crossfloor_preview import restore_crossfloor
    from d1max_pct_planner.tomogram_map import TomogramMap
    from .navigation_contract import artifact_identity
    began = time.monotonic()
    manifest,route_config,route_audit = map(lambda p:Path(p).resolve(strict=True),
                                           (manifest,route_config,route_audit))
    cfg = yaml.safe_load(route_config.read_text())
    tomogram = TomogramMap(cfg['tomogram_path'],
        unknown_ceiling_policy=cfg['unknown_ceiling_policy'],
        max_ground_step_m=cfg['limits']['max_ground_step_m'])
    bridge = GroundPathBridge.from_artifacts(manifest)
    builder = SourceRouteBuilder.from_artifacts(manifest,tomogram,bridge)
    source = json.loads(manifest.read_text())['source_path']
    version,_ = artifact_identity(dict(map_pcd=source,planning_manifest=str(manifest),
        tomogram_npz=cfg['tomogram_path'],crossfloor_route_config=str(route_config)))
    candidate = restore_crossfloor(tomogram,route_config,route_audit)
    candidate['direction'] = 'lower_to_upper'
    records, snapshots = [], []
    for result in (candidate,reverse_audit_route(candidate)):
        points,labels = labels_for_result(result,same_floor=None)
        tomogram.validate_path(points,np.asarray(result['layer_ids']))
        snapshot = builder.build(result,points,labels,map_version_id=version)
        value = snapshot.payload(); snapshots.append(value)
        records.append(dict(direction=value['direction'],route_hash=snapshot.route_hash,
            points=len(value['xyz']),segments=value['segments'],
            measured_source_support=value['geometry_evidence']['measured_source_support'],
            preview_ready=value['preview_ready'],execution_eligible=value['execution_eligible'],
            eligibility_reason=value['eligibility_reason'],
            certified_geometric_error_bound_m=None))
    return dict(schema=2,map_version_id=version,elapsed_s=time.monotonic()-began,
        source_map_sha256=builder.provenance['source_map_sha256'],
        opposite_directions_same_source_ground=bool(np.allclose(
            snapshots[0]['xyz'],snapshots[1]['xyz'][::-1],atol=1e-9,rtol=0)),
        directions=records,motion_enabled=False,source_artifacts_modified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--route-config',required=True)
    parser.add_argument('--route-audit',required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.manifest,args.route_config,args.route_audit),
                     ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
