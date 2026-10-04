#!/usr/bin/env python3
"""Rebind an unsealed copied map package, without rebuilding its geometry.

All numeric arrays, source indices, PCD records, manifest bytes, collision costs
and acceptance flags stay unchanged. Only route paths and two NPZ provenance
paths change. The resulting new artifact hashes must be sealed as a new release.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src/d1max_pct_scan'), str(ROOT/'src/d1max_pct_planner')]
from d1max_pct_scan.source_identity import SourceIdentityBridge, validate_package
from d1max_pct_scan.source_route import digest_file
from d1max_pct_planner.paths import expand_tree, inside
from d1max_pct_planner.native_runtime import prepare_native_environment, vendorverify


def array_identity(value):
    a = np.asarray(value)
    return dict(dtype=a.dtype.str, shape=list(a.shape),
                sha256=hashlib.sha256(a.tobytes(order='C')).hexdigest())


def rebase(release, map_directory, *, vendor_root=None):
    release = Path(release).resolve(strict=True)
    descriptor = json.loads((release/'release.json').read_text())
    if descriptor.get('schema') != 1:
        raise ValueError('release_descriptor_schema')
    seal = inside(release, descriptor['sealed_manifest'], what='sealed_manifest')
    if seal.exists():
        raise ValueError('sealed_release_must_not_be_rebased')
    directory = Path(map_directory).resolve(strict=True)
    if not directory.is_relative_to(release/'map'):
        raise ValueError('map_rebase_must_remain_inside_unsealed_release')
    evidence_path = directory/'deployment_rebase.json'
    if evidence_path.exists():
        raise ValueError('map_rebase_evidence_already_exists')
    bridge = SourceIdentityBridge.from_artifacts(directory/'manifest.json')
    route_path, tomo_path = directory/'route.yaml', directory/'tomogram.npz'
    raw = expand_tree(yaml.safe_load(route_path.read_text()))
    if (raw.get('schema') != 'd1max.source_identity_route/v1'
            or raw.get('stairs_enabled') is not False or raw.get('floor_id') != bridge.floor_id):
        raise ValueError('source_identity_route_invalid_for_rebase')
    # Prove this is a copied package, not a licence to redirect another map.
    if (digest_file(raw['source_pcd']) != bridge.manifest['output_sha256']
            or digest_file(raw['tomogram_path']) != digest_file(tomo_path)):
        raise ValueError('copied_route_artifact_bytes_differ')
    with np.load(tomo_path, allow_pickle=False) as archive:
        payload = {k:archive[k].copy() for k in archive.files}
    if (str(payload['source_sha256'].item()) != bridge.manifest['output_sha256']
            or str(payload['source_processing_manifest_sha256'].item()) != digest_file(directory/'manifest.json')
            or str(payload['original_source_sha256'].item()) != bridge.manifest['source_sha256']
            or str(payload['geometry_operation'].item()) != 'source_identity'):
        raise ValueError('copied_tomogram_provenance_mismatch')
    vendor = Path(vendor_root or raw['vendor_root']).resolve(strict=True)
    native = vendorverify(vendor)
    environment = prepare_native_environment(vendor)
    # Read the actual loader resolution used by a production PCT child. A bare
    # ldd can select ROS's incompatible GTSAM 4.2 despite this verified 4.1 build.
    dependencies = {}
    for name, library in native['native_libraries'].items():
        result = subprocess.run(['ldd',library], env=environment, text=True,
                                capture_output=True, timeout=10)
        if result.returncode or 'not found' in result.stdout:
            raise ValueError('rebase_native_dependency_unresolved:'+name)
        dependencies[name] = result.stdout
    before = dict(route_sha256=digest_file(route_path), tomogram_sha256=digest_file(tomo_path),
        route_paths={k:raw[k] for k in ('source_pcd','tomogram_path','vendor_root')},
        provenance_paths={k:str(payload[k].item()) for k in ('source_pcd','source_processing_manifest')})
    invariant = {k:array_identity(v) for k,v in payload.items()
                 if k not in ('source_pcd','source_processing_manifest')}
    raw.update(source_pcd=str(directory/bridge.manifest['output_file']),
               tomogram_path=str(tomo_path), vendor_root=str(vendor))
    payload['source_pcd'] = np.asarray(raw['source_pcd'])
    payload['source_processing_manifest'] = np.asarray(str(directory/'manifest.json'))
    # The only binary rewrite is deployment metadata. Numeric data are verified
    # bit for bit, including NaN payloads, after the serialized archive is read.
    temporary = directory/'tomogram.deployment-rebase.npz'
    if temporary.exists(): raise ValueError('temporary_rebase_artifact_exists')
    np.savez_compressed(temporary, **payload)
    with np.load(temporary, allow_pickle=False) as archive:
        if {k:array_identity(archive[k]) for k in invariant} != invariant:
            raise ValueError('deployment_rebase_changed_geometry_or_semantics')
    temporary.replace(tomo_path)
    route_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    validate_package(directory, bridge=bridge)
    evidence = dict(schema=1, geometry_changed=False, acceptance_changed=False,
        map_directory=str(directory), before=before,
        after=dict(route_sha256=digest_file(route_path), tomogram_sha256=digest_file(tomo_path),
            route_paths={k:raw[k] for k in ('source_pcd','tomogram_path','vendor_root')}),
        original_source=dict(path=bridge.manifest['source_path'],sha256=bridge.manifest['source_sha256']),
        unchanged_files={name:digest_file(directory/name) for name in
            ('manifest.json',bridge.manifest['output_file'],bridge.manifest['source_indices_file'])},
        unchanged_array_identity=invariant, native_build=native, native_loader_dependencies=dependencies,
        note='Deployment path rebase only. Reseal as a new version; no motion authorization.')
    evidence_path.write_text(json.dumps(evidence, indent=2, ensure_ascii=False)+'\n')
    return evidence


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--map-directory', type=Path, required=True)
    parser.add_argument('--vendor-root', type=Path)
    args = parser.parse_args()
    result = rebase(**vars(args))
    print(json.dumps(dict(map_directory=result['map_directory'], before=result['before'],
        after=result['after'], geometry_changed=False, acceptance_changed=False),ensure_ascii=False))
