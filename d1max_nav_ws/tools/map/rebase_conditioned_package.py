#!/usr/bin/env python3
"""Copy-path rebind of a verified conditioned map; never edit its geometry.

Only route paths and NPZ deployment metadata change. Historical floor manifests,
optimized poses and original PCD remain explicit sealed external dependencies.
This does not make non-rigid geometry execution eligible or create a TF.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'src/d1max_pct_scan'), str(ROOT/'src/d1max_pct_planner')]
from d1max_pct_scan.navigation_map_package import validate_package
from d1max_pct_scan.source_route import digest_file


def array_identity(value):
    a = np.asarray(value)
    return dict(dtype=a.dtype.str, shape=list(a.shape),
                sha256=hashlib.sha256(a.tobytes(order='C')).hexdigest())


def rebase(release, directory, original_directory):
    release = Path(release).resolve(strict=True)
    descriptor = json.loads((release/'release.json').read_text())
    if (release/descriptor['sealed_manifest']).exists():
        raise ValueError('sealed_release_must_not_be_rebased')
    directory = Path(directory).resolve(strict=True)
    original = Path(original_directory).resolve(strict=True)
    if not directory.is_relative_to(release/'map') or directory == original:
        raise ValueError('rebase_requires_a_copied_release_map')
    verified = validate_package(original, purpose='planning_only')
    if verified['capabilities']['geometry_kind'] != 'conditioned_multifloor_ground':
        raise ValueError('conditioned_map_required')
    report_path = directory/'deployment_rebase.json'
    if report_path.exists():
        raise ValueError('rebase_evidence_already_exists')
    for name in ('manifest.json', 'processed_map.pcd', 'route.yaml', 'pct/tomogram.npz'):
        if digest_file(directory/name) != digest_file(original/name):
            raise ValueError('copied_map_bytes_differ:'+name)
    route_path, tomo_path = directory/'route.yaml', directory/'pct/tomogram.npz'
    route = yaml.safe_load(route_path.read_text())
    before = dict(route_sha256=digest_file(route_path), tomogram_sha256=digest_file(tomo_path),
                  manifest_sha256=digest_file(directory/'manifest.json'))
    # One owned archive. Numeric arrays are checked after serialization, not
    # reconstructed from rounded coordinates or recomputed PCT costs.
    with np.load(tomo_path, allow_pickle=False) as archive:
        payload = {key: archive[key] for key in archive.files}
    changes = ('source_pcd', 'source_processing_manifest')
    invariant = {key:array_identity(value) for key,value in payload.items() if key not in changes}
    route.update(source_pcd=str(directory/'processed_map.pcd'), tomogram_path=str(tomo_path),
                 vendor_root=str(release/'pct_vendor'), output_directory=str(directory/'route'))
    payload['source_pcd'] = np.asarray(route['source_pcd'])
    payload['source_processing_manifest'] = np.asarray(str(directory/'manifest.json'))
    temporary = directory/'pct/tomogram.deployment-rebase.npz'
    if temporary.exists():
        raise ValueError('rebase_temporary_already_exists')
    np.savez_compressed(temporary, **payload)
    del payload
    with np.load(temporary, allow_pickle=False) as archive:
        for key, expected in invariant.items():
            if array_identity(archive[key]) != expected:
                raise ValueError('deployment_rebase_changed_array:'+key)
    temporary.replace(tomo_path)
    route_path.write_text(yaml.safe_dump(route,sort_keys=False))
    checked = validate_package(directory, purpose='planning_only')
    report = dict(schema=1, original_package=str(original), copied_package=str(directory),
        geometry_changed=False, execution_eligibility_changed=False,
        before=before, after=dict(route_sha256=digest_file(route_path),
            tomogram_sha256=digest_file(tomo_path),manifest_sha256=digest_file(directory/'manifest.json')),
        unchanged_arrays=invariant, capabilities=checked['capabilities'],
        external_runtime_dependencies={p:h for p,h in checked['dependency_files'].items()
            if not Path(p).is_relative_to(release)},
        rebuild_input_references=checked['rebuild_input_references'])
    report_path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release',type=Path,required=True)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--original-directory',type=Path,required=True)
    result=rebase(**vars(parser.parse_args()))
    print(json.dumps({key:result[key] for key in ('copied_package','geometry_changed',
        'execution_eligibility_changed','before','after','capabilities')},ensure_ascii=False))
