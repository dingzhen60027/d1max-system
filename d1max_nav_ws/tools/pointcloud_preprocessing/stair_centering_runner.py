"""Offline measured stair-centering build; never activates a map or a robot.

The immutable native PCT origin remains literal. Its separately documented XY
quality derivative is checked in the unchanged tomogram and original support.
Only package identities are rebased; no PCD point or map cell is rewritten.
"""
from copy import deepcopy
import argparse
import json
from pathlib import Path
import pickle
import shutil

import numpy as np
import yaml

from .pcd_io import read_pcd
from .stair_regularization_runner import _sha, _json


def _record_rejection(output, result):
    """Preserve the actual quality rejection instead of masking it with JSON errors."""
    record = dict(result, path=np.asarray(result['path']).tolist(),
                  layer_ids=np.asarray(result['layer_ids']).tolist())
    _json(output/'centering_not_applied.json', record)
    return dict(applied=False, output=str(output),
        reason=result['diagnostics']['candidate_rejection'], diagnostics=result['diagnostics'])


def relocate_package(source, output, *, destination=None):
    """Copy a sealed package and update its explicit current-path/hash graph.

    ``destination`` is only for staging an installation at the existing default
    path. In that case this function does not validate against files still at
    that path: the deployment transaction must validate after installation.
    Historical origin/evidence files are copied unchanged.
    """
    from d1max_pct_scan.navigation_map_package import validate_package
    from d1max_pct_planner.crossfloor_route import validate_config
    from d1max_pct_planner.compiled_stair_connector import map_settings_sha256
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    destination = Path(destination).resolve() if destination is not None else output
    if output.exists() or output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('stair_centering_output_must_be_new_independent_directory')
    checked = validate_package(source)
    frozen = dict(checked['dependency_files'])
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('stair_centering_package_symlink_not_supported')

    def rebase(value):
        if isinstance(value, str) and (value == str(source) or value.startswith(str(source)+'/')):
            return str(destination)+value[len(str(source)):]
        if isinstance(value, dict): return {rebase(k): rebase(v) for k, v in value.items()}
        if isinstance(value, list): return [rebase(v) for v in value]
        return value

    shutil.copytree(source, output, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    stage = rebase(json.loads((source/'floor_stage/manifest.json').read_text()))
    _json(output/'floor_stage/manifest.json', stage)
    manifest = rebase(checked['manifest'])
    manifest['floor_processing_manifest_sha256'] = _sha(output/'floor_stage/manifest.json')
    _json(output/'manifest.json', manifest)
    for name in ('pct_pipeline.yaml', 'pct/pipeline.yaml'):
        (output/name).write_text(yaml.safe_dump(rebase(yaml.safe_load((source/name).read_text())),
                                              sort_keys=False, allow_unicode=True))
    with np.load(source/'pct/tomogram.npz', allow_pickle=False) as archive:
        payload = {key: archive[key] for key in archive.files}
    payload.update(source_pcd=str(destination/'processed_map.pcd'),
        source_processing_manifest=str(destination/'manifest.json'),
        source_processing_manifest_sha256=_sha(output/'manifest.json'),
        config_sha256=_sha(output/'pct_pipeline.yaml'))
    np.savez_compressed(output/'pct/tomogram.npz', **payload)
    with (output/'pct/tomogram.pickle').open('wb') as stream:
        pickle.dump(payload, stream, protocol=pickle.HIGHEST_PROTOCOL)
    route = rebase(yaml.safe_load((source/'route.yaml').read_text()))
    bundle = json.loads((source/'compiled_stair_connector.json').read_text())
    bundle['tomogram_sha256'] = _sha(output/'pct/tomogram.npz')
    bundle['map_settings_sha256'] = map_settings_sha256(validate_config(route))
    _json(output/'compiled_stair_connector.json', bundle)
    route['compiled_stair_connector']['sha256'] = _sha(output/'compiled_stair_connector.json')
    (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    pct = rebase(json.loads((source/'pct/manifest.json').read_text()))
    pct['config_sha256'] = _sha(output/'pct_pipeline.yaml')
    import hashlib
    pct['canonical_config_sha256'] = hashlib.sha256(json.dumps(pct['config'], sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    pct['source_processing']['manifest_sha256'] = _sha(output/'manifest.json')
    pct['files'] = {name: dict(sha256=_sha(output/'pct'/name), bytes=(output/'pct'/name).stat().st_size)
                    for name in pct['files']}
    _json(output/'pct/manifest.json', pct)
    if any(_sha(path) != digest for path, digest in frozen.items()):
        raise ValueError('stair_centering_input_changed_during_copy')
    metadata = {'floor_stage/manifest.json', 'manifest.json', 'pct_pipeline.yaml', 'pct/pipeline.yaml',
                'pct/tomogram.npz', 'pct/tomogram.pickle', 'route.yaml',
                'compiled_stair_connector.json', 'pct/manifest.json'}
    for path in source.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
            relative = str(path.relative_to(source))
            if relative not in metadata and _sha(path) != _sha(output/relative):
                raise ValueError('stair_centering_relocation_changed_geometry:' + relative)
    if destination == output:
        validate_package(output)
    return output


def build(package, output):
    from d1max_pct_planner.compiled_stair_connector import (
        REFINEMENT_SCHEMA, polyline_sha256, validate_centering_origin)
    from d1max_pct_planner.stair_centering import (
        center_measured_stair_path, extract_measured_flight_structure)
    from d1max_pct_scan.navigation_map_package import validate_package, package_version
    from d1max_pct_scan.source_route import SourceRouteBuilder
    from d1max_pct_scan.static_route_validation import validate_static_route
    package = Path(package).resolve(strict=True)
    before = validate_package(package)
    frozen = dict(before['dependency_files'])
    output = relocate_package(package, output)
    checked = validate_package(output)
    grid, settings = checked['tomogram'], checked['route']
    bundle = json.loads((output/'compiled_stair_connector.json').read_text())
    if bundle['origin'].get('centering_refinement'):
        raise ValueError('stair_centering_requires_unmodified_native_origin_not_cumulative_refinement')
    original = json.loads((output/bundle['origin']['route_file']).read_text())
    points, layers = np.asarray(bundle['path'], float), np.asarray(bundle['layer_ids'], int)
    edge_legs = ['stair_lower']*bundle['landing_index']+['stair_upper']*(len(points)-1-bundle['landing_index'])
    builder = SourceRouteBuilder.from_artifacts(checked['manifest_path'], grid, checked['bridge'])
    support_checks = []
    def validate_candidate(candidate, candidate_layers):
        proposed = deepcopy(original)
        proposed.update(path=np.asarray(candidate).tolist(), layer_ids=np.asarray(candidate_layers).tolist(),
            source_tomogram_sha256=grid.sha256, execution_authorized=False)
        result = validate_static_route(proposed, grid, checked['bridge'], builder=builder,
                                      map_version_id=package_version(checked))
        evidence = result['route_snapshot'].payload()['geometry_evidence']
        if not evidence['measured_source_support']['all_samples_supported']:
            raise ValueError('stair_centering_original_support_missing')
        support_checks.append(evidence)
        return True
    models = checked['manifest']['stair_regularization']['surfaces']
    flights = extract_measured_flight_structure(read_pcd(output/'processed_map.pcd').xyz, models)
    result = center_measured_stair_path(grid, points, layers, edge_legs,
        settings=settings, flights=flights, validate_candidate=validate_candidate)
    if not result['applied']:
        return _record_rejection(output, result)
    new = np.asarray(result['path'], float)
    validate_candidate(new, layers)
    evidence = dict(schema=REFINEMENT_SCHEMA, execution_authorized=False,
        source_route_sha256=bundle['source_route_sha256'],
        input_geometry_sha256=polyline_sha256(points, layers), output_geometry_sha256=polyline_sha256(new, layers),
        changed_vertices=np.flatnonzero(np.any(points != new, axis=1)).tolist(),
        flight_spans=result['flight_spans'], diagnostics=result['diagnostics'],
        original_support=support_checks[-1]['measured_source_support'],
        source_projection=support_checks[-1]['projection'],
        physical_acceptance=False, unchanged_pcd_sha256=checked['manifest']['output_sha256'],
        input_dependencies=frozen, geometry_kind='checked_measured_xy_quality_derivative_not_new_native_search')
    validate_centering_origin(evidence, original, new, layers, bundle['landing_index'], bundle['source_route_sha256'])
    _json(output/'stair_centering.json', evidence)
    bundle['path'] = new.tolist()
    bundle['origin']['centering_refinement'] = dict(file='stair_centering.json', sha256=_sha(output/'stair_centering.json'))
    _json(output/'compiled_stair_connector.json', bundle)
    route = yaml.safe_load((output/'route.yaml').read_text())
    route['compiled_stair_connector']['sha256'] = _sha(output/'compiled_stair_connector.json')
    (output/'route.yaml').write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    final = validate_package(output)
    if any(_sha(path) != digest for path, digest in frozen.items()):
        raise ValueError('stair_centering_original_dependencies_changed')
    return dict(applied=True, output=str(output), changed_vertices=len(evidence['changed_vertices']),
        diagnostics=result['diagnostics'], source_support=evidence['original_support'],
        dependency_files=final['dependency_files'], activated=False, physical_acceptance=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.package, args.output), indent=2, allow_nan=False))
