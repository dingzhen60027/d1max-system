"""Read-only runtime input identities; no ROS/SDK context or numeric workers.

External scientific packages remain external, but their actually selected
Python/native bytes and native loader dependencies are version-bound. A copied
source YAML that is not used by the worker is never substituted for its real
compute-budget source.
"""
import hashlib
import importlib.metadata
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

from .compute_budget import FROZEN_ENV, load_budget, validate_budget

PACKAGES = {'numpy':'numpy', 'scipy':'scipy', 'open3d':'open3d', 'yaml':'PyYAML'}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def compute_budget_contract(environment=None):
    env = os.environ if environment is None else environment
    explicit = env.get('D1MAX_PCT_COMPUTE_CONFIG')
    if explicit:
        path = Path(explicit)
        if not path.is_absolute():
            raise ValueError('compute_budget_override_must_be_absolute')
    else:
        from . import compute_budget
        path = Path(compute_budget.__file__).resolve().parents[1]/'config/compute_budget.yaml'
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('d1max_pct_planner'))/'config/compute_budget.yaml'
    path = path.resolve(strict=True)
    import yaml
    file_value = validate_budget(yaml.safe_load(path.read_text()))
    effective = load_budget(env)
    # Internal child FROZEN_ENV is permitted only as an exact equivalent of
    # the sealed file. An arbitrary ambient JSON value cannot bypass that file.
    if effective != file_value:
        raise ValueError('compute_budget_frozen_override_differs_from_actual_file')
    return dict(path=str(path), sha256=sha(path), effective=effective)


def _dependencies(output):
    for line in output.splitlines():
        target=line.split('=>',1)[-1].strip().rsplit(' (',1)[0].strip()
        if target.startswith('/'):
            yield Path(target).resolve(strict=True)


def _package_identity(module, distribution):
    spec = importlib.util.find_spec(module)
    if spec is None or not spec.origin or not spec.submodule_search_locations:
        raise ValueError('scientific_runtime_package_missing:'+module)
    origin = Path(spec.origin).resolve(strict=True)
    roots = [Path(p).resolve(strict=True) for p in spec.submodule_search_locations]
    if len(roots) != 1 or not origin.is_relative_to(roots[0]):
        raise ValueError('scientific_runtime_package_origin_ambiguous:'+module)
    return dict(origin=str(origin), root=str(roots[0]),
                version=importlib.metadata.version(distribution), distribution=distribution)


def scientific_contract():
    packages = {name:_package_identity(name,dist) for name,dist in PACKAGES.items()}
    files = {}
    for package in packages.values():
        for path in Path(package['root']).rglob('*'):
            if path.is_file() and (path.suffix == '.py' or '.so' in path.name):
                resolved = path.resolve(strict=True)
                files[str(resolved)] = sha(resolved)
    interpreter = Path(sys.executable).resolve(strict=True)
    files[str(interpreter)] = sha(interpreter)
    return dict(packages=packages, interpreter=str(interpreter),
                files=dict(sorted(files.items())),
                storage='external_actual_python_and_native_bytes_not_copied')


def scientific_loader_dependencies(scientific):
    # Keep prepare a pure file/config operation. Loader resolution is a
    # separate read-only preflight, alongside the existing executable ldd.
    # Python dlopen extensions are not dependencies of the ROS executable.
    # Inspect these actual selected ELFs in the ordinary application's loader
    # environment, not the separate PCT child's GTSAM loader override.
    files = {}
    for path in scientific['files']:
        with Path(path).open('rb') as stream:
            elf = stream.read(4) == b'\x7fELF'
        if not elf:
            continue
        result = subprocess.run(['ldd',path], text=True,capture_output=True,timeout=10)
        if result.returncode or 'not found' in result.stdout:
            raise ValueError('scientific_runtime_dependency_unresolved:'+path)
        for dependency in _dependencies(result.stdout):
            files[str(dependency)] = sha(dependency)
    return dict(sorted(files.items()))


def capture_runtime_inputs(environment=None):
    return dict(schema_version=1, compute_budget=compute_budget_contract(environment),
                scientific=scientific_contract())


def verify_runtime_inputs(saved, environment=None):
    if not isinstance(saved,dict) or saved.get('schema_version') != 1:
        raise ValueError('runtime_inputs_contract_schema')
    budget = compute_budget_contract(environment)
    if budget != saved.get('compute_budget'):
        raise ValueError('compute_budget_runtime_changed_or_unsealed_override')
    actual = scientific_contract()
    if actual != saved.get('scientific'):
        raise ValueError('scientific_runtime_changed_or_unsealed_import')
    return saved


def contract_files(saved):
    budget = saved['compute_budget']
    return {budget['path']:budget['sha256'], **saved['scientific']['files']}
