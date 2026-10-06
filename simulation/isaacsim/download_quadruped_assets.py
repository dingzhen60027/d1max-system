#!/usr/bin/env python3
"""Fetch the pinned official Isaac 6 Spot USD dependency closure and policy.

Uses only the NVIDIA production asset URLs and expected sizes/SHA256 in the
shipped manifest. No Kit, ROS, simulator, pip install or GPU is started. A changed
remote asset fails verification; it is not accepted as the tested model/policy.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
import urllib.request
from pathlib import Path
from urllib.parse import quote, urlsplit

OFFICIAL_BASE = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/'
MANIFEST_PATH = Path(__file__).resolve().parent / 'assets/official_spot_assets.json'
DEFAULT_OUTPUT = Path(os.environ.get('D1MAX_SIM_BUILD',
    str(Path(__file__).resolve().parents[3]/'d1max-build-isaac'))) / 'quadruped-assets-spot'


def _valid_entry(entry):
    path = Path(entry['path'])
    if path.is_absolute() or not path.parts or '..' in path.parts or path.parts[0] != 'Isaac':
        raise ValueError('unsafe_official_asset_path')
    expected_url = OFFICIAL_BASE + quote(path.as_posix(), safe='/._-')
    if entry.get('source_url') != expected_url:
        raise ValueError('untrusted_official_asset_url')
    if type(entry.get('size_bytes')) is not int or not 0 < entry['size_bytes'] <= 100_000_000:
        raise ValueError('invalid_official_asset_size')
    digest = entry.get('sha256', '')
    if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
        raise ValueError('invalid_official_asset_digest')
    return path


def _matches(path, entry):
    return (path.is_file() and path.stat().st_size == entry['size_bytes']
            and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'])


def download(output, *, manifest_path=MANIFEST_PATH):
    output = Path(output).expanduser().resolve()
    source_bytes = Path(manifest_path).read_bytes()
    manifest = json.loads(source_bytes)
    if manifest.get('schema') != 1 or manifest.get('source') != 'official_isaac_6_physx_spot':
        raise ValueError('invalid_pinned_official_spot_manifest')
    files = manifest.get('files', [])
    if len(files) != 10 or len({f['path'] for f in files}) != 10:
        raise ValueError('incomplete_pinned_official_spot_manifest')
    entries = [(entry, _valid_entry(entry)) for entry in files]
    output.mkdir(parents=True, exist_ok=True)
    fetched = skipped = 0
    for entry, relative in entries:
        target = output / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.parent.resolve().is_relative_to(output) or target.is_symlink():
            raise ValueError('unsafe_cache_symlink')
        if _matches(target, entry):
            skipped += 1
            continue
        if target.exists():
            raise ValueError('existing_cache_hash_mismatch:' + str(target))
        temporary = None
        try:
            request = urllib.request.Request(entry['source_url'], headers={'User-Agent': 'D1Max-Isaac-official-asset-cache/1'})
            with urllib.request.urlopen(request, timeout=60) as response:
                final_url = response.geturl()
                if urlsplit(final_url).netloc != urlsplit(OFFICIAL_BASE).netloc or not final_url.startswith(OFFICIAL_BASE):
                    raise ValueError('untrusted_asset_redirect')
                with tempfile.NamedTemporaryFile(dir=target.parent, prefix=target.name + '.', suffix='.partial', delete=False) as stream:
                    temporary = Path(stream.name)
                    size, digest = 0, hashlib.sha256()
                    while block := response.read(1024 * 1024):
                        size += len(block)
                        if size > entry['size_bytes']:
                            raise ValueError('official_asset_exceeds_pinned_size')
                        stream.write(block)
                        digest.update(block)
            if size != entry['size_bytes'] or digest.hexdigest() != entry['sha256']:
                raise ValueError('official_asset_hash_mismatch:' + entry['path'])
            temporary.replace(target)
            fetched += 1
            print('verified', entry['path'], size, flush=True)
        finally:
            if temporary and temporary.exists():
                temporary.unlink()
    # Publish authorization only after the entire original dependency closure
    # has matched its pinned hashes. Manifest bytes are reproducible.
    manifest_out = output / 'asset_manifest.json'
    manifest_out.write_bytes(source_bytes)
    return dict(manifest_path=str(manifest_out), manifest_sha256=hashlib.sha256(source_bytes).hexdigest(),
                total_bytes=manifest['total_bytes'], downloaded_files=fetched, already_verified_files=skipped)


def verify_usd_dependencies(output):
    """Optional offline USD resolver verification; requires pxr, never Kit."""
    from pxr import Usd, UsdUtils
    output = Path(output).resolve()
    manifest = json.loads((output / 'asset_manifest.json').read_text())
    allowed = {(output / entry['path']).resolve() for entry in manifest['files']}
    usd_paths = [path for path in allowed if path.suffix in ('.usd', '.usda', '.usdc')]
    for path in usd_paths:
        for group in UsdUtils.ExtractExternalReferences(str(path)):
            for reference in group:
                if reference == 'OmniPBR.mdl':
                    continue  # bundled Isaac material shader, not robot geometry
                resolved = (path.parent / reference).resolve()
                if resolved not in allowed or not resolved.is_file():
                    raise ValueError('unsealed_usd_dependency:' + str(reference))
    stage = Usd.Stage.Open(str(output / 'Isaac/Robots/BostonDynamics/spot/spot.usd'))
    if not stage or not stage.GetDefaultPrim():
        raise ValueError('official_spot_usd_did_not_compose')
    return dict(usd_dependency_files=len(usd_paths), default_prim=str(stage.GetDefaultPrim().GetPath()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--verify-usd-dependencies', action='store_true', help='Offline dependency audit using installed pxr')
    args = parser.parse_args()
    result = download(args.output)
    if args.verify_usd_dependencies:
        result['usd_audit'] = verify_usd_dependencies(args.output)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
