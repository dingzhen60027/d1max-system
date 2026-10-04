#!/usr/bin/env python3
"""Copy actual existing LIO/Livox bytes; no build, ROS, SDK or deployment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def freeze(nav_root, release):
    nav_root=Path(nav_root).resolve(strict=True);release=Path(release).resolve(strict=True)
    descriptor=json.loads((release/'release.json').read_text())
    if (release/descriptor['sealed_manifest']).exists():raise ValueError('sealed_release_not_mutable')
    target=release/'localization_dependencies'
    if target.exists():raise ValueError('refusing_to_overwrite_localization_dependencies')
    sources={name:nav_root/'install'/name for name in ('faster_lio','livox_ros_driver2')}
    for path in sources.values():
        if not path.is_dir():raise ValueError('missing_actual_localization_prefix:'+str(path))
    target.mkdir();prefix=target/'install';prefix.mkdir()
    files={}
    for package,source in sources.items():
        subprocess.run(['rsync','-aL','--exclude=__pycache__',str(source),str(prefix)+'/'],check=True)
        for copied in (prefix/package).rglob('*'):
            if not copied.is_file():continue
            original=(source/copied.relative_to(prefix/package)).resolve(strict=True)
            original_hash=sha(original);copied_hash=sha(copied)
            if original_hash!=copied_hash:raise ValueError('actual_copy_bytes_mismatch:'+str(copied))
            files[str(copied)]=dict(source_path=str(original),sha256=copied_hash,source_sha256=original_hash)
    # A relocatable loader for the two same-byte isolated package prefixes.
    # Package setup scripts determine their own copied path; no old workspace
    # setup.bash is copied with its additional source/build prefixes.
    loader='''#!/usr/bin/env bash
_d1max_localization_dependencies_prefix=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
for _d1max_localization_package in livox_ros_driver2 faster_lio; do
  source "$_d1max_localization_dependencies_prefix/$_d1max_localization_package/share/$_d1max_localization_package/package.bash"
done
unset _d1max_localization_dependencies_prefix _d1max_localization_package
'''
    (prefix/'local_setup.bash').write_text(loader)
    result=dict(schema_version=1,operation='same_byte_rsync_aL_runtime_copy',
        rebuilt_from_source=False,physical_acceptance=False,
        source_workspace=str(nav_root),copied_prefix=str(prefix),files=files,
        caveat='Original ELF RUNPATH and system ROS/GTSAM dependencies remain; actual loaded paths must be separately sealed.')
    (target/'runtime_copy_provenance.json').write_text(json.dumps(result,indent=2)+'\n')
    return dict(prefix=str(prefix),copied_files=len(files),rebuilt_from_source=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-root',required=True);parser.add_argument('--release',required=True)
    args=parser.parse_args();print(json.dumps(freeze(args.nav_root,args.release)))


if __name__=='__main__':main()
