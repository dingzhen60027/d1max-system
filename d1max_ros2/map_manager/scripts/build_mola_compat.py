#!/usr/bin/env python3
"""Build only the pinned MOLA map plugin with a Humble RKNN compatibility fix."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

COMMIT = 'e40c814584e5d787e7bf69e287a7c3e8d732e948'


def run(command, **kwargs):
    subprocess.run(command, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    ros = prefix / 'root/opt/ros/humble'
    if not (ros / 'share/mola_metric_maps/package.xml').is_file():
        raise SystemExit('Run install_mola_isolated.py first')
    source = prefix / 'src/mola'
    patch = Path(__file__).parent / 'patches/mola-3.0.0-humble-knn.patch'
    if not source.exists():
        run(['git', 'clone', '--depth', '1', '--branch', '3.0.0',
             'https://github.com/MOLAorg/mola.git', str(source)])
    revision = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != COMMIT:
        raise SystemExit('Refusing to patch a different source revision')
    run(['git', '-C', str(source), 'submodule', 'update', '--init', '--depth', '1',
         'mola_metric_maps/3rdparty/robin-map'])
    applied = subprocess.run(['git', '-C', str(source), 'apply', '--reverse', '--check', str(patch.resolve())],
                             capture_output=True).returncode == 0
    if not applied:
        run(['git', '-C', str(source), 'apply', '--check', str(patch.resolve())])
        run(['git', '-C', str(source), 'apply', str(patch.resolve())])
    build = prefix / 'build/metric-maps'
    destination = prefix / 'compat/opt/ros/humble'
    # Do not inherit ROS live discovery or a different overlay for this build.
    env = {**os.environ, 'CMAKE_PREFIX_PATH': str(ros) + ':/opt/ros/humble'}
    run(['cmake', '-S', str(source / 'mola_metric_maps'), '-B', str(build),
         '-DCMAKE_BUILD_TYPE=Release', f'-DCMAKE_PREFIX_PATH={ros};/opt/ros/humble',
         f'-DCMAKE_INSTALL_PREFIX={destination}', '-DBUILD_TESTING=OFF',
         '-DMOLA_METRIC_MAPS_BUILD_APPS=OFF'], env=env)
    run(['cmake', '--build', str(build), '--target', 'mola_metric_maps', '-j2'], env=env)
    run(['cmake', '--install', str(build)], env=env)
    library = destination / 'lib/libmola_metric_maps.so.3.0.0'
    (prefix / 'compat.json').write_text(json.dumps({
        'source': 'https://github.com/MOLAorg/mola', 'commit': COMMIT,
        'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
        'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
        'fix': 'KNN + identical radius bound, no planar or estimator changes',
    }, indent=2) + '\n')
    print('Private compatibility plugin ready:', library)


if __name__ == '__main__':
    main()
