#!/usr/bin/env python3
"""Download distro binaries into a private prefix; never install/upgrade system packages."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

PACKAGES = ['ros-humble-mola-lidar-odometry', 'ros-humble-mola-sm-loop-closure',
            'ros-humble-mp2p-icp', 'ros-humble-mola-metric-maps']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    if prefix in (Path('/'), Path.home(), Path('/opt/ros/humble')):
        raise SystemExit('Refusing a system/home prefix')
    env = {**os.environ, 'LC_ALL': 'C'}
    plan = subprocess.check_output(['apt-get', '-s', '--no-install-recommends', 'install',
                                    *PACKAGES], text=True, env=env)
    packages = []
    for line in plan.splitlines():
        match = re.match(r'Inst (\S+)(?: \[[^]]+\])? \((\S+)', line)
        if match:
            packages.append(match.group(1) + '=' + match.group(2))
    # Also extract requested packages if already installed: the prefix remains portable
    # relative to the existing Humble/system runtime, not dependent on a global MOLA install.
    for package in PACKAGES:
        if not any(item.startswith(package + '=') for item in packages):
            policy = subprocess.check_output(['apt-cache', 'policy', package], text=True,
                                           env=env)
            version = re.search(r'Candidate: (\S+)', policy)
            if not version or version[1] == '(none)':
                raise SystemExit('No candidate for ' + package)
            packages.append(package + '=' + version[1])
    cache = prefix / 'debs'
    cache.mkdir(parents=True, exist_ok=True)
    subprocess.run(['apt-get', 'download', *packages], cwd=cache, check=True)
    destination = prefix / 'root'
    destination.mkdir(exist_ok=True)
    for package in sorted(cache.glob('*.deb')):
        name = subprocess.check_output(['dpkg-deb', '-f', str(package), 'Package'], text=True).strip()
        version = subprocess.check_output(['dpkg-deb', '-f', str(package), 'Version'], text=True).strip()
        if name + '=' + version in packages:
            subprocess.run(['dpkg-deb', '-x', str(package), str(destination)], check=True)
    (prefix / 'packages.json').write_text(json.dumps(packages, indent=2) + '\n')
    print('Private MOLA prefix:', destination)
    print('No system packages or robot services were changed.')


if __name__ == '__main__':
    main()
