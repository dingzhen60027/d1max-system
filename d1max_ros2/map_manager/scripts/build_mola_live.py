#!/usr/bin/env python3
"""Build an isolated CLI observer against pinned native MOLA libraries (no estimator patch)."""
import hashlib
import json
from pathlib import Path
import subprocess

APP=Path(__file__).resolve().parents[1]
PREFIX=APP/'.local/mola'
COMMIT='4a8c7b70b59550a4ad66cfcc9be593bffe56c2aa'


def main():
    source=PREFIX/'src/mola_lidar_odometry'
    revision=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if revision!=COMMIT:raise RuntimeError('Unexpected MOLA CLI source revision')
    original=(source/'apps/mola-lidar-odometry-cli.cpp').read_text()
    code=original
    for before,after in [('#include <mola_lidar_odometry/LidarOdometry.h>', '#include <mola_lidar_odometry/LidarOdometry.h>\n#include "live_sink.hpp"'),
        ('auto liodom = mola::LidarOdometry::Create();','auto liodom = mola::LidarOdometry::Create();\n  auto live_sink = d1max::attach_live(liodom);'),
        ('  if (cli.arg_outPath.isSet()) {','  if (live_sink) live_sink->finish();\n\n  if (cli.arg_outPath.isSet()) {')]:
        if code.count(before)!=1:raise RuntimeError('Upstream CLI anchor changed')
        code=code.replace(before,after)
    build=PREFIX/'build/live-cli';build.mkdir(parents=True,exist_ok=True)
    generated=build/'mola-live-cli.cpp'
    if not generated.is_file() or generated.read_text()!=code:generated.write_text(code)
    ros=PREFIX/'root/opt/ros/humble'
    destination=PREFIX/'live/opt/ros/humble'
    for command in [ ['cmake','-S',str(APP/'native_live'),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
                      '-DCMAKE_EXE_LINKER_FLAGS=',f'-DPRIVATE_RUNTIME_LIB={PREFIX}/root/usr/lib/x86_64-linux-gnu',
                      f'-DGENERATED_CLI={generated}',f'-DCMAKE_PREFIX_PATH={ros};/opt/ros/humble',f'-DCMAKE_INSTALL_PREFIX={destination}'],
                     ['cmake','--build',str(build),'-j2'],['cmake','--install',str(build)]]:
        subprocess.run(command,check=True)
    (PREFIX/'live.json').write_text(json.dumps({'source_commit':COMMIT,
        'original_cli_sha256':hashlib.sha256(original.encode()).hexdigest(),
        'observer_sha256':hashlib.sha256((APP/'native_live/live_sink.hpp').read_bytes()).hexdigest(),
        'estimator_modified':False},indent=2))


if __name__=='__main__':main()
