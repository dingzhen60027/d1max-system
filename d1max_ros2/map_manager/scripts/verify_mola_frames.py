#!/usr/bin/env python3
"""Bounded, offline D1 extrinsic regression using an existing task's SI input."""
import argparse
import json
from pathlib import Path
import tempfile
from backend.mapping.configuration import parse_config, config_yaml
from backend.mapping.mola_runtime import atomic_json, now
from backend.mapping.worker import Worker, prepare
from backend.mapping.toolchain import live_binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared_bag', type=Path)
    parser.add_argument('--steps', type=int, default=30000)
    args = parser.parse_args()
    if not 1000 <= args.steps <= 60000:
        raise ValueError('Diagnostic is limited to 1000..60000 input messages')
    app = Path(__file__).resolve().parents[1]
    config = parse_config((app / 'config/mapping/experiments/d1max_903_mola.yaml').read_text())
    # Already converted SI copy, never scale acceleration twice.
    config.input.bag_path = str(args.prepared_bag.resolve(strict=True))
    config.input.imu_acceleration_scale = 1.
    config.loop_closure.enabled = False
    config.execution.stage_timeout_sec = 300
    session = Path(tempfile.mkdtemp(prefix='d1max-mola-frames-'))
    (session / 'task.yaml').write_text(config_yaml(config))
    atomic_json(session / 'manifest.json', {'id': session.name, 'started_at': now(), 'artifacts': []})
    worker = Worker(session, app)
    worker.update(provenance=prepare(session, app, config), diagnostic_steps=args.steps)
    print(f'FRAME_CHECK_SESSION={session}', flush=True)
    worker.stage('lio', 5, [str(live_binary(app)), '-c', 'frontend.yaml',
        '--state-estimator-param-file', 'estimator.yaml', '--input-rosbag2', config.input.bag_path,
        '--lidar-sensor-label', config.input.lidar_topic, '--imu-sensor-label', config.input.imu_topic,
        '--base-link-frame-id', config.input.base_frame, '--only-first-n', str(args.steps),
        '--output-tum-path', 'trajectory.tum', '--output-simplemap', 'raw.simplemap'])
    worker.export('raw', 40)
    worker.update(status='complete', stage='complete', completed_at=now())
    print(json.dumps(worker.manifest, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
