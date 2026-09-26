#!/usr/bin/env python3
"""Explicit opt-in: only a uniquely named QA systemd unit; no SLAM or robot."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch
import uuid
import yaml
from backend.mapping.mola_runtime import MolaRuntime, Systemd, MARKER
from backend.mapping.configuration import MolaConfig, config_yaml


class QASystemd(Systemd):
    def start(self, session, app_root):
        # A worker plus a child verifies KillMode=control-group, not just MainPID.
        code = 'import subprocess,time; subprocess.Popen(["/usr/bin/sleep","120"]); time.sleep(120)'
        subprocess.run(['systemd-run', '--user', '--collect', '--unit=' + self.unit,
            '--property=Description=' + MARKER + session.name,
            '--property=KillMode=control-group', '--property=TimeoutStopSec=3',
            sys.executable, '-c', code], check=True, capture_output=True, timeout=8)


def main():
    if os.environ.get('D1MAX_MOLA_QA') != '1':
        raise SystemExit('Set D1MAX_MOLA_QA=1 for the isolated lifecycle test')
    app_root = Path(__file__).resolve().parents[1]
    system = QASystemd('d1max-mola-qa-' + uuid.uuid4().hex + '.service')
    with tempfile.TemporaryDirectory(prefix='d1max-mola-cgroup-') as tmp:
        root = Path(tmp)
        bag = root / 'bag'; bag.mkdir(); (bag / 'empty.db3').touch()
        (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
            'relative_file_paths':['empty.db3'], 'topics_with_message_count':[
                {'topic_metadata':{'name':'/front_lidar','type':'sensor_msgs/msg/PointCloud2'}},
                {'topic_metadata':{'name':'/front_lidar/imu','type':'sensor_msgs/msg/Imu'}}]}}))
        runtime = MolaRuntime(root / 'runs', app_root, system=system)
        options = {'config_yaml': config_yaml(MolaConfig(input={'bag_path':str(bag)}))}
        try:
            with patch.object(runtime,'availability',return_value={'available':True}):
                first = runtime.start(options)
                deadline = time.monotonic() + 5
                group = Path('/sys/fs/cgroup') / system.show()['ControlGroup'].lstrip('/')
                while time.monotonic() < deadline and len((group / 'cgroup.procs').read_text().split()) < 2:
                    time.sleep(.05)
                assert len((group / 'cgroup.procs').read_text().split()) >= 2
                try:
                    runtime.start(options)
                    raise AssertionError('duplicate start accepted')
                except RuntimeError:
                    pass
                recovered = MolaRuntime(root / 'runs', app_root, system=system)
                assert recovered.snapshot()['id'] == first['id']
                recovered.stop()
                assert not recovered.alive(system.show())
                assert (root / 'runs' / first['id'] / 'cancel.request').exists()
                print(json.dumps({'unit':system.unit,'duplicate_start':'blocked','reattach':'passed','worker_and_child':'stopped'},indent=2))
        finally:
            runtime.close()


if __name__ == '__main__':
    main()
