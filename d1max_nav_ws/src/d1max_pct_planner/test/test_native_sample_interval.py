"""The spacing knob changes the planner-owned optimizer, not a Python copy."""
from pathlib import Path
import subprocess
import sys

import pytest

from d1max_pct_planner.native_runtime import prepare_native_environment


VENDOR = Path(__file__).resolve().parents[2] / 'pct_planner_vendor'


@pytest.mark.skipif(not (VENDOR / 'planner/lib/ele_planner.cpython-310-x86_64-linux-gnu.so').is_file(),
                    reason='Native PCT extension is not built')
def test_native_owned_optimizer_responds_to_spacing_and_bounds():
    script = r'''
import numpy as np
from d1max_pct_planner.planner_core import TomogramPlanner
data = np.zeros((5, 1, 61, 61), np.float32)
data[4] = 2
payload = dict(data=data, resolution=.1, center=np.zeros(2), slice_h0=.5, slice_dh=.5)
counts = []
for interval in (10, 2):
    planner = TomogramPlanner(VENDOR, ground_z=True, optimizer_sample_interval=interval)
    planner.load_payload(payload)
    result = planner.plan([-1.5, 0], [1.5, 0], return_details=True)
    counts.append(len(result['path']))
    assert np.isfinite(result['path']).all()
    assert planner.native_parameters['astar_cost_threshold'] == 20
    for bad in (0, -1, 101):
        try:
            planner.planner.set_optimizer_sample_interval(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('native setter accepted invalid interval')
assert counts[1] > counts[0] * 3, counts
print('PASS actual native optimizer spacing', counts)
'''
    result = subprocess.run([sys.executable, '-c', 'VENDOR=' + repr(str(VENDOR)) + '\n' + script],
                            env=prepare_native_environment(VENDOR), capture_output=True,
                            text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'PASS actual native optimizer spacing' in result.stdout
