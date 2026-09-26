"""Exercise the actual budgeted native worker, offline with real saved endpoints.

No ROS/SDK, no synthetic map or obstacle. Only the owned spawned process is
stopped. Affinity is inspected from the parent after native pools have started.
"""
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time

WS = Path('/home/dndx/d1max_nav_ws')
sys.path[:0] = [str(WS/'src/d1max_pct_planner'),str(WS/'src/d1max_pct_scan')]


def main():
    import yaml
    from d1max_pct_planner.compute_budget import budgeted_worker, spawn_with_environment, load_budget
    from d1max_pct_planner.native_runtime import prepare_native_environment
    from d1max_pct_scan.native_global_worker import native_worker
    root = WS/'maps/processed/sc_pgo_20260923_crossfloor_complete_001'
    tomo, route = root/'pct/tomogram.npz', root/'route.yaml'
    cfg = yaml.safe_load(route.read_text())
    saved = json.loads((root/'route_001/audit.json').read_text())
    options = dict(unknown_ceiling_policy=cfg['unknown_ceiling_policy'],
                   max_ground_step_m=cfg['limits']['max_ground_step_m'])
    env = prepare_native_environment(cfg['vendor_root'])
    budget = load_budget(env)
    expected = set(budget['cpu_ids'] or sorted(os.sched_getaffinity(0))[:budget['max_cpu_cores']])
    parent_affinity, parent_nice = os.sched_getaffinity(0), os.getpriority(os.PRIO_PROCESS,0)
    ctx = multiprocessing.get_context('spawn')
    parent, child = ctx.Pipe()
    worker = ctx.Process(target=budgeted_worker, args=(native_worker,child,str(tomo),str(route),options,
        hashlib.sha256(tomo.read_bytes()).hexdigest(),hashlib.sha256(route.read_bytes()).hexdigest()))
    spawn_with_environment(worker,env)
    child.close()
    runs = []
    try:
        for generation in (1,2):
            started = time.monotonic()
            parent.send(dict(kind='plan',generation=generation,
                start_xyz=saved['path_xyz'][0],goal_xyz=saved['path_xyz'][-1],
                start_layer=saved['layer_ids'][0],goal_layer=saved['layer_ids'][-1]))
            while True:
                if not parent.poll(max(.01,25-(time.monotonic()-started))):
                    raise RuntimeError('Offline worker reply deadline')
                packet = parent.recv()
                if packet['kind'] != 'progress':
                    break
                if time.monotonic()-started > 25:
                    raise RuntimeError('Offline worker progress deadline')
            assert packet['kind'] == 'planned', packet
            assert packet['generation'] == generation
            threads = {p.name:dict(cpu_ids=sorted(os.sched_getaffinity(int(p.name))),
                       nice=os.getpriority(os.PRIO_PROCESS,int(p.name)))
                       for p in Path(f'/proc/{worker.pid}/task').iterdir()}
            assert all(set(t['cpu_ids']).issubset(expected) and t['nice']>=budget['nice']
                       for t in threads.values()), threads
            status = Path(f'/proc/{worker.pid}/status').read_text().splitlines()
            memory = {line.split(':',1)[0]:line.split(':',1)[1].strip()
                      for line in status if line.startswith(('VmRSS:','VmHWM:'))}
            result = packet['result']
            import numpy as np
            geometry = hashlib.sha256(np.asarray(result['path'],dtype=np.float64).tobytes()
                       +np.asarray(result['layer_ids'],dtype=np.int64).tobytes()).hexdigest()
            assert geometry == '46a273a2c8ce4b2c41fe9774659f2f7d6620e3df8d42e46ba001b330b651720f'
            runs.append(dict(generation=generation,elapsed_s=time.monotonic()-started,
                worker_elapsed_s=packet['worker_elapsed_sec'],threads=threads,memory=memory,
                geometry_sha256=geometry,native_map_cache=result['native_map_cache']))
        parent.send({'kind':'shutdown'})
        worker.join(5)
        assert worker.exitcode == 0
        assert os.sched_getaffinity(0) == parent_affinity
        assert os.getpriority(os.PRIO_PROCESS,0) == parent_nice
        report = dict(no_ros_or_sdk=True,budget=budget,parent_unchanged=True,
                      clean_worker_exit=True,runs=runs)
        output = Path(__file__).parent/'final/worker_budget.json'
        output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        if worker.is_alive():
            worker.terminate()
            worker.join(5)
        parent.close()


if __name__ == '__main__':
    main()
