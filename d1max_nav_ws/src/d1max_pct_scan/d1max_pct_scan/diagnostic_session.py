"""Best-effort trace index, not a health check or another navigation owner."""
import hashlib
import json
import os
from pathlib import Path
import time


def trace_environment(directory, session, commands, *, environment=None):
    """One independent bounded trace directory per launch, even after retries.

    No bag, subscription, SDK query or thread is created. Failure leaves the
    child environment unchanged, allowing each sink's own bounded fallback.
    """
    env = dict(os.environ if environment is None else environment)
    try:
        directory = Path(directory)
        traces = directory/'diagnostics'/('launch-'+str(time.time_ns())+'-'+str(os.getpid()))
        traces.mkdir(mode=0o700, parents=True, exist_ok=False)
        files = {}
        for name in ('session.json', 'runtime_bundle.json', 'tracker.yaml', 'scan.yaml', 'reference.yaml',
                     'bt.yaml', 'localization.yaml'):
            path = directory/name
            # Metadata/config only, never hash an entire point cloud at launch.
            if path.is_file() and path.stat().st_size <= 16*1024*1024:
                files[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        index = dict(schema=1, session_id=session.get('id'), created_unix_ns=time.time_ns(),
            expected_sdk_session=session.get('expected_sdk_session'),
            map_version_id=session.get('version_id'), transport_mode=session.get('transport_mode'),
            session_directory=str(directory), component_logs={name:str(directory/(name+'.log')) for name in commands},
            component_trace_limit_bytes=4*(4*1024*1024+64*1024), files=files,
            task_trace_retention=dict(task_buckets=4, bytes_per_task=4*1024*1024,
                idle_bytes=256*1024, post_task_seconds=10,
                task_identity='length-framed session and task; not curve/reference generation',
                eviction='new active task only; idle never evicts completed task history',
                layout='component task-index.json and bounded task/idle trace subdirectories'),
            sdk_trace_location='SDKCommandTraceLocation in existing SDK log; same SDK session may span navigation launches',
            clocks='payload source stamps are original; sink write time is not acquisition time',
            note='commands and SDK ACK are not proof of physical motion; no raw cloud or rosbag recorded')
        (traces/'session-index.json').write_text(json.dumps(index, ensure_ascii=False, indent=2)+'\n')
        env['D1MAX_TRACE_DIRECTORY'] = str(traces)
    except (OSError, ValueError, TypeError):
        pass  # diagnostics never blocks starting or stopping navigation
    return env
