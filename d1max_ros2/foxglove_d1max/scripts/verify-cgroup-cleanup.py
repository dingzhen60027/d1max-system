#!/usr/bin/env python3
"""Opt-in isolated cgroup test. No ROS, SDK, robot, maps or production service changes."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

if os.environ.get("D1MAX_LIFECYCLE_QA") != "1":
    raise SystemExit("Set D1MAX_LIFECYCLE_QA=1 for the isolated child-cleanup test")
for unit in ("d1max-monitor-managed.service", "d1max-web-managed.service"):
    mode = subprocess.check_output(["systemctl", "--user", "show", unit, "-p", "KillMode", "--value"], text=True).strip()
    assert mode == "control-group", (unit, mode)
unit = "d1max-cleanup-test-" + uuid.uuid4().hex[:12] + ".service"
child = "import os,signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); pid=os.fork(); os.setsid() if pid==0 else None; time.sleep(120)"
def show():
    result = subprocess.check_output(["systemctl", "--user", "show", unit, "-p", "ActiveState,ControlGroup,MainPID"], text=True)
    return dict(line.split("=",1) for line in result.splitlines() if "=" in line)
try:
    subprocess.run(["systemd-run", "--user", "--quiet", "--unit="+unit, "--property=KillMode=control-group",
                    "--property=KillSignal=SIGINT", "--property=TimeoutStopSec=2s",
                    "--property=SendSIGKILL=yes", "/usr/bin/python3", "-c", child], check=True)
    end = time.monotonic() + 5
    pids = []
    while time.monotonic() < end:
        state = show()
        group = Path("/sys/fs/cgroup") / state["ControlGroup"].lstrip("/")
        if state["ControlGroup"] and (group / "cgroup.procs").is_file():
            pids = (group / "cgroup.procs").read_text().split()
            if len(pids) >= 2:
                break
        time.sleep(.05)
    assert len(pids) == 2, pids
    # Child has its own session and both processes ignore graceful termination.
    assert len({os.getsid(int(pid)) for pid in pids}) == 2
    start = time.monotonic()
    subprocess.run(["systemctl", "--user", "stop", unit], check=True, timeout=12)
    state = show()
    assert state["MainPID"] == "0", state
    assert not state["ControlGroup"], state
    assert all(not Path("/proc").joinpath(pid).exists() for pid in pids)
    report = {"unit":unit, "separate_sessions":True, "ignored_sigint_sigterm":True,
              "cgroup_empty":True, "stop_seconds":round(time.monotonic()-start,2), "robot_commands":0}
    print(json.dumps(report))
finally:
    subprocess.run(["systemctl", "--user", "stop", unit], capture_output=True, timeout=12)
    subprocess.run(["systemctl", "--user", "reset-failed", unit], capture_output=True, timeout=5)
