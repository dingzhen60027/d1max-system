"""Shell supervision with fake children only: no ROS, ports, SDK or services."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/start_live_monitor.sh"


@pytest.fixture
def monitor(tmp_path):
    package = tmp_path / "foxglove_d1max"
    scripts = package / "scripts"
    scripts.mkdir(parents=True)
    launch = scripts / SCRIPT.name
    launch.write_bytes(SCRIPT.read_bytes())
    (tmp_path / "d1max_ros2_env.sh").write_text("")
    (package / "config").mkdir()
    tools = tmp_path / "bin"
    tools.mkdir()
    child = r'''#!/usr/bin/env bash
role="${1:?role}"
printf '%s %s\n' "$role" "$$" >> "$FAKE_PID_FILE"
if [[ "${FAIL_CRITICAL:-}" == "$role" ]]; then exit 31; fi
exec /usr/bin/sleep 60
'''
    child_path = tools / "fake_child"
    child_path.write_text(child)
    child_path.chmod(0o755)
    (tools / "ros2").write_text('#!/usr/bin/env bash\nexec fake_child router\n')
    (tools / "ros2").chmod(0o755)
    for name, role in (("start_sdk_monitor.sh", "sdk"), ("start_monitor_bridge.sh", "bridge")):
        (scripts / name).write_text(f'#!/usr/bin/env bash\nexec fake_child {role}\n')
    # The production script's two inline Python preflights are deliberately
    # replaced with stdin consumption, so this test cannot probe any network.
    (tools / "python3").write_text(r'''#!/usr/bin/env bash
if [[ "$1" == "-" ]]; then /usr/bin/cat >/dev/null; exit 0; fi
case "$1" in
  */pcd_map_publisher.py)
    if [[ "${2:-}" == "--is-enabled" ]]; then exit 0; fi
    if [[ "${FAKE_MAP_FAIL:-1}" == 1 ]]; then exit 23; fi
    exec fake_child optional_map ;;
  */monitor_health.py) exec fake_child health ;;
  *) exit 99 ;;
esac
''')
    (tools / "python3").chmod(0o755)
    processes = []
    pid_file = tmp_path / "children"
    def start(**settings):
        env = dict(os.environ, PATH=str(tools) + os.pathsep + os.environ["PATH"],
                   FAKE_PID_FILE=str(pid_file), D1MAX_MANAGED_MONITOR="1", **settings)
        log = tmp_path / "output.log"
        with log.open("w") as output:
            proc = subprocess.Popen(["bash", str(launch)], env=env,
                                    stdout=output, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        processes.append(proc)
        return proc, log, pid_file
    yield start
    for proc in processes:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)


def wait_for(check, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return
        time.sleep(.02)
    raise AssertionError("fake monitor condition did not occur")


def assert_children_gone(pid_file):
    assert pid_file.exists(), "fake critical children never started"
    for line in pid_file.read_text().splitlines():
        _, pid = line.split()
        with pytest.raises(ProcessLookupError):
            os.kill(int(pid), 0)


def test_missing_optional_pcd_warns_but_critical_processes_keep_running(monitor):
    proc, log, pids = monitor()
    wait_for(lambda: "[WARN] 可选静态 PCD" in log.read_text())
    assert proc.poll() is None
    assert "status=23" in log.read_text()
    assert {line.split()[0] for line in pids.read_text().splitlines()} == {
        "router", "sdk", "bridge", "health"}
    proc.terminate()
    assert proc.wait(timeout=3) == 0
    assert_children_gone(pids)


@pytest.mark.parametrize("role", ["router", "sdk", "bridge", "health"])
def test_critical_startup_failure_retains_exit_status_and_cleans_every_child(monitor, role):
    # Immediate failure also exercises jobs that have already exited before
    # the parent begins supervising; wait -n alone can overlook those jobs.
    proc, log, pids = monitor(FAIL_CRITICAL=role, FAKE_MAP_FAIL="0")
    assert proc.wait(timeout=3) == 31
    assert "[ERROR] 关键实时进程退出" in log.read_text()
    assert_children_gone(pids)


def test_signal_cleanup_includes_a_running_optional_map(monitor):
    proc, log, pids = monitor(FAKE_MAP_FAIL="0")
    wait_for(lambda: pids.exists() and len(pids.read_text().splitlines()) == 5)
    assert proc.poll() is None
    proc.terminate()
    assert proc.wait(timeout=3) == 0
    assert_children_gone(pids)
