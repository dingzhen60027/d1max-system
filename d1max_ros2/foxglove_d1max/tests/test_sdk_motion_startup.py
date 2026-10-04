"""Offline startup handoff tests: manager, ROS, SDK exec and sockets are fakes."""
import importlib.util
import fcntl
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest


MODULE = Path(__file__).resolve().parents[1] / "scripts/sdk_motion_startup.py"
spec = importlib.util.spec_from_file_location("sdk_motion_startup", MODULE)
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)
ManagerClientClass = startup.ManagerClient
INSTANCE = "a" * 32
INVOCATION = "b" * 32


def test_joint_telemetry_audit_disables_takeover_without_changing_normal_preference(monkeypatch):
    monkeypatch.setenv('D1MAX_JOINT_TELEMETRY_AUDIT', 'true')
    arguments = startup.sdk_arguments(False)
    assert 'joint_state_enabled:=true' in arguments
    assert 'auto_take_control_on_available:=false' in arguments
    assert 'navigation_control_enabled:=false' in arguments
    with pytest.raises(RuntimeError, match='cannot consume'):
        startup.sdk_arguments(True)
    monkeypatch.delenv('D1MAX_JOINT_TELEMETRY_AUDIT')
    assert 'joint_state_enabled:=true' not in startup.sdk_arguments(False)
    assert 'auto_take_control_on_available:=false' not in startup.sdk_arguments(False)


def test_joint_telemetry_audit_rejects_ambiguous_option(monkeypatch):
    monkeypatch.setenv('D1MAX_JOINT_TELEMETRY_AUDIT', '1')
    with pytest.raises(RuntimeError, match='Invalid'):
        startup.sdk_arguments(False)


def snapshot(phase="stopped", **extra):
    return {"instance": INSTANCE, "motion_start_binding": 1, "monitor": {"phase": phase, "active": False,
            "owned": False, "busy": False, **extra}}


def binding(ticket):
    return {"request_id": "motion_" + ticket["nonce"], "invocation": INVOCATION}


class FakeManager:
    config = {"monitor_port": 8769, "start_timeout_seconds": .1}

    def __init__(self):
        self.value = snapshot()
        self.starts = []

    def status(self):
        return self.value

    def start(self, instance, request_id):
        self.starts.append((instance, request_id))
        ticket = startup.read_private_json(startup.session_directory() / startup.TICKET_NAME)
        startup.cancel_ticket(ticket["nonce"])
        receipt = startup.session_directory() / startup.RECEIPT_NAME
        receipt.write_text(json.dumps({"nonce": ticket["nonce"], "startup_disarmed": True,
                                       "invocation": INVOCATION}))
        receipt.chmod(0o600)
        self.value = snapshot("connected", active=True, owned=True, last_start=binding(ticket))
        return self.value


@pytest.fixture
def isolated(tmp_path, monkeypatch):
    runtime = tmp_path / "runtime"
    runtime.mkdir(mode=0o700)
    session = runtime / "d1max-session"
    session.mkdir(mode=0o700)
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(runtime))
    monkeypatch.setattr(startup, "core_lease_path", lambda: runtime / "d1max-sdk-monitor.lock")
    monkeypatch.delenv("D1MAX_NAVIGATION_CONTROL_ENABLED", raising=False)
    monkeypatch.setattr(startup, "local_sdk_processes", lambda: [])
    monkeypatch.setattr(startup, "port_in_use", lambda _port: False)
    monkeypatch.setattr(startup, "ros_duplicate_preflight", lambda: None)
    manager = FakeManager()
    monkeypatch.setattr(startup, "ManagerClient", lambda: manager)
    return manager, session


def test_check_and_status_have_no_ticket_or_lifecycle_side_effects(isolated, monkeypatch, capsys):
    manager, session = isolated
    for action in ("check", "status"):
        monkeypatch.setattr(startup.sys, "argv", ["sdk_motion_startup.py", action])
        startup.main()
        output = json.loads(capsys.readouterr().out)
        assert output["pending_ticket"] is False
        assert output["motion_start_limits"] == startup.LIMITS
    assert list(session.iterdir()) == []
    assert manager.starts == []


@pytest.mark.parametrize("phase,extra", [("connected", {"active": True, "owned": True}),
    ("degraded", {"active": True, "owned": True}), ("starting", {"busy": True}),
    ("stopping", {"busy": True}), ("failed", {"owned": True}), ("conflict", {})])
def test_existing_monitor_never_restarted_or_enabled(isolated, phase, extra):
    manager, session = isolated
    manager.value = snapshot(phase, **extra)
    with pytest.raises(RuntimeError, match="现有 manager 停止"):
        startup.start_managed(manager)
    assert manager.starts == []
    assert not (session / startup.TICKET_NAME).exists()


@pytest.mark.parametrize("kind", ["process", "port"])
def test_unmanaged_sdk_or_port_conflict_is_not_adopted(isolated, monkeypatch, kind):
    manager, _ = isolated
    if kind == "process":
        monkeypatch.setattr(startup, "local_sdk_processes", lambda: [123])
    else:
        monkeypatch.setattr(startup, "port_in_use", lambda port: port == 7448)
    with pytest.raises(RuntimeError, match="拒绝重复连接"):
        startup.start_managed(manager)
    assert manager.starts == []


def test_prepare_is_private_short_lived_and_never_starts(isolated):
    manager, session = isolated
    ticket = startup.create_ticket(manager)
    path = session / startup.TICKET_NAME
    assert path.stat().st_mode & 0o777 == 0o600
    assert ticket["manager_instance"] == INSTANCE
    assert ticket["limits"] == {"navigation_max_forward_mps": .3,
        "navigation_max_lateral_mps": 0., "navigation_max_yaw_radps": .5}
    assert startup.TTL_SECONDS == 60
    assert manager.starts == []
    with pytest.raises(RuntimeError, match="已有待消费"):
        startup.create_ticket(manager)
    assert startup.cancel_ticket(ticket["nonce"])
    assert not path.exists()


def test_old_running_manager_is_rejected_before_ticket_or_start(isolated):
    manager, session = isolated
    manager.value.pop("motion_start_binding")
    with pytest.raises(RuntimeError, match="手动重启 manager"):
        startup.start_managed(manager)
    assert manager.starts == []
    assert not (session / startup.TICKET_NAME).exists()


def test_start_uses_existing_manager_and_confirms_only_startup_profile(isolated):
    manager, session = isolated
    result = startup.start_managed(manager)
    assert result == {"motion_capable": True, "startup_disarmed": True,
                      "monitor_phase": "connected", "limits": startup.LIMITS}
    assert len(manager.starts) == 1
    assert manager.starts[0][0] == INSTANCE
    assert manager.starts[0][1].startswith("motion_")
    assert not (session / startup.TICKET_NAME).exists()


def test_prepared_ticket_is_reused_only_by_explicit_start(isolated):
    manager, _ = isolated
    prepared = startup.create_ticket(manager)
    startup.start_managed(manager)
    assert manager.starts == [(INSTANCE, "motion_" + prepared["nonce"])]


def test_prepared_ticket_is_cleared_if_start_finds_newly_active_monitor(isolated):
    manager, session = isolated
    startup.create_ticket(manager)
    manager.value = snapshot("connected", active=True, owned=True)
    with pytest.raises(RuntimeError, match="现有 manager 停止"):
        startup.start_managed(manager)
    assert not (session / startup.TICKET_NAME).exists()
    assert manager.starts == []


def test_manager_job_can_finish_before_sdk_consumes_ticket(isolated, monkeypatch):
    manager, session = isolated
    consumed = []
    def start(instance, request_id):
        manager.starts.append((instance, request_id))
        ticket = startup.read_private_json(session / startup.TICKET_NAME)
        manager.value = snapshot("degraded", active=True, owned=True,
                                 operation=None, last_start=binding(ticket))
    def sdk_after_preflight(_delay):
        # SDK preflight finishes only after manager has already returned idle.
        assert manager.value["monitor"]["operation"] is None
        ticket = startup.consume_ticket()
        consumed.append(ticket)
        path = session / startup.RECEIPT_NAME
        path.write_text(json.dumps({"nonce": ticket["nonce"], "startup_disarmed": True,
                                    "invocation": ticket["invocation"]}))
        path.chmod(0o600)
    monkeypatch.setattr(manager, "start", start)
    monkeypatch.setattr(startup, "managed_invocation", lambda: INVOCATION)
    monkeypatch.setattr(startup.time, "sleep", sdk_after_preflight)
    assert startup.start_managed(manager)["motion_capable"] is True
    assert len(consumed) == 1
    assert not (session / startup.TICKET_NAME).exists()


def test_real_manager_ui_start_wins_and_explicit_noop_cannot_consume(isolated, monkeypatch):
    # Use the actual Manager history logic, but fake systemd/ports/health. No
    # HTTP, ROS, systemctl, SDK child or robot connection is created.
    from manager.runtime import Manager
    _fake, session = isolated
    config = json.loads((startup.BASE / "config/manager.json").read_text())
    config.update(start_timeout_seconds=.1, stop_timeout_seconds=.1)
    class System:
        calls = []
        state = {"ActiveState": "inactive", "MainPID": "0"}
        def show(self, unit):
            return dict(self.state) if unit == startup.UNIT else {"ActiveState": "inactive"}
        def populated(self, state):
            return state.get("ActiveState") == "active"
        def action(self, unit, verb):
            self.calls.append((unit, verb))
            assert verb == "start"
            self.state = {"ActiveState": "active", "MainPID": "123", "InvocationID": INVOCATION}
    system = System()
    actual = Manager(config, session, system=system,
        probe=lambda host, port, *_: host != "127.0.0.1" or (port == 8769 and system.state["ActiveState"] == "active"),
        network_check=lambda _config: "")
    monkeypatch.setattr(actual, "_health", lambda invocation: {"invocation": invocation})
    class Client:
        def __init__(self):
            self.config = config
        def status(self):
            return actual.snapshot()
        def start(self, instance, request_id):
            actual.request("monitor", "start", "ordinary_ui_request_012345", instance)
            for worker in actual.threads:
                worker.join(1)
                assert not worker.is_alive()
            # Same API behavior: start on active unit is accepted as no-op.
            actual.request("monitor", "start", request_id, instance)
            assert actual.jobs["monitor"] is None
            assert actual.snapshot()["monitor"]["last_start"]["request_id"] == "ordinary_ui_request_012345"
            with pytest.raises(RuntimeError, match="不是本票据绑定"):
                startup.consume_ticket()
            return actual.snapshot()
    client = Client()
    monkeypatch.setattr(startup, "ManagerClient", lambda: client)
    monkeypatch.setattr(startup, "managed_invocation", lambda: INVOCATION)
    with pytest.raises(RuntimeError, match="并非本次显式请求启动"):
        startup.start_managed(client)
    assert system.calls == [(startup.UNIT, "start")]
    assert not (session / startup.TICKET_NAME).exists()


def test_existing_authenticated_api_schema_is_preserved_without_network(isolated, tmp_path, monkeypatch):
    package = tmp_path / "package"
    config = package / "config"
    config.mkdir(parents=True)
    (config / "manager.json").write_text(json.dumps({"host": "127.0.0.1", "port": 8771,
        "monitor_unit": startup.UNIT}))
    credential = config / "manager.local.json"
    credential.write_text(json.dumps({"token": "test-only-capability-" * 3}))
    credential.chmod(0o600)
    before = credential.read_bytes()
    monkeypatch.setattr(startup, "BASE", package)
    requests = []
    class Reply:
        def __enter__(self):
            import io
            return io.StringIO(json.dumps(snapshot()))
        def __exit__(self, *_args):
            pass
    class FakeOpener:
        def open(self, request, **_kwargs):
            requests.append(request)
            return Reply()
    monkeypatch.setattr(startup, "build_opener", lambda *_args: FakeOpener())
    client = ManagerClientClass()
    client.status()
    client.start(INSTANCE, "motion_" + "d" * 32)
    assert [request.full_url for request in requests] == [
        "http://127.0.0.1:8771/v1/status", "http://127.0.0.1:8771/v1/monitor/start"]
    assert [request.method for request in requests] == ["GET", "POST"]
    assert set(json.loads(requests[1].data)) == {"instance", "request_id"}
    assert requests[1].get_header("Authorization").startswith("Bearer ")
    assert credential.read_bytes() == before


@pytest.mark.parametrize("failure", ["post", "failed", "unconsumed", "new_instance", "timeout"])
def test_failed_or_uncertain_start_cleans_ticket_without_retry_stop_restart(isolated, monkeypatch, failure):
    manager, session = isolated
    def fail_start(instance, request_id):
        manager.starts.append((instance, request_id))
        if failure == "post":
            raise RuntimeError("request uncertain")
        manager.value = snapshot("failed") if failure == "failed" else snapshot("starting", busy=True)
        if failure == "unconsumed":
            manager.value = snapshot("connected", active=True, owned=True)
        if failure == "new_instance":
            manager.value["instance"] = "c" * 32
        if failure == "timeout":
            manager.config = {**manager.config, "start_timeout_seconds": -5}
    monkeypatch.setattr(manager, "start", fail_start)
    with pytest.raises(RuntimeError):
        startup.start_managed(manager)
    assert len(manager.starts) == 1
    assert not (session / startup.TICKET_NAME).exists()


@pytest.mark.parametrize("failure", ["expired", "wrong_instance", "wrong_package", "unmanaged", "wrong_request", "wrong_invocation"])
def test_invalid_ticket_is_rejected_and_consumed_once(isolated, monkeypatch, failure):
    manager, session = isolated
    ticket = startup.create_ticket(manager)
    manager.value["monitor"]["last_start"] = binding(ticket)
    monkeypatch.setattr(startup, "managed_invocation", lambda: INVOCATION)
    if failure == "expired":
        ticket["created_monotonic"] -= 61
    elif failure == "wrong_package":
        ticket["package"] = "/different/package"
    elif failure == "wrong_instance":
        manager.value["instance"] = "c" * 32
    elif failure == "unmanaged":
        def reject():
            raise RuntimeError("not managed")
        monkeypatch.setattr(startup, "managed_invocation", reject)
    elif failure == "wrong_request":
        manager.value["monitor"]["last_start"]["request_id"] = "ordinary_ui_connect_request"
    else:
        manager.value["monitor"]["last_start"]["invocation"] = "c" * 32
    path = session / startup.TICKET_NAME
    path.write_text(json.dumps(ticket))
    with pytest.raises(RuntimeError):
        startup.consume_ticket()
    assert not path.exists()


def test_strict_managed_consumption_requires_env_cgroup_and_matching_invocation(isolated, monkeypatch):
    monkeypatch.delenv("D1MAX_MANAGED_MONITOR", raising=False)
    with pytest.raises(RuntimeError, match="受管 monitor"):
        startup.managed_invocation()
    monkeypatch.setenv("D1MAX_MANAGED_MONITOR", "1")
    monkeypatch.setenv("INVOCATION_ID", INVOCATION)
    original = Path.read_text
    group = ["0::/some-other.service\n"]
    monkeypatch.setattr(Path, "read_text", lambda path, *a, **kw:
        group[0] if str(path) == "/proc/self/cgroup" else original(path, *a, **kw))
    with pytest.raises(RuntimeError, match="cgroup"):
        startup.managed_invocation()
    group[0] = "0::/user.slice/" + startup.UNIT + "\n"
    value = ["c" * 32]
    monkeypatch.setattr(startup.subprocess, "run", lambda *a, **kw: SimpleNamespace(stdout=value[0]))
    with pytest.raises(RuntimeError, match="invocation"):
        startup.managed_invocation()
    value[0] = INVOCATION
    assert startup.managed_invocation() == INVOCATION


def test_sdk_exec_is_locked_and_motion_default_is_false(isolated, monkeypatch):
    seen = []
    def fake_exec(binary, args):
        seen.append((binary, args))
        with pytest.raises(RuntimeError, match="已有 monitor"):
            with startup.startup_lock():
                pass
        # Simulates SDK main's independent open after exec, with the launcher
        # descriptor still open. Different lock paths prevent self-rejection.
        path = startup.runtime_directory() / "d1max-sdk-monitor.lock"
        sdk_fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(sdk_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with pytest.raises(RuntimeError, match="已有 monitor"):
                startup.require_core_lease_available()
        finally:
            os.close(sdk_fd)
    monkeypatch.setattr(startup.os, "execv", fake_exec)
    startup.launch_sdk()
    assert len(seen) == 1
    args = seen[0][1]
    assert "navigation_control_enabled:=false" in args
    assert "navigation_max_forward_mps:=0.3" in args
    assert "navigation_max_lateral_mps:=0.0" in args
    assert "navigation_max_yaw_radps:=0.5" in args


def test_core_session_lease_blocks_prepare_without_mutating_lock(isolated):
    manager, session = isolated
    path = startup.runtime_directory() / "d1max-sdk-monitor.lock"
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match="已有 monitor"):
            startup.create_ticket(manager)
        assert manager.starts == []
        assert not (session / startup.TICKET_NAME).exists()
    finally:
        os.close(fd)
    startup.require_core_lease_available()
    assert path.read_bytes() == b""


def test_valid_managed_ticket_enables_same_exec_without_arm_or_mode_actions(isolated, monkeypatch):
    manager, session = isolated
    ticket = startup.create_ticket(manager)
    manager.value = snapshot("starting", busy=True, owned=True, last_start=binding(ticket))
    monkeypatch.setattr(startup, "managed_invocation", lambda: INVOCATION)
    seen = []
    monkeypatch.setattr(startup.os, "execv", lambda binary, args: seen.append(args))
    startup.launch_sdk()
    assert len(seen) == 1
    assert "navigation_control_enabled:=true" in seen[0]
    assert all(not any(word in arg for word in ("StandUp", "SetMode", "arm:=", "estop")) for arg in seen[0])
    assert not (session / startup.TICKET_NAME).exists()
    assert startup.read_private_json(session / startup.RECEIPT_NAME)["startup_disarmed"] is True


@pytest.mark.parametrize("failure", ["preflight", "exec", "environment"])
def test_prepared_ticket_does_not_survive_launch_failure(isolated, monkeypatch, failure):
    manager, session = isolated
    ticket = startup.create_ticket(manager)
    manager.value = snapshot("starting", busy=True, owned=True, last_start=binding(ticket))
    monkeypatch.setattr(startup, "managed_invocation", lambda: INVOCATION)
    def fail(*_args):
        raise RuntimeError("offline fake failure")
    if failure == "preflight":
        monkeypatch.setattr(startup, "ros_duplicate_preflight", fail)
    elif failure == "exec":
        monkeypatch.setattr(startup.os, "execv", fail)
    else:
        monkeypatch.setenv("D1MAX_NAVIGATION_CONTROL_ENABLED", "true")
    with pytest.raises(RuntimeError):
        startup.launch_sdk()
    assert not (session / startup.TICKET_NAME).exists()


def test_private_ticket_and_lock_reject_symlinks(isolated, tmp_path):
    manager, session = isolated
    target = tmp_path / "untouched"
    target.write_text("unchanged")
    path = session / startup.TICKET_NAME
    path.symlink_to(target)
    with pytest.raises(RuntimeError, match="已有待消费"):
        startup.create_ticket(manager)
    with pytest.raises(OSError):
        startup.read_private_json(path)
    assert target.read_text() == "unchanged"


def sealed_release(tmp_path, *, seal_binary=True):
    import hashlib
    release = tmp_path / "release"
    binary = release / startup.RELEASE_MONITOR
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b"schema3-monitor")
    descriptor = release / "release.json"
    descriptor.write_text(json.dumps({"schema": 1, "sealed_manifest": "manifest.json"}))
    sealed = [descriptor] + ([binary] if seal_binary else [])
    files = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sealed}
    (release / "manifest.json").write_text(json.dumps({"files": files}))
    return release, binary


IDENTITY = {"D1MAX_EXECUTION_ACCEPTANCE_RECORD": "/data/acceptance/record.json",
            "D1MAX_EXECUTION_ROBOT_ID": "0123", "D1MAX_EXECUTION_SDK_VERSION": "1.2.0",
            "D1MAX_EXECUTION_CALIBRATION_SHA256": "c" * 64, "D1MAX_EXECUTION_ROBOT_PROFILE_SHA256": "d" * 64}


def test_release_monitor_is_opt_in_and_default_is_unchanged(monkeypatch):
    monkeypatch.delenv(startup.RELEASE_ENV, raising=False)
    args = startup.sdk_arguments(False)
    assert args[0].endswith("sdk_bridge_ws/install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge")
    assert not any(arg.startswith("execution_") for arg in args)


def test_release_monitor_uses_sealed_schema3_binary_without_acceptance(tmp_path, monkeypatch):
    release, binary = sealed_release(tmp_path)
    monkeypatch.setenv(startup.RELEASE_ENV, str(release))
    for variable in IDENTITY:
        monkeypatch.delenv(variable, raising=False)
    args = startup.sdk_arguments(False)
    assert args[0] == str(binary)
    assert "execution_v3_enabled:=true" in args
    assert "navigation_control_enabled:=false" in args
    # No identity means an unaccepted channel; no record is invented here.
    assert not any(arg.startswith(("execution_acceptance_record", "execution_robot_id")) for arg in args)


def test_release_monitor_rejects_legacy_motion_ticket(tmp_path, monkeypatch):
    release, _ = sealed_release(tmp_path)
    monkeypatch.setenv(startup.RELEASE_ENV, str(release))
    with pytest.raises(RuntimeError, match="互斥"):
        startup.sdk_arguments(True)


@pytest.mark.parametrize("change", ["binary", "unsealed", "relative", "manifest_outside"])
def test_release_monitor_refuses_changed_or_unsealed_release(tmp_path, monkeypatch, change):
    release, binary = sealed_release(tmp_path, seal_binary=change != "unsealed")
    value = str(release)
    if change == "binary":
        binary.write_bytes(b"replaced")
    elif change == "relative":
        value = "release"
    elif change == "manifest_outside":
        (release / "release.json").write_text(json.dumps({"schema": 1, "sealed_manifest": "../manifest.json"}))
    monkeypatch.setenv(startup.RELEASE_ENV, value)
    with pytest.raises(RuntimeError):
        startup.sdk_arguments(False)


def test_release_monitor_binds_all_five_identities_as_strings(tmp_path, monkeypatch):
    release, _ = sealed_release(tmp_path)
    monkeypatch.setenv(startup.RELEASE_ENV, str(release))
    for variable, value in IDENTITY.items():
        monkeypatch.setenv(variable, value)
    args = startup.sdk_arguments(False)
    assert 'execution_robot_id:="0123"' in args
    assert 'execution_sdk_version:="1.2.0"' in args
    assert 'execution_acceptance_record:="/data/acceptance/record.json"' in args


@pytest.mark.parametrize("variable,value", [("D1MAX_EXECUTION_ROBOT_ID", ""),
                                            ("D1MAX_EXECUTION_CALIBRATION_SHA256", "C" * 64),
                                            ("D1MAX_EXECUTION_ACCEPTANCE_RECORD", "relative.json"),
                                            ("D1MAX_EXECUTION_ROBOT_ID", "a b")])
def test_release_monitor_rejects_partial_or_malformed_identity(tmp_path, monkeypatch, variable, value):
    release, _ = sealed_release(tmp_path)
    monkeypatch.setenv(startup.RELEASE_ENV, str(release))
    for name, good in IDENTITY.items():
        monkeypatch.setenv(name, good)
    monkeypatch.setenv(variable, value)
    with pytest.raises(RuntimeError, match=variable):
        startup.sdk_arguments(False)
