#!/usr/bin/env python3
"""Production Monitor + fake SDK, private Unix IPC and loopback-only Zenoh.

This checks software calls/callbacks, not measured robot movement or braking.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--router", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--post-estop-zero-failure", action="store_true")
    args = ap.parse_args()
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    # No discovery, gossip or non-loopback endpoint; this process cannot join
    # the user's live navigation domain/router.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0)); port = probe.getsockname()[1]
    base = {"scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
            "timestamping": {"enabled": True, "drop_future_timestamp": False}}
    router = dict(base, mode="router", listen={"endpoints": [f"tcp/127.0.0.1:{port}"]}, connect={"endpoints": []})
    client = dict(base, mode="client", listen={"endpoints": []}, connect={"endpoints": [f"tcp/127.0.0.1:{port}"]})
    (out / "router.json5").write_text(json.dumps(router)); (out / "client.json5").write_text(json.dumps(client))
    env = dict(os.environ, RMW_IMPLEMENTATION="rmw_zenoh_cpp", ROS_DOMAIN_ID="229",
               ZENOH_ROUTER_CONFIG_URI=str(out / "router.json5"), ZENOH_SESSION_CONFIG_URI=str(out / "client.json5"),
               D1MAX_SHARED_MONITOR_TEST="1", D1MAX_MOCK_DELAY_AFTER_STAND_MS="700")
    env.pop("ZENOH_CONFIG_OVERRIDE", None)
    if args.post_estop_zero_failure:
        env["D1MAX_MOCK_FAIL_POST_ESTOP_ZERO"] = "1"
    with tempfile.TemporaryDirectory(prefix="d1max-shared-monitor-") as temporary:
        ipc = str(Path(temporary) / "manual.sock")
        with (out / "router.log").open("w") as router_log, (out / "monitor.log").open("w") as monitor_log:
            routing = subprocess.Popen([args.router], env=env, stdout=router_log, stderr=subprocess.STDOUT)
            process = None; connection = None
            try:
                time.sleep(.5)
                assert routing.poll() is None, "isolated router startup failed"
                process = subprocess.Popen([args.binary, "--ros-args", "-p", "robot_ip:=127.0.0.1",
                    "-p", "execution_task_writer_enabled:=true", "-p", "auto_take_control_on_available:=false",
                    "-p", f"shared_estop_ipc_path:={ipc}"], env=env, stdout=monitor_log, stderr=subprocess.STDOUT)
                due = time.monotonic() + 10
                while not Path(ipc).exists():
                    assert process.poll() is None, "mock monitor exited"
                    assert time.monotonic() < due, "IPC startup deadline"
                    time.sleep(.05)
                connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); connection.connect(ipc); connection.settimeout(.2)
                buffer = bytearray(); session = ""; sequence = 0; latest = {}; messages = []

                def send(value):
                    connection.sendall((json.dumps(value) + "\n").encode())

                def receive_until(predicate, timeout=10):
                    nonlocal latest, session
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        if b"\n" in buffer:
                            raw, _, tail = buffer.partition(b"\n"); buffer[:] = tail
                            value = json.loads(raw); messages.append(value)
                            if value.get("type") in ("hello", "state"):
                                latest = value; session = value["sdk_session"]
                            if predicate(value): return value
                            continue
                        try:
                            data = connection.recv(65536)
                            assert data, "IPC closed unexpectedly"
                            buffer.extend(data)
                        except socket.timeout:
                            assert process.poll() is None, "mock monitor exited"
                    raise AssertionError("IPC response deadline: " + str(latest))

                def command(op, **values):
                    nonlocal sequence
                    sequence += 1
                    send(dict(type="command", version=1, sdk_session=session, request=str(sequence), op=op, **values))
                    return str(sequence)

                def result(request, timeout=10):
                    value = receive_until(lambda v: v.get("type") == "result" and v.get("request") == request, timeout)
                    assert value["ok"], value
                    return value

                send(dict(type="hello", version=1, client="production-monitor-test"))
                receive_until(lambda v: v.get("state", {}).get("present") and v["state"]["connected"])
                assert "manual_control_v1" in latest["capabilities"]
                text = (out / "monitor.log").read_text()
                assert "MOCK_CMD TakeControl" not in text and "MOCK_CMD Move" not in text
                result(command("manual_take"))
                receive_until(lambda v: v.get("manual", {}).get("phase") == "manual")
                lease = latest["manual"]["lease"]
                assert latest["state"]["owned"] and latest["state"]["motion"] == 2
                result(command("posture", lease=lease, mode=3, action=3))
                receive_until(lambda v: v.get("state", {}).get("mode") == 3 and v["state"]["motion"] == 7)
                # Same connected SDK; selecting STAIR then Stand maps to one
                # SetMode(3), never an extra StandUp that races auto-standing.
                text = (out / "monitor.log").read_text()
                assert text.count("MOCK_CMD SetMode") == 1 and "MOCK_CMD StandUp" not in text
                result(command("speed", lease=lease, level=2))
                request = command("jog", lease=lease, generation="1", key="w", percent=30); result(request)
                for _ in range(9):
                    time.sleep(.08); result(command("heartbeat", lease=lease, generation="1", key="w", percent=30))
                result(command("release_jog", lease=lease, generation="1"))
                receive_until(lambda v: v.get("state", {}).get("actual_forward") == 0 and v.get("manual", {}).get("phase") == "manual")
                # Direction change is a new press; a queued old release cannot
                # erase it, and a stale heartbeat cannot recreate it later.
                result(command("jog", lease=lease, generation="2", key="s", percent=30))
                result(command("release_jog", lease=lease, generation="1"))
                receive_until(lambda v: v.get("state", {}).get("actual_forward", 0) < -.2)
                result(command("manual_release", lease=lease))
                receive_until(lambda v: v.get("manual", {}).get("phase") == "idle")
                result(command("manual_take")); receive_until(lambda v: v.get("manual", {}).get("phase") == "manual")
                next_lease = latest["manual"]["lease"]; assert next_lease != lease
                assert (out / "monitor.log").read_text().count("MOCK_CMD TakeControl") == 1
                result(command("posture", lease=next_lease, mode=3, action=4))
                result(command("posture", lease=next_lease, mode=3, action=3))
                # Stop while actually holding motion, not only while idle.
                # OFF must drain the pending zero without requiring another
                # manual takeover or restoring the previous held command.
                result(command("jog", lease=next_lease, generation="1", key="w", percent=30))
                receive_until(lambda v: v.get("state", {}).get("actual_forward", 0) > .2)
                result(command("emergency", on=True))
                receive_until(lambda v: v.get("state", {}).get("soft") == 2 and not v["state"]["estop_pending"]
                              and v.get("manual", {}).get("phase") == "idle")
                result(command("emergency", on=False))
                receive_until(lambda v: v.get("state", {}).get("soft") == 1 and not v["state"]["estop_pending"])
                def after_release_moves():
                    calls = [line.split() for line in (out / "monitor.log").read_text().splitlines() if line.startswith("MOCK_CMD ")]
                    released = max(float(c[5]) for c in calls if c[1] == "SoftEmergencyStop" and c[2] == "0.000000")
                    return [c for c in calls if c[1] == "Move" and float(c[5]) > released]
                receive_until(lambda _: bool(after_release_moves()), timeout=3)
                assert after_release_moves()[0][2:5] == ["0.000000"] * 3
                old = command("heartbeat", lease=next_lease, generation="1", key="w", percent=30)
                rejection = receive_until(lambda v: v.get("type") == "result" and v.get("request") == old)
                assert not rejection["ok"] and rejection["reason"] == "manual_lease_not_owned"
                time.sleep(.2)
                assert all(c[2:5] == ["0.000000"] * 3 for c in after_release_moves()), "OFF replayed manual motion"
                if args.post_estop_zero_failure:
                    failed = command("manual_take")
                    rejection = receive_until(lambda v: v.get("type") == "result" and v.get("request") == failed)
                    assert not rejection["ok"] and "结果未知" in rejection["reason"], rejection
                else:
                    result(command("manual_take")); receive_until(lambda v: v.get("manual", {}).get("phase") == "manual")
                    final_lease = latest["manual"]["lease"]
                    result(command("jog", lease=final_lease, generation="1", key="j", percent=20))
                    time.sleep(.1)
                # Closing the panel must zero only its own held manual jog.
                connection.close(); connection = None
                time.sleep(.5)
                process.send_signal(signal.SIGINT); process.wait(timeout=10)
                assert process.returncode == 0
                text = (out / "monitor.log").read_text(); calls = [line.split() for line in text.splitlines() if line.startswith("MOCK_CMD ")]
                assert sum(c[1] == "Connect" for c in calls) == 1
                first_release = next(float(c[5]) for c in calls if c[1] == "Move" and c[2:5] == ["0.000000"] * 3)
                forward = [float(c[5]) for c in calls if c[1] == "Move" and float(c[3]) > .2 and float(c[5]) < first_release]
                assert len(forward) >= 8, len(forward)
                intervals = [b-a for a,b in zip(forward,forward[1:])]
                assert max(intervals) < .16, intervals
                moves = [c for c in calls if c[1] == "Move"]
                assert moves[-1][2:5] == ["0.000000"] * 3, moves[-1]
                report = dict(status="PASS", scope="production Monitor/IPC/writer with SDK substitute, no physical robot",
                    sdk_connections=1, take_control_calls=1, manual_forward_calls=len(forward),
                    max_forward_interval_s=max(intervals), command_count=len(calls), callbacks="inline posture ACK and delayed Move write",
                    checks=["open_no_takeover", "lying_to_stair", "mode_single_dispatch", "speed_ACK_state", "20Hz_jog",
                            "generation_reversal", "zero_handback", "same_SDK_retake", "posture_feedback", "estop_held_ON_OFF",
                            "OFF_drains_zero_without_takeover", "old_manual_lease_not_revived",
                            "failed_zero_cannot_release_uncertainty" if args.post_estop_zero_failure else "disconnect_zero"])
                (out / "result.json").write_text(json.dumps(report, indent=2)); print(json.dumps(report))
            finally:
                if connection is not None: connection.close()
                if process and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired: process.kill(); process.wait()
                routing.send_signal(signal.SIGINT)
                try: routing.wait(timeout=5)
                except subprocess.TimeoutExpired: routing.kill(); routing.wait()


if __name__ == "__main__": main()
