#!/usr/bin/env python3
"""Launch the existing navigation session against the isolated Isaac plant."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]


def candidate_environment(candidate):
    env = dict(os.environ, D1MAX_RELEASE=str(candidate))
    setup = candidate / "application/install/local_setup.bash"
    if not setup.is_file():
        raise FileNotFoundError(setup)
    result = subprocess.run(
        ["bash", "-c", 'set -e; source "$1"; /usr/bin/python3 -c '
         "'import json,os; print(json.dumps(dict(os.environ)))'", "isaac-env", str(setup)],
        env=env, text=True, capture_output=True, check=True,
    )
    return json.loads(result.stdout.splitlines()[-1])


def selected_candidate(build_root):
    explicit = os.environ.get("D1MAX_SIM_CANDIDATE")
    if explicit:
        return Path(explicit)
    selector = build_root / "isaac_fixture.json"
    if not selector.exists():
        return build_root / "isaac-candidate"
    value = json.loads(selector.read_text())
    if value.get("kind") != "isolated_isaac_fixture_selection":
        raise ValueError("Invalid local Isaac fixture selection")
    candidate = Path(value["candidate"]).resolve(strict=True)
    manifest = candidate / "isaac_candidate_integrity.json"
    if hashlib.sha256(manifest.read_bytes()).hexdigest() != value["integrity_sha256"]:
        raise ValueError("Selected Isaac fixture integrity record changed")
    return candidate


def require_retired_shutdown(session, session_id):
    record = json.loads((session / "shutdown.json").read_text())
    if (record.get("session_id") != session_id or record.get("request_accepted") is not True
            or record.get("software_retired") is not True):
        raise RuntimeError("Navigation owner retirement unconfirmed; inspect shutdown.json")
    return record


def main():
    build_root = Path(os.environ.get("D1MAX_SIM_BUILD", REPO.parent / "d1max-build-isaac")).resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--session", type=Path, help="New run directory; never overwrite a run")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-fps", type=float, default=0.,
                        help="GUI recording: render-only cadence (0 keeps original behavior; max 60)")
    parser.add_argument("--capture-scene", action="store_true", help="Capture a rendered overview during a headless run")
    parser.add_argument("--rviz", action="store_true", help="Open the original single navigation RViz")
    parser.add_argument("--camera", choices=("overview", "follow"), default="follow",
                        help="Isaac viewport camera; large scenes default to following the physical robot")
    parser.add_argument("--duration", type=float, default=0., help="Wall seconds; 0 waits for Ctrl+C")
    parser.add_argument("--smoke", action="store_true", help="Send a goal through the original BT command interface")
    parser.add_argument("--scenario-case", help="Run a sealed system-test case in this same scene and navigation session")
    parser.add_argument("--smoke-case", choices=("goal", "cancel", "preview_cancel"), default="goal")
    parser.add_argument("--smoke-duration", type=float, default=90., help="Navigation test timeout in wall seconds")
    parser.add_argument("--goal", nargs=3, type=float, help="Ground XYZ goal in the original map frame")
    args = parser.parse_args()
    if not math.isfinite(args.render_fps) or not 0 <= args.render_fps <= 60:
        parser.error("render-fps must be finite and between 0 and 60")
    if args.headless and args.render_fps:
        parser.error("render-fps requires a visible Isaac GUI; omit --headless")
    if args.scenario_case and (args.smoke or args.goal):
        parser.error('scenario-case selects its sealed goals; omit smoke/goal')
    candidate = (args.candidate or selected_candidate(build_root)).resolve()
    fixture = candidate / "simulation"
    if not candidate.is_dir():
        raise SystemExit("Candidate missing. Run build_local.sh first: " + str(candidate))
    # Fresh interpreter guarantees all task/control imports come from the copied
    # candidate; changing PYTHONPATH after import would keep stale modules alive.
    if os.environ.get("D1MAX_ISAAC_REEXEC") != str(candidate):
        env = candidate_environment(candidate)
        env["D1MAX_ISAAC_REEXEC"] = str(candidate)
        env["D1MAX_SIM_BUILD"] = str(build_root)
        # Pin the selected candidate across re-exec, and run its frozen harness
        # as well as its frozen sensor/observer scripts.
        argv = [*sys.argv[1:]]
        if args.candidate is None:
            argv.extend(["--candidate", str(candidate)])
        os.execve(sys.executable, [sys.executable, str(fixture / "run.py"), *argv], env)

    from d1max_pct_scan.isolated_zenoh import private_router, stop_owned
    session = (args.session or build_root / "runs" /
               time.strftime("isaac_%Y%m%d_%H%M%S")).resolve()
    if session.exists():
        raise SystemExit("Run directory already exists: " + str(session))
    session.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(fixture / "nav_prepare.py"),
                    "--release", str(candidate), "--output", str(session)], check=True)
    session_spec = json.loads((session / "session.json").read_text())
    plant_contract = session_spec["isaac_bridge_contract"]
    scene_file = Path(plant_contract["scene_config"]).resolve(strict=True)
    if hashlib.sha256(scene_file.read_bytes()).hexdigest() != plant_contract["scene_sha256"]:
        raise RuntimeError("Sealed scene geometry changed before simulator startup")

    def bridge_status():
        try:
            return json.loads((session / "isaac_bridge_status.json").read_text())
        except (OSError, ValueError):
            return {}

    stopping = False
    def stop(_sig, _frame):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    children, logs = [], []
    def start(name, command, env):
        log = (session / (name + ".log")).open("w")
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
        return process

    @contextmanager
    def owned_graph():
        with private_router(session / "transport") as env:
            try:
                yield env
            finally:
                # The task owner retires while measured sensors, physical plant
                # and its router remain alive, including on faults/exceptions.
                for name, process in children:
                    if name == "navigation" and process.poll() is None:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=12.)
                        except subprocess.TimeoutExpired:
                            stop_owned(process)
                # Once the owner has retired, stop sensor callbacks and send
                # the bridge's bounded zero while the physical plant is alive.
                # Keep its ROS context/router alive until bridge teardown ends.
                for name, process in children:
                    if name == "isaac_bridge":
                        stop_owned(process)
                for name, process in reversed(children):
                    stop_owned(process)

    started = time.monotonic()
    result = dict(session=str(session), candidate=str(candidate),
        transport="rmw_zenoh_cpp", domain=219, fixture_only=True,
        localization="Isaac PhysX ground truth", physical_robot_acceptance=False)
    try:
        with owned_graph() as env:
            # Keep every sensor/control process on one owned loopback graph.
            (session / "ros_environment.json").write_text(json.dumps({
                key: env[key] for key in ("RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
                "ZENOH_SESSION_CONFIG_URI", "ZENOH_ROUTER_CONFIG_URI",
                "D1MAX_NAV_ISOLATION_TOKEN", "D1MAX_NAV_ISOLATED",
                "D1MAX_NAV_TRANSPORT", "D1MAX_OFFLINE_ZENOH_TEST")}, indent=2) + "\n")
            start("isaac_bridge", [sys.executable, str(fixture / "bridge.py"), "--session", str(session)], env)
            isaac_env = dict(env)
            # Isaac's Python 3.12 cannot import Humble's Python 3.10 modules.
            # Only the bounded loopback plant protocol crosses this boundary.
            for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH",
                        "CMAKE_PREFIX_PATH", "PYTHONHOME", "CONDA_PREFIX", "VIRTUAL_ENV",
                        "PYTHONEXE", "LD_PRELOAD"):
                isaac_env.pop(key, None)
            isaac = Path(os.environ.get("ISAAC_SIM_ROOT", "/home/eric/isaacsim"))
            scene_command = [str(isaac / "python.sh"), str(fixture / "scene.py"),
                "--scene-file", str(scene_file), "--scene-sha256", plant_contract["scene_sha256"],
                "--result-dir", str(session / "physics"),
                "--export-scene", str(session / "physics/indoor_scene.usda"),
                "--control-file", str(session / "physics_control.json")]
            scene_command.extend(["--camera", args.camera])
            scene_command.extend(['--session-id', session_spec['id'], '--clock-anchor-ns',
                str(plant_contract['clock_anchor_ns'])])
            if session_spec.get('static_collision_prior_contract'):
                scene_command.extend(['--static-prior-geometry-sha256',
                    session_spec['static_collision_prior_contract']['static_prior_geometry_sha256']])
                if session_spec['static_collision_prior_contract'].get('body_envelope_attestation_required'):
                    scene_command.extend(['--body-envelope-json', json.dumps(
                        session_spec['static_collision_prior_contract']['body_envelope'], separators=(',', ':'))])
            if args.headless:
                scene_command.append("--headless")
            if args.render_fps:
                scene_command.extend(["--render-fps", str(args.render_fps)])
            if not args.headless or args.capture_scene:
                scene_command.extend(["--screenshot-path", str(session / "physics/overview.png")])
            start("isaac_sim", scene_command, isaac_env)
            deadline = time.monotonic() + 180.
            while True:
                if stopping:
                    raise RuntimeError("Stopped during simulator startup")
                exited = [(name, p.returncode) for name, p in children if p.poll() is not None]
                if exited:
                    raise RuntimeError("Plant startup failed: " + repr(exited))
                status = bridge_status()
                if status.get("fault"):
                    raise RuntimeError("Plant bridge fault: " + str(status["fault"]))
                if (status.get("measured_state_samples", 0) >= 5
                        and status.get("native_imu_samples", 0) > 0
                        and all(x > 0 for x in status.get("ray_scans", [0, 0]))):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Simulator did not produce measured state and both LiDAR scans")
                time.sleep(.2)
            start("navigation", [sys.executable, "-m", "d1max_pct_scan.navigation_session",
                "run", "--session", str(session)], env)
            if args.rviz:
                start("rviz", [sys.executable, "-m", "d1max_pct_scan.navigation_session",
                    "view", "--session", str(session)], env)
            smoke = None
            if args.smoke:
                smoke_command = [sys.executable, str(fixture / "smoke.py"),
                    "--session", str(session), "--case", args.smoke_case,
                    "--duration", str(args.smoke_duration)]
                if args.goal:
                    smoke_command.extend(["--goal", *map(str, args.goal)])
                smoke = start("smoke", smoke_command, env)
            elif args.scenario_case:
                smoke = start('smoke', [sys.executable, str(fixture/'scenario_suite.py'),
                    'execute', '--session', str(session), '--case', args.scenario_case], env)
            print("Isaac navigation running. Logs: " + str(session), flush=True)
            while not stopping:
                status = bridge_status()
                if status.get("fault"):
                    raise RuntimeError("Plant bridge fault: " + str(status["fault"]) +
                                       "; start a new session before continuing")
                failures = [(name, p.returncode) for name, p in children
                    if p.poll() is not None and name not in ("rviz", "smoke")]
                if failures:
                    raise RuntimeError("Required process exited: " + repr(failures))
                if smoke is not None and smoke.poll() is not None:
                    if smoke.returncode:
                        raise RuntimeError("Navigation test failed; inspect smoke.log and its original reports")
                    result["smoke_completed"] = True
                    result['scenario_case'] = args.scenario_case
                    break
                if args.duration and time.monotonic() - started >= args.duration:
                    if smoke is not None:
                        raise RuntimeError("Run duration ended before the navigation test completed")
                    break
                time.sleep(.2)
        # The supervisor's request ACK and a successful task result do not
        # prove that the owner drained. Check its recorded retirement evidence
        # after the owned graph has finished its ordered teardown.
        result["navigation_shutdown"] = require_retired_shutdown(session, session_spec["id"])
        result["clean_shutdown"] = True
    except Exception as exc:
        result.update(error=str(exc), clean_shutdown=False)
        raise
    finally:
        for name, process in reversed(children):
            stop_owned(process)
        for stream in logs:
            stream.close()
        if (session / "shutdown.json").exists():
            result["navigation_shutdown"] = json.loads((session / "shutdown.json").read_text())
        result["child_exit_codes"] = {name: process.returncode for name, process in children}
        result["wall_seconds"] = time.monotonic() - started
        (session / "run_summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print("Run summary: " + str(session / "run_summary.json"), flush=True)


if __name__ == "__main__":
    main()
