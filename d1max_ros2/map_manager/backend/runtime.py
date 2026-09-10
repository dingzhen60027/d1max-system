from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


class RuntimeManager:
    """Own exactly one D1 Max SLAM process group and retain its recent logs."""

    COMMANDS = {
        "faster_lio": ["start_slam.sh", "faster_lio", "false"],
        "fastlio2": ["start_fastlio2.sh", "false"],
        "faster_lio_pgo": ["start_slam_pgo.sh", "false"],
    }

    def __init__(self, nav_root: Path, state_path: Path) -> None:
        self.nav_root = nav_root.resolve()
        self.state_path = state_path
        self.lock = threading.RLock()
        self.process: subprocess.Popen[str] | None = None
        self.reader: threading.Thread | None = None
        self.logs: deque[str] = deque(maxlen=300)
        self.state: dict[str, Any] = {
            "status": "idle",
            "algorithm": None,
            "pid": None,
            "started_at": None,
            "stopped_at": None,
            "exit_code": None,
            "error": None,
        }

    def _persist(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(self.state, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        os.replace(temporary, self.state_path)

    def recover_stale_state(self) -> None:
        if not self.state_path.is_file():
            return
        try:
            previous = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        pid = int(previous.get("pid") or 0)
        running = previous.get("status") in {"running", "stopping"}
        alive = pid > 1 and Path(f"/proc/{pid}").exists()
        with self.lock:
            self.state = {**self.state, **previous}
            if running and alive:
                # The Web process no longer owns this process group. Do not attach to
                # or kill it silently; the existing SLAM launch scripts clean stale
                # D1 Max mapping processes before a new start.
                self.state.update({
                    "status": "detached",
                    "error": "Web 重启后检测到旧流程，请确认后再启动新建图",
                })
            elif running:
                self.state.update({
                    "status": "idle",
                    "pid": None,
                    "stopped_at": now_iso(),
                    "error": None,
                })
            self._persist()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {**self.state, "logs": list(self.logs)}

    def start(self, algorithm: str) -> dict[str, Any]:
        arguments = self.COMMANDS.get(algorithm)
        if arguments is None:
            raise ValueError("不支持的建图算法")
        script = self.nav_root / arguments[0]
        if not script.is_file():
            raise FileNotFoundError(f"启动脚本不存在: {script}")
        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError("Web 已有建图流程运行中，请先停止")
            self.logs.clear()
            environment = {
                **os.environ,
                "RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
                "ROS_DOMAIN_ID": os.environ.get("D1MAX_ROS_DOMAIN_ID", "24"),
            }
            self.process = subprocess.Popen(
                ["/usr/bin/env", "bash", str(script), *arguments[1:]],
                cwd=self.nav_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=environment,
            )
            self.state = {
                "status": "running",
                "algorithm": algorithm,
                "pid": self.process.pid,
                "started_at": now_iso(),
                "stopped_at": None,
                "exit_code": None,
                "error": None,
            }
            self._persist()
            process = self.process
            self.reader = threading.Thread(target=self._watch, args=(process,), daemon=True)
            self.reader.start()
            return self.snapshot()

    def _watch(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        with process.stdout:
            for raw_line in process.stdout:
                line = raw_line.rstrip()
                if line:
                    stamp = datetime.now().strftime("%H:%M:%S")
                    with self.lock:
                        self.logs.append(f"[{stamp}] {line}")
        exit_code = process.wait()
        with self.lock:
            if self.process is not process:
                return
            was_stopping = self.state.get("status") == "stopping"
            self.state.update({
                "status": "idle" if was_stopping or exit_code == 0 else "failed",
                "pid": None,
                "stopped_at": now_iso(),
                "exit_code": exit_code,
                "error": None if was_stopping or exit_code == 0 else f"流程异常退出，退出码 {exit_code}",
            })
            self.process = None
            self._persist()

    def stop(self, timeout: float = 75.0) -> dict[str, Any]:
        with self.lock:
            process = self.process
            if not process or process.poll() is not None:
                self.process = None
                self.state.update({"status": "idle", "pid": None})
                self._persist()
                return self.snapshot()
            self.state["status"] = "stopping"
            self._persist()

        for stop_signal, wait_seconds in (
            (signal.SIGINT, timeout),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 2.0),
        ):
            try:
                os.killpg(process.pid, stop_signal)
                process.wait(timeout=wait_seconds)
                break
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
        with self.lock:
            if self.process is process:
                self.process = None
                self.state.update({
                    "status": "idle",
                    "pid": None,
                    "stopped_at": now_iso(),
                    "exit_code": process.poll(),
                    "error": None,
                })
                self._persist()
            return self.snapshot()

    def save(self, timeout: float = 90.0) -> dict[str, Any]:
        script = self.nav_root / "save_map.sh"
        if not script.is_file():
            raise FileNotFoundError(f"保存脚本不存在: {script}")
        result = subprocess.run(
            ["/usr/bin/env", "bash", str(script)],
            cwd=self.nav_root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            env={
                **os.environ,
                "RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
                "ROS_DOMAIN_ID": os.environ.get("D1MAX_ROS_DOMAIN_ID", "24"),
            },
        )
        output = result.stdout.strip()
        stamp = datetime.now().strftime("%H:%M:%S")
        with self.lock:
            for line in output.splitlines():
                self.logs.append(f"[{stamp}] {line}")
        if result.returncode != 0:
            raise RuntimeError(output or f"地图保存失败，退出码 {result.returncode}")
        return {"saved": True, "output": output}

    def close(self) -> None:
        self.stop(timeout=15.0)
