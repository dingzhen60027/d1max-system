"""Launch the independent desktop remote; never create an SDK session here."""
from __future__ import annotations

import logging
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlsplit
import uuid

from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel


logger = logging.getLogger(__name__)
DESKTOP_ENV = ("DISPLAY", "WAYLAND_DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR",
               "DBUS_SESSION_BUS_ADDRESS", "D1MAX_SDK_ROOT", "LANG", "LC_ALL")


class OpenRemoteRequest(BaseModel):
    class Config:
        extra = "forbid"


class RemoteApplication:
    def __init__(self, root: Path | None = None, *, runner=None, timeout=30.0):
        self.root = (root or Path(os.environ.get(
            "D1MAX_REMOTE_ROOT", str(Path.home() / "d1max_sdk_remote")))).resolve()
        self.runner = runner or subprocess.run
        self.timeout = timeout
        self.lock = threading.Lock()

    def open(self):
        # Launches are independent of all navigation locks and SDK ownership.
        if not self.lock.acquire(blocking=False):
            return {"status": "opening"}
        unit = None
        try:
            entry = self.root / "gui.sh"
            if not entry.is_file() or not os.access(entry, os.X_OK):
                raise HTTPException(404, "未安装遥控器")
            environment = {key: os.environ[key] for key in DESKTOP_ENV if os.environ.get(key)}
            if not (environment.get("DISPLAY") or environment.get("WAYLAND_DISPLAY")):
                raise HTTPException(503, "请在桌面登录后打开")
            unit = "d1max-remote-" + uuid.uuid4().hex + ".service"
            # A separate user service keeps the manual remote independent when
            # the Web/navigation service is stopped. Qt owns SDK cleanup on exit.
            with tempfile.TemporaryDirectory(prefix="d1max-remote-open-") as directory:
                ready = Path(directory) / "ready"
                command = ["systemd-run", "--user", "--quiet", "--collect",
                           "--unit=" + unit, "--service-type=exec",
                           "--property=Description=D1 Max Remote",
                           "--property=TimeoutStopSec=12", "--property=KillSignal=SIGTERM",
                           "--property=Restart=no", "--working-directory=" + str(self.root)]
                command += ["--setenv=" + key + "=" + value for key, value in environment.items()]
                command += [str(entry), "--ready-file", str(ready)]
                self.runner(command, check=True, capture_output=True, text=True, timeout=5)
                deadline = time.monotonic() + self.timeout
                next_check = 0.0
                while True:
                    if ready.is_file() and ready.read_text(encoding="utf-8").strip() == "ready":
                        return {"status": "opened"}
                    now = time.monotonic()
                    if now >= deadline:
                        raise TimeoutError("Remote did not acknowledge a window")
                    if now >= next_check:
                        result = self.runner(
                            ["systemctl", "--user", "show", unit, "--property=ActiveState", "--value"],
                            check=False, capture_output=True, text=True, timeout=2)
                        if result.stdout.strip() in {"failed", "inactive"}:
                            # A short-lived activator may have acknowledged and
                            # exited during the systemd query.
                            if ready.is_file() and ready.read_text(encoding="utf-8").strip() == "ready":
                                return {"status": "opened"}
                            raise RuntimeError("Remote exited before showing a window")
                        next_check = now + 1.0
                    time.sleep(0.05)
        except HTTPException:
            raise
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            logger.warning("Remote launch failed (%s): %s", unit, exc)
            if unit:
                try:
                    # Only the launcher instance allocated above, never an
                    # existing remote, navigation unit, or SDK background service.
                    self.runner(["systemctl", "--user", "stop", "--no-block", unit],
                                check=False, capture_output=True, text=True, timeout=2)
                except (OSError, subprocess.SubprocessError):
                    logger.exception("Could not retire remote launcher %s", unit)
            raise HTTPException(503, "遥控器打开失败，请重试") from exc
        finally:
            self.lock.release()


def create_remote_router(remote: RemoteApplication):
    def local_request(request: Request):
        # JSON + local Host/Origin prevents a foreign page from launching apps.
        loopback = {"localhost", "127.0.0.1", "::1"}
        if request.url.hostname not in loopback:
            raise HTTPException(403, "仅支持本机打开")
        if request.headers.get("content-type", "").split(";")[0] != "application/json":
            raise HTTPException(415, "需要 JSON 请求")
        origin = request.headers.get("origin")
        if origin:
            try:
                parsed = urlsplit(origin)
                allowed = (parsed.scheme in {"http", "https"} and parsed.hostname in loopback
                           and parsed.port in {request.url.port, 5173})
            except ValueError:
                allowed = False
            if not allowed:
                raise HTTPException(403, "请求来源不允许")

    router = APIRouter(prefix="/api/remote", dependencies=[Depends(local_request)])

    @router.post("/open")
    def open_remote(_request: OpenRemoteRequest):
        return remote.open()

    return router
