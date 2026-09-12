"""Authenticated loopback HTTP API. Fixed lifecycle actions, never arbitrary commands."""
from __future__ import annotations
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import signal
import threading

from .runtime import Manager, Conflict

ALLOWED_ORIGINS = {"null", "file://"}


def handler_for(manager, token, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass  # Never log Authorization or per-install capabilities.

        def _headers_ok(self):
            return (self.headers.get("Host") in {f"127.0.0.1:{port}", f"localhost:{port}"}
                    and self.headers.get("Origin") in ALLOWED_ORIGINS | {None})

        def _authorized(self):
            return self._headers_ok() and hmac.compare_digest(self.headers.get("Authorization", "").encode("utf-8"), ("Bearer " + token).encode("utf-8"))

        def respond(self, status, value):
            data = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            origin = self.headers.get("Origin")
            if origin in ALLOWED_ORIGINS:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_OPTIONS(self):
            if not self._headers_ok():
                return self.respond(403, {"error": "Origin or host rejected"})
            self.send_response(204)
            origin = self.headers.get("Origin")
            if origin in ALLOWED_ORIGINS:
                self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.send_header("Access-Control-Max-Age", "60")
            self.end_headers()

        def do_GET(self):
            if not self._authorized():
                return self.respond(403, {"error": "Unauthorized local client"})
            if self.path != "/v1/status":
                return self.respond(404, {"error": "Not found"})
            try:
                self.respond(200, manager.snapshot())
            except Exception:
                self.respond(503, {"error": "本机服务管理不可用，请检查 systemd 用户会话"})

        def do_POST(self):
            if not self._authorized():
                return self.respond(403, {"error": "Unauthorized local client"})
            match = re.fullmatch(r"/v1/(monitor|web)/(start|stop)", self.path)
            if not match:
                return self.respond(404, {"error": "Only fixed lifecycle actions are available"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 2048 or self.headers.get_content_type() != "application/json":
                    raise ValueError("Invalid JSON request")
                self.connection.settimeout(3)
                value = json.loads(self.rfile.read(length))
                if not isinstance(value, dict) or set(value) != {"request_id", "instance"}:
                    raise ValueError("Unexpected request fields")
                if not re.fullmatch(r"[a-zA-Z0-9_-]{16,80}", str(value["request_id"])):
                    raise ValueError("Invalid request identifier")
                manager.request(match[1], match[2], value["request_id"], value["instance"])
                self.respond(202, manager.snapshot())
            except Conflict as exc:
                self.respond(409, {"error": str(exc)})
            except (ValueError, TypeError, TimeoutError):
                self.respond(400, {"error": "Invalid request"})
            except Exception:
                self.respond(503, {"error": "服务操作未确认，请刷新状态核对；不会自动重试"})

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--credential", required=True)
    parser.add_argument("--runtime-dir", required=True)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if config["host"] != "127.0.0.1":
        raise ValueError("Manager must remain loopback-only")
    for name in ("monitor", "web"):
        if config[name + "_unit"] != f"d1max-{name}-managed.service":
            raise ValueError("Only dedicated D1 Max units are allowed")
    credential = Path(args.credential)
    if credential.stat().st_mode & 0o077:
        raise ValueError("Local credential must have mode 0600")
    token = json.loads(credential.read_text())["token"]
    if not isinstance(token, str) or len(token) < 40:
        raise ValueError("Missing installation capability")
    manager = Manager(config, args.runtime_dir)
    server = ThreadingHTTPServer((config["host"], config["port"]), handler_for(manager, token, config["port"]))
    server.daemon_threads = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
