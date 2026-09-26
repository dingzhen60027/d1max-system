#!/usr/bin/env python3
"""Explicit motion-capable, startup-disarmed monitor lifecycle handoff.

Public actions use only the existing authenticated loopback manager API. No ROS
or robot connection is made by check/prepare/cancel. The private _launch action
is called by start_sdk_monitor.sh after the unchanged Zenoh environment setup.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import socket
import stat
import subprocess
import sys
import time
from urllib.request import Request, ProxyHandler, build_opener


BASE = Path(__file__).resolve().parents[1]
UNIT = "d1max-monitor-managed.service"
TTL_SECONDS = 60
TICKET_NAME = "motion-next-start.json"
RECEIPT_NAME = "motion-start-receipt.json"
SDK_EXECUTABLES = {"sdk_monitor_bridge", "sdk_state_bridge", "sdk_telemetry_bridge", "sdk_console_bridge"}
LIMITS = {"navigation_max_forward_mps": 0.30, "navigation_max_lateral_mps": 0.0,
          "navigation_max_yaw_radps": 0.50}
STOP_FIRST = "已有 monitor 或启动/停止尚未完成；请通过现有 manager 停止并确认 stopped 后再执行。不会旁起 SDK 或自动重启。"


def private_directory(path):
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise RuntimeError(f"需要当前用户独占的目录：{path}")
    return path


def runtime_directory():
    root = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    if not root.is_absolute():
        raise RuntimeError("XDG_RUNTIME_DIR 必须是绝对路径")
    return private_directory(root)


def session_directory():
    # Created by the existing manager's RuntimeDirectory, never by this CLI.
    return private_directory(runtime_directory() / "d1max-session")


def read_private_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd) as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077 or info.st_size > 8192:
            raise RuntimeError(f"拒绝非私有或无效配置：{path.name}")
        value = json.load(stream)
        if not isinstance(value, dict):
            raise RuntimeError(f"配置必须是JSON对象：{path.name}")
        return value


@contextmanager
def startup_lock():
    # Do not use the SDK core's SessionLease path here: flock on an independently
    # reopened descriptor would reject the lock inherited across exec by SDK main.
    path = runtime_directory() / "d1max-sdk-monitor-startup.lock"
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError("SDK monitor 启动锁权限异常")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(STOP_FIRST) from exc
        # Retain startup serialization across exec. The SDK core independently
        # owns d1max-sdk-monitor.lock as the final atomic SDK-session lease.
        os.set_inheritable(fd, True)
        yield fd
    finally:
        os.close(fd)


def core_lease_path():
    # Match SessionLease::default_path(), which intentionally ignores XDG overrides.
    return Path(f"/run/user/{os.getuid()}/d1max-sdk-monitor.lock")


def require_core_lease_available():
    path = core_lease_path()
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    except FileNotFoundError:
        return
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise RuntimeError("SDK core 会话锁权限异常")
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(STOP_FIRST) from exc
        # Read-only availability probe; core acquires its own lifetime lease.
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


class ManagerClient:
    def __init__(self):
        self.config = json.loads((BASE / "config/manager.json").read_text())
        if self.config.get("host") != "127.0.0.1" or self.config.get("monitor_unit") != UNIT:
            raise RuntimeError("仅支持现有本机 monitor manager")
        self.token = read_private_json(BASE / "config/manager.local.json")["token"]
        if not isinstance(self.token, str) or len(self.token) < 40:
            raise RuntimeError("现有 manager 凭据无效；未修改任何凭据")
        self.opener = build_opener(ProxyHandler({}))

    def request(self, path, body=None):
        headers = {"Authorization": "Bearer " + self.token}
        data = None
        if body is not None:
            headers["Content-Type"] = "application/json"
            data = json.dumps(body).encode()
        request = Request(f"http://127.0.0.1:{int(self.config['port'])}" + path,
                          headers=headers, data=data, method="POST" if body is not None else "GET")
        try:
            with self.opener.open(request, timeout=4) as response:
                value = json.load(response)
        except (OSError, ValueError) as exc:
            raise RuntimeError("现有 manager 请求未确认；请在管理界面核对状态。未自动重试或重启。") from exc
        if not isinstance(value, dict):
            raise RuntimeError("manager 返回了无效状态")
        return value

    def status(self):
        return self.request("/v1/status")

    def start(self, instance, request_id):
        return self.request("/v1/monitor/start", {"instance": instance, "request_id": request_id})


def local_sdk_processes():
    found = []
    for path in Path("/proc").iterdir():
        if not path.name.isdecimal():
            continue
        try:
            executable = (path / "exe").readlink().name.removesuffix(" (deleted)")
            if executable in SDK_EXECUTABLES:
                found.append(int(path.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # ROS publisher discovery remains a second check for other users
            # or hosts; never treat /proc scanning as a network ownership proof.
            continue
    return found


def port_in_use(port):
    with socket.socket() as sock:
        sock.settimeout(.25)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def require_stopped(snapshot, config):
    monitor = snapshot.get("monitor", {})
    if (monitor.get("phase") != "stopped" or monitor.get("active") is not False or
            monitor.get("owned") is not False or monitor.get("busy") is not False):
        raise RuntimeError(STOP_FIRST)
    instance = snapshot.get("instance")
    if not isinstance(instance, str) or not re.fullmatch(r"[0-9a-f]{32}", instance):
        raise RuntimeError("manager instance 无效")
    require_core_lease_available()
    if local_sdk_processes() or any(port_in_use(port) for port in (int(config["monitor_port"]), 7448)):
        raise RuntimeError("发现本机 SDK 进程或监控端口占用；拒绝重复连接。请核对现有 manager/桥接进程，不会自动清理。")


def validate_ticket(ticket, instance):
    age = time.monotonic() - float(ticket["created_monotonic"])
    if (ticket.get("version") != 1 or ticket.get("package") != str(BASE) or ticket.get("limits") != LIMITS or
            not re.fullmatch(r"[0-9a-f]{32}", str(ticket.get("nonce", ""))) or not 0 <= age <= TTL_SECONDS):
        raise RuntimeError("运动启动票据无效或超过60秒；请重新显式 start")
    if instance != ticket["manager_instance"]:
        raise RuntimeError("manager 已重启；旧运动启动票据失效")


def create_ticket(client, *, reuse=False):
    with startup_lock():
        snapshot = client.status()
        require_stopped(snapshot, client.config)
        if snapshot.get("motion_start_binding") != 1:
            raise RuntimeError("运行中的 manager 不支持持久启动关联；请在 monitor 确认为 stopped 后，手动重启 manager 加载新版本。未准备票据或启动 monitor。")
        path = session_directory() / TICKET_NAME
        ticket = {"version": 1, "nonce": secrets.token_hex(16), "manager_instance": snapshot["instance"],
                  "created_monotonic": time.monotonic(), "package": str(BASE), "limits": LIMITS}
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        except FileExistsError as exc:
            if reuse:
                existing = read_private_json(path)
                try:
                    validate_ticket(existing, snapshot["instance"])
                except (ValueError, TypeError, KeyError, RuntimeError):
                    path.unlink()
                    raise
                return existing
            raise RuntimeError("已有待消费启动票据；请先执行 cancel，再重新 prepare/start。") from exc
        with os.fdopen(fd, "w") as stream:
            json.dump(ticket, stream)
            stream.flush()
            os.fsync(stream.fileno())
        return ticket


def cancel_ticket(nonce=None):
    path = session_directory() / TICKET_NAME
    try:
        current = read_private_json(path)
    except FileNotFoundError:
        return False
    if nonce is not None and current.get("nonce") != nonce:
        return False
    # Only this private, exact ticket is removed. No process or credential is touched.
    path.unlink()
    return True


def managed_invocation():
    invocation = os.environ.get("INVOCATION_ID", "")
    if os.environ.get("D1MAX_MANAGED_MONITOR") != "1" or not re.fullmatch(r"[0-9a-f]{32}", invocation):
        raise RuntimeError("运动启动票据仅能由现有 manager 的受管 monitor 消费")
    groups = Path("/proc/self/cgroup").read_text().splitlines()
    if not any(line.split(":", 2)[-1].endswith("/" + UNIT) for line in groups):
        raise RuntimeError("当前进程不在受管 monitor cgroup 中")
    result = subprocess.run(["systemctl", "--user", "show", UNIT, "--property=InvocationID", "--value"],
                            capture_output=True, text=True, timeout=4, check=True)
    if result.stdout.strip() != invocation:
        raise RuntimeError("monitor invocation 已改变；拒绝消费旧启动票据")
    return invocation


def consume_ticket():
    path = session_directory() / TICKET_NAME
    try:
        ticket = read_private_json(path)
    except FileNotFoundError:
        return None
    try:
        invocation = managed_invocation()
        snapshot = ManagerClient().status()
        validate_ticket(ticket, snapshot.get("instance"))
        binding = snapshot.get("monitor", {}).get("last_start") or {}
        if (binding.get("request_id") != "motion_" + ticket["nonce"] or
                binding.get("invocation") != invocation):
            raise RuntimeError("不是本票据绑定的显式启动请求；请使用 sdk_motion_startup.py start")
        return {**ticket, "invocation": invocation}
    finally:
        # A failed or successful attempt is one-shot. Invalid managed context
        # also cannot leave a latent enable request for a later invocation.
        path.unlink()


def ros_duplicate_preflight():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("d1max_monitor_preflight", enable_rosout=False, start_parameter_services=False)
    try:
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.get_publishers_info_by_topic("/d1max_sdk_bridge/robot_state"):
            raise RuntimeError("SDK 已有状态发布者，拒绝重复连接。请先核对现有桥。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def sdk_arguments(enabled):
    audit = os.environ.get('D1MAX_JOINT_TELEMETRY_AUDIT', 'false')
    if audit not in ('true', 'false'):
        raise RuntimeError('Invalid joint telemetry audit option')
    if audit == 'true' and enabled:
        raise RuntimeError('Joint telemetry audit cannot consume a motion startup request')
    workspace = BASE.parent / "sdk_bridge_ws"
    args = [str(workspace / "install/d1max_sdk_bridge/lib/d1max_sdk_bridge/sdk_monitor_bridge"),
            "--ros-args", "--params-file", str(workspace / "src/d1max_sdk_bridge/config/monitor.yaml"),
            "-p", "navigation_control_enabled:=" + ("true" if enabled else "false")]
    for name, value in LIMITS.items():
        args.extend(["-p", f"{name}:={value}"])
    if audit == 'true':
        # A scoped read-only audit overrides, but never rewrites, the operator's
        # normal APP handoff preference. No ownership acquisition is authorized.
        args.extend(['-p', 'joint_state_enabled:=true',
                     '-p', 'auto_take_control_on_available:=false'])
    return args


def launch_sdk():
    try:
        if os.environ.get("D1MAX_NAVIGATION_CONTROL_ENABLED", "false") != "false":
            raise RuntimeError("请使用 sdk_motion_startup.py start 显式启动；不再通过环境变量启用运动能力")
        with startup_lock():
            require_core_lease_available()
            if local_sdk_processes():
                raise RuntimeError(STOP_FIRST)
            ros_duplicate_preflight()
            # Standalone telemetry monitoring still works without a manager session;
            # motion capability always requires a ticket plus a verified managed unit.
            try:
                session_directory()
            except FileNotFoundError:
                ticket = None
            else:
                ticket = consume_ticket()
            args = sdk_arguments(ticket is not None)
            if ticket is not None:
                receipt = session_directory() / RECEIPT_NAME
                fd = os.open(receipt, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
                with os.fdopen(fd, "w") as stream:
                    json.dump({"nonce": ticket["nonce"], "invocation": ticket["invocation"],
                               "limits": LIMITS, "startup_disarmed": True}, stream)
            print("SDK monitor: motion-capable, startup DISARMED" if ticket else "SDK monitor: motion DISABLED", flush=True)
            # This is the only process launch: the current existing monitor child is
            # replaced, never a second SDK client beside it. No SDK command is sent here.
            os.execv(args[0], args)
    except BaseException:
        # Covers preflight or exec failure, including an ordinary manager Connect
        # request rejected for not matching a prepared ticket. No latent opt-in.
        try:
            cancel_ticket()
        except (OSError, ValueError, TypeError, RuntimeError):
            pass
        raise


def start_managed(client):
    try:
        ticket = create_ticket(client, reuse=True)
    except BaseException:
        # A previously prepared opt-in must not survive a rejected explicit
        # start (for example, the UI won the race and now owns the monitor).
        try:
            cancel_ticket()
        except (OSError, ValueError, TypeError, RuntimeError):
            pass
        raise
    try:
        client.start(ticket["manager_instance"], "motion_" + ticket["nonce"])
        deadline = time.monotonic() + float(client.config["start_timeout_seconds"]) + 5
        while time.monotonic() < deadline:
            snapshot = client.status()
            if snapshot.get("instance") != ticket["manager_instance"]:
                raise RuntimeError("manager 已改变；启动未确认，请核对现有管理器状态")
            monitor = snapshot.get("monitor", {})
            if monitor.get("phase") in {"failed", "conflict", "stopped", "stopping"}:
                raise RuntimeError("monitor 启动失败或取消：" + str(monitor.get("error", monitor.get("phase"))))
            if monitor.get("active") is True and monitor.get("busy") is False:
                binding = monitor.get("last_start") or {}
                if binding.get("request_id") != "motion_" + ticket["nonce"]:
                    raise RuntimeError("monitor 并非本次显式请求启动；拒绝启用或接管。请通过manager停止后重新start")
                try:
                    receipt = read_private_json(session_directory() / RECEIPT_NAME)
                except FileNotFoundError:
                    receipt = {}
                if (receipt.get("nonce") == ticket["nonce"] and receipt.get("startup_disarmed") is True and
                        receipt.get("invocation") == binding.get("invocation")):
                    return {"motion_capable": True, "startup_disarmed": True,
                            "monitor_phase": monitor.get("phase"), "limits": LIMITS}
                # Generic health can precede the SDK's 3-second preflight. The
                # persistent association remains valid after jobs is cleared;
                # wait for our one-shot receipt instead of deleting its ticket.
            time.sleep(.15)
        raise RuntimeError("monitor 启动超时未确认；请核对manager状态。未自动重试/重启。")
    finally:
        cancel_ticket(ticket["nonce"])


def main():
    parser = argparse.ArgumentParser(description="显式启动同一个受管SDK monitor的运动能力；启动仍DISARMED")
    parser.add_argument("action", choices=("check", "status", "prepare", "cancel", "start", "_launch"))
    args = parser.parse_args()
    try:
        if args.action == "_launch":
            launch_sdk()
            return
        if args.action == "cancel":
            print("已取消待消费票据；运行中的monitor不变" if cancel_ticket() else "没有待消费票据")
            return
        client = ManagerClient()
        if args.action in {"check", "status"}:
            snapshot = client.status()
            if args.action == "check":
                require_stopped(snapshot, client.config)
            print(json.dumps({"monitor": snapshot.get("monitor"),
                              "motion_start_binding": snapshot.get("motion_start_binding"), "motion_start_limits": LIMITS,
                              "pending_ticket": (session_directory() / TICKET_NAME).exists()}, ensure_ascii=False))
        elif args.action == "prepare":
            create_ticket(client)
            print("已准备60秒内一次性运动能力启动票据；请执行本CLI的start提交绑定请求。启动仍DISARMED；取消请执行cancel。")
        else:
            print(json.dumps(start_managed(client), ensure_ascii=False))
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(2) from None


if __name__ == "__main__":
    main()
