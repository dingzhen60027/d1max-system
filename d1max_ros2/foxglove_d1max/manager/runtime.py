"""Own only two named systemd user units; never find-and-kill arbitrary processes."""
from __future__ import annotations

from collections import OrderedDict
import json
from pathlib import Path
import secrets
import socket
import subprocess
import threading
import time
from urllib.request import urlopen, ProxyHandler, build_opener


class Conflict(RuntimeError):
    pass


class Systemd:
    def show(self, unit):
        result = subprocess.run(
            ["systemctl", "--user", "show", unit, "--property=ActiveState,SubState,MainPID,InvocationID,ControlGroup,Result"],
            capture_output=True, text=True, timeout=4, check=True,
        )
        return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)

    def action(self, unit, verb):
        if verb not in {"start", "stop"}:
            raise ValueError("Unsupported lifecycle action")
        subprocess.run(["systemctl", "--user", verb, "--no-block", unit],
                       capture_output=True, text=True, timeout=5, check=True)

    def populated(self, state):
        group = state.get("ControlGroup", "")
        if not group:
            return False
        # Trust only kernel cgroup paths, never user-supplied process identifiers.
        path = (Path("/sys/fs/cgroup") / group.lstrip("/")).resolve()
        if not path.is_relative_to(Path("/sys/fs/cgroup")):
            raise RuntimeError("Invalid unit cgroup")
        try:
            return "populated 1" in (path / "cgroup.events").read_text()
        except FileNotFoundError:
            return False


def port_open(host, port, timeout=.35):
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def web_ready(port):
    try:
        with build_opener(ProxyHandler({})).open(f"http://127.0.0.1:{port}/api/health", timeout=1) as response:
            data = json.load(response)
        return data.get("ok") is True and data.get("rmw") == "rmw_zenoh_cpp"
    except (OSError, ValueError):
        return False


def robot_network_error(config):
    """A proxy TUN accepting TCP is not evidence of a physical robot link."""
    interface = config.get("robot_interface", "")
    if not interface or "/" in interface:
        return "机器人网卡尚未配置"
    try:
        if (Path("/sys/class/net") / interface / "carrier").read_text().strip() != "1":
            return f"机器人网络不可达：{interface} 未连接网线"
        for host, _port in config["robot_endpoints"]:
            result = subprocess.run(["ip", "-j", "route", "get", host], capture_output=True,
                                    text=True, timeout=2, check=True)
            routes = json.loads(result.stdout)
            if not routes or routes[0].get("dev") != interface:
                return f"机器人网络不可达：{host} 未经过 {interface}；请核对网卡配置和代理绕行"
    except (OSError, ValueError, subprocess.SubprocessError):
        return f"机器人网络不可达：无法确认 {interface} 的有线链路和路由"
    return ""


class Manager:
    def __init__(self, config, runtime_dir, *, system=None, probe=port_open, web_probe=web_ready, network_check=robot_network_error):
        self.config, self.runtime_dir = config, Path(runtime_dir)
        self.system, self.probe, self.web_probe = system or Systemd(), probe, web_probe
        self.network_check = network_check
        self.instance = secrets.token_hex(16)
        self.lock = threading.RLock()
        self.jobs = {key: None for key in ("monitor", "web")}
        self.errors = {key: "" for key in self.jobs}
        self.requests = OrderedDict()
        self.threads = []

    def _unit(self, name):
        if name not in self.jobs:
            raise ValueError("Unknown component")
        return self.config[name + "_unit"]

    def _health(self, invocation):
        try:
            value = json.loads((self.runtime_dir / "monitor-health.json").read_text())
            if (not invocation or value.get("invocation") != invocation or
                    not 0 <= time.time() - float(value.get("wall_time", 0)) <= self.config["stale_seconds"]):
                return {}
            return value
        except (OSError, ValueError, TypeError, AttributeError):
            return {}

    def component(self, name):
        unit = self.system.show(self._unit(name))
        state, substate = unit.get("ActiveState"), unit.get("SubState")
        occupied = self.probe("127.0.0.1", self.config[name + "_port"])
        active = state == "active"
        owned = active or state in {"activating", "deactivating"} or self.system.populated(unit)
        with self.lock:
            job, error = self.jobs[name], self.errors[name]
        details = {}
        if job:
            phase = "starting" if job["desired"] == "start" else "stopping"
        elif state == "deactivating":
            phase = "stopping"
        elif state == "activating":
            phase = "starting"
        elif not active and owned:
            phase, error = "failed", "进程组尚未清理完成；禁止重复启动"
        elif not active and occupied:
            phase, error = "conflict", "端口被非本管理器服务占用；不会自动终止未知进程"
        elif active and name == "web":
            phase = "running" if occupied and self.web_probe(self.config["web_port"]) else "degraded"
            if phase == "degraded":
                error = error or "Web 进程存在，但页面健康检查未通过"
        elif active:
            details = self._health(unit.get("InvocationID"))
            sdk = details.get("sdk_fresh") is True
            lidar = details.get("lidar_fresh") is True
            images = details.get("images_fresh") is True
            mc = details.get("mc_ready") is True
            phase = "connected" if occupied and sdk and lidar and images and mc and details.get("replay") is False else "degraded"
            if phase == "degraded":
                missing = [label for label, ready in (("SDK 状态", sdk), ("雷达", lidar), ("图像", images), ("MC 速度流", mc)) if not ready]
                error = error or ("回放模式，不是实机数据" if details.get("replay") else
                                  "等待" + "／".join(missing) if missing else "等待可视化桥连接")
        elif error or state == "failed":
            phase, error = "failed", error or "进程异常退出；确认原因后手动重试"
        else:
            phase = "stopped"
        return {"phase": phase, "active": active, "owned": owned, "error": error,
                "busy": phase in {"starting", "stopping"}, "health": details,
                "pid": int(unit.get("MainPID", 0)), "operation": job and job["id"],
                "unit_state": state, "unit_substate": substate}

    def snapshot(self):
        return {"api": 1, "instance": self.instance, "wall_time": time.time(),
                "monitor": self.component("monitor"), "web": self.component("web")}

    def request(self, name, desired, request_id, instance):
        if instance != self.instance:
            raise Conflict("管理器已重启，请刷新状态后重新点击")
        if desired not in {"start", "stop"}:
            raise ValueError("Unsupported desired state")
        self._unit(name)
        with self.lock:
            if request_id in self.requests:
                old = self.requests[request_id]
                if old != (name, desired):
                    raise Conflict("请求标识不能复用于不同操作")
                return  # Idempotent replay, including a lost HTTP response.
            if self.jobs[name]:
                raise Conflict("当前操作尚未完成；请等待状态确认")
            current = self.component(name)
            if current["phase"] == "conflict":
                raise Conflict(current["error"])
            if current["busy"]:
                raise Conflict("服务仍在转换状态")
            if desired == "start" and current["owned"] and not current["active"]:
                raise Conflict("旧进程组仍存在；必须先完成停止")
            self.requests[request_id] = (name, desired)
            while len(self.requests) > 128:
                self.requests.popitem(last=False)
            if (desired == "start" and current["active"]) or (desired == "stop" and not current["owned"]):
                if desired == "stop":
                    self.errors[name] = ""
                return
            self.errors[name] = ""
            job = {"id": request_id, "desired": desired}
            self.jobs[name] = job
            worker = threading.Thread(target=self._run, args=(name, job), daemon=True)
            self.threads.append(worker)
            self.threads = [thread for thread in self.threads if thread.is_alive() or thread is worker]
            worker.start()

    def _wait_stopped(self, name):
        end = time.monotonic() + self.config["stop_timeout_seconds"]
        while time.monotonic() < end:
            unit = self.system.show(self._unit(name))
            if unit.get("ActiveState") in {"inactive", "failed"} and not self.system.populated(unit):
                if self.probe("127.0.0.1", self.config[name + "_port"]):
                    raise RuntimeError("本服务进程组已退出，但端口仍被其他进程占用；未清理未知进程")
                return
            time.sleep(.15)
        raise RuntimeError("停止尚未确认：进程组未清空；禁止重复启动，请核对管理器日志")

    def _run(self, name, job):
        started = False
        try:
            if job["desired"] == "stop":
                self.system.action(self._unit(name), "stop")
                self._wait_stopped(name)
                return
            if name == "monitor":
                network_error = self.network_check(self.config)
                if network_error:
                    raise RuntimeError(network_error)
                missing = [f"{host}:{port}" for host, port in self.config["robot_endpoints"] if not self.probe(host, port, .8)]
                if missing:
                    raise RuntimeError("机器人网络不可达，请检查电源和有线连接（" + "、".join(missing) + "）")
            # Recheck immediately before starting; external services are never adopted.
            if self.probe("127.0.0.1", self.config[name + "_port"]):
                raise RuntimeError("端口已被占用；拒绝重复启动")
            started = True
            self.system.action(self._unit(name), "start")
            end = time.monotonic() + self.config["start_timeout_seconds"]
            seen_active = False
            while time.monotonic() < end:
                unit = self.system.show(self._unit(name))
                state = unit.get("ActiveState")
                if state == "failed" or (seen_active and state == "inactive"):
                    raise RuntimeError("服务启动失败或提前退出，请检查网络和启动日志")
                seen_active = seen_active or state == "active"
                occupied = self.probe("127.0.0.1", self.config[name + "_port"])
                if state == "active" and occupied:
                    if name == "web" and self.web_probe(self.config["web_port"]):
                        return
                    if name == "monitor" and self._health(unit.get("InvocationID")):
                        return  # The UI separately distinguishes real telemetry from merely running.
                time.sleep(.2)
            raise RuntimeError("启动超时，已请求清理本次启动的进程组")
        except Exception as exc:
            error = str(exc)
            if started:
                try:
                    self.system.action(self._unit(name), "stop")
                    self._wait_stopped(name)
                except Exception as cleanup:
                    error += "；清理未确认：" + str(cleanup)
            with self.lock:
                self.errors[name] = error[:600]
        finally:
            with self.lock:
                if self.jobs[name] is job:
                    self.jobs[name] = None
