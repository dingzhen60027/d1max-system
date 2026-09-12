"""Lifecycle tests use fake units and sockets; no ROS or robot commands."""
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer

from manager.runtime import Manager, Conflict
from manager.server import handler_for

class FakeSystem:
    def __init__(self):
        self.states = {}
        self.calls = []
        self.ports = set()
        self.defer_stop = False
        self.start_ready = True
    def show(self, unit):
        return dict(self.states.get(unit, {"ActiveState": "inactive", "MainPID": "0"}))
    def populated(self, state):
        return bool(state.get("populated"))
    def action(self, unit, verb):
        self.calls.append((unit, verb))
        name = "web" if "web" in unit else "monitor"
        port = 8766 if name == "web" else 8769
        if verb == "start":
            self.states[unit] = {"ActiveState": "active", "MainPID": "123", "InvocationID": "session", "populated": True}
            if self.start_ready:
                self.ports.add(port)
        elif self.defer_stop:
            self.states[unit] = {"ActiveState": "inactive", "MainPID": "0", "populated": True}
        else:
            self.states[unit] = {"ActiveState": "inactive", "MainPID": "0"}
            self.ports.discard(port)

class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.system = FakeSystem()
        self.config = json.loads((Path(__file__).parents[1] / "config/manager.json").read_text())
        self.config.update(start_timeout_seconds=.3, stop_timeout_seconds=.4)
        self.robot_online = True
        self.manager = Manager(self.config, self.tmp.name, system=self.system,
            probe=lambda host, port, *_: port in self.system.ports if host == "127.0.0.1" else self.robot_online,
            web_probe=lambda port: port in self.system.ports, network_check=lambda _config: "")
    def request(self, name="web", desired="start", rid="request_0123456789"):
        self.manager.request(name, desired, rid, self.manager.instance)
    def finish(self):
        for thread in self.manager.threads:
            thread.join(2)
            self.assertFalse(thread.is_alive())
    def test_start_stop_idempotent_no_stacking(self):
        self.request()
        self.finish()
        self.request()
        self.request(rid="another_request_01234")
        self.assertEqual(len(self.system.calls), 1)
        self.assertEqual(self.manager.component("web")["phase"], "running")
        self.request(desired="stop", rid="stop_request_01234")
        self.finish()
        self.assertEqual(self.manager.component("web")["phase"], "stopped")
        self.assertFalse(self.system.ports)
        self.assertEqual(len(self.system.calls), 2)
    def test_conflicting_request_id_and_old_manager(self):
        self.request()
        self.finish()
        with self.assertRaises(Conflict):
            self.request(desired="stop")
        with self.assertRaises(Conflict):
            self.manager.request("web", "stop", "new_request_01234", "previous")
    def test_unknown_port_is_never_killed_or_adopted(self):
        self.system.ports.add(8766)
        self.assertEqual(self.manager.component("web")["phase"], "conflict")
        for desired in ("start", "stop"):
            with self.assertRaises(Conflict):
                self.request(desired=desired)
        self.assertEqual(self.system.calls, [])
    def test_offline_robot_never_starts_workers(self):
        self.robot_online = False
        self.request("monitor")
        self.finish()
        self.assertEqual(self.system.calls, [])
        self.assertEqual(self.manager.component("monitor")["phase"], "failed")
        self.assertIn("网络不可达", self.manager.component("monitor")["error"])
    def test_proxy_tcp_success_cannot_bypass_physical_link_check(self):
        self.manager.network_check = lambda _config: "机器人网络不可达：代理路由不是有线链路"
        self.request("monitor")
        self.finish()
        self.assertEqual(self.system.calls, [])
        self.assertIn("代理路由", self.manager.component("monitor")["error"])
    def test_start_timeout_rolls_back_cgroup(self):
        self.system.start_ready = False
        self.request()
        self.finish()
        self.assertEqual([x[1] for x in self.system.calls], ["start", "stop"])
        self.assertFalse(self.manager.component("web")["owned"])
        self.assertEqual(self.manager.component("web")["phase"], "failed")
    def test_double_click_during_start_is_serialized(self):
        self.system.start_ready = False
        self.request()
        with self.assertRaises(Conflict):
            self.request(rid="duplicate_0123456789")
        self.finish()
        self.assertEqual([x[1] for x in self.system.calls], ["start", "stop"])
    def test_stop_waits_for_children_not_just_parent_pid(self):
        self.request()
        self.finish()
        self.system.defer_stop = True
        self.request(desired="stop", rid="stop_request_01234")
        self.finish()
        state = self.manager.component("web")
        self.assertEqual(state["phase"], "failed")
        self.assertTrue(state["owned"])
        with self.assertRaises(Conflict):
            self.request(rid="restart_request_01234")
    def test_only_current_fresh_complete_nonreplay_health_connects(self):
        self.system.action(self.config["monitor_unit"], "start")
        path = Path(self.tmp.name) / "monitor-health.json"
        valid = dict(invocation="session", wall_time=time.time(), sdk_fresh=True, lidar_fresh=True, images_fresh=True, mc_ready=True, replay=False)
        for extra in ({"invocation":"old"}, {"wall_time":0}, {"sdk_fresh":False}, {"lidar_fresh":False}, {"images_fresh":False}, {"mc_ready":False}, {"replay":True}):
            path.write_text(json.dumps({**valid, **extra}))
            self.assertEqual(self.manager.component("monitor")["phase"], "degraded")
        path.write_text(json.dumps(valid))
        self.assertEqual(self.manager.component("monitor")["phase"], "connected")
        path.write_text(json.dumps({**valid, "mc_ready": False}))
        self.assertIn("MC 速度流", self.manager.component("monitor")["error"])
        self.assertEqual(self.system.calls, [(self.config["monitor_unit"], "start")])  # no restart to repair telemetry
        path.write_text("[]")
        self.assertEqual(self.manager.component("monitor")["phase"], "degraded")
    def test_http_auth_origin_host_fixed_routes_and_schema(self):
        token = "test-capability-" * 4
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(self.manager, token, 0))
        port = server.server_address[1]
        server.RequestHandlerClass = handler_for(self.manager, token, port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        opener = build_opener(ProxyHandler({}))
        def call(path="/v1/status", method="GET", extra=None, body=None, auth=True):
            headers = {"Origin":"null", **({"Authorization":"Bearer "+token} if auth else {}), **(extra or {})}
            if body is not None:
                headers["Content-Type"] = "application/json"
            try:
                with opener.open(Request(f"http://127.0.0.1:{port}"+path, method=method, headers=headers, data=json.dumps(body).encode() if body is not None else None), timeout=2) as response:
                    return response.status
            except HTTPError as error:
                return error.code
        try:
            self.assertEqual(call(), 200)
            self.assertEqual(call(auth=False), 403)
            self.assertEqual(call(extra={"Origin":"https://evil.example"}), 403)
            self.assertEqual(call(extra={"Host":"evil.example"}), 403)
            self.assertEqual(call(extra={"Authorization":"Bearer é"}), 403)
            self.assertEqual(call("/v1/robot/stand", "POST"), 404)
            self.assertEqual(call("/v1/web/start", "POST", body={"command":"sh"}), 400)
            self.assertEqual(self.system.calls, [])
            body={"request_id":"http_request_012345", "instance":self.manager.instance}
            self.assertEqual(call("/v1/web/start", "POST", body=body), 202)
            self.finish()
            self.assertEqual(call("/v1/web/start", "POST", body=body), 202)
            self.assertEqual(len(self.system.calls), 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2)

if __name__ == "__main__":
    unittest.main()
