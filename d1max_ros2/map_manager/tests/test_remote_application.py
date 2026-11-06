"""Desktop launcher only; no production app, robot or SDK imports."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient
from backend.remote_application import RemoteApplication, create_remote_router


class RemoteApplicationTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="d1max-remote-test-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / "gui.sh").touch(mode=0o700)
        self.calls = []
        self.ready = None
        self.ack = True
        self.state = "active"
        self.fail_launch = False
        self.remote = RemoteApplication(self.root, runner=self.run_command, timeout=.02)
        environment = patch.dict(os.environ, {"DISPLAY": ":0", "XAUTHORITY": "/tmp/auth",
                                               "UNRELATED_SECRET": "not-forwarded"}, clear=True)
        environment.start()
        self.addCleanup(environment.stop)

    def run_command(self, command, **kwargs):
        self.calls.append(command)
        if command[0] == "systemd-run":
            if self.fail_launch:
                raise subprocess.CalledProcessError(1, command)
            self.ready = Path(command[-1])
            if self.ack:
                self.ready.write_text("ready\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, self.state + "\n", "")

    def test_waits_for_ack_and_only_opens_independent_gui(self):
        self.assertEqual(self.remote.open(), {"status": "opened"})
        command = self.calls[0]
        self.assertEqual(command[-3:-1], [str(self.root / "gui.sh"), "--ready-file"])
        self.assertIn("--setenv=DISPLAY=:0", command)
        self.assertNotIn("not-forwarded", " ".join(command))
        self.assertNotIn("BindsTo", " ".join(command))
        self.assertNotIn("remote.sh", " ".join(command))
        self.assertFalse(self.ready.parent.exists())
        self.assertFalse(self.remote.lock.locked())

    def test_concurrent_open_does_not_start_another_process(self):
        self.remote.lock.acquire()
        try:
            self.assertEqual(self.remote.open(), {"status": "opening"})
            self.assertEqual(self.calls, [])
        finally:
            self.remote.lock.release()

    def test_unacknowledged_exit_or_timeout_only_retires_own_launcher(self):
        for state in ("failed", "inactive", "active"):
            with self.subTest(state=state):
                self.calls.clear()
                self.ack = False
                self.state = state
                with self.assertRaises(HTTPException) as caught:
                    self.remote.open()
                self.assertEqual(caught.exception.status_code, 503)
                allocated = next(arg.split("=", 1)[1] for arg in self.calls[0] if arg.startswith("--unit="))
                self.assertEqual(self.calls[-1], ["systemctl", "--user", "stop", "--no-block", allocated])
                self.assertFalse(self.remote.lock.locked())

    def test_launch_failure_allows_retry(self):
        self.fail_launch = True
        with self.assertRaises(HTTPException):
            self.remote.open()
        self.fail_launch = False
        self.assertEqual(self.remote.open(), {"status": "opened"})

    def test_missing_gui_and_desktop_are_not_success(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(HTTPException) as caught:
                self.remote.open()
            self.assertEqual(caught.exception.status_code, 503)
        (self.root / "gui.sh").unlink()
        with self.assertRaises(HTTPException) as caught:
            self.remote.open()
        self.assertEqual(caught.exception.status_code, 404)
        self.assertEqual(self.calls, [])


class RemoteRouterTests(unittest.TestCase):
    def setUp(self):
        self.remote = Mock()
        self.remote.open.return_value = {"status": "opened"}
        app = FastAPI()
        app.include_router(create_remote_router(self.remote))
        self.client = TestClient(app, base_url="http://127.0.0.1:8766")
        self.addCleanup(self.client.close)

    def test_local_json_click(self):
        response = self.client.post("/api/remote/open", json={}, headers={"Origin": "http://127.0.0.1:8766"})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {"status": "opened"})
        self.remote.open.assert_called_once_with()

    def test_foreign_site_form_and_command_arguments_cannot_launch(self):
        for options, code in (
            ({"json": {}, "headers": {"Origin": "https://example.org"}}, 403),
            ({"json": {}, "headers": {"Host": "foreign.example:8766"}}, 403),
            ({"data": "{}"}, 415),
            ({"json": {"command": "remote.sh", "connect": True}}, 422),
        ):
            with self.subTest(options=options):
                self.assertEqual(self.client.post("/api/remote/open", **options).status_code, code)
        self.remote.open.assert_not_called()


if __name__ == "__main__":
    unittest.main()
