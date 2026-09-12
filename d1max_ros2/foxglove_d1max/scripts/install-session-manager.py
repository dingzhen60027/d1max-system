#!/usr/bin/env python3
"""Install a per-user idle supervisor; never auto-start robot communication."""
import json
import os
from pathlib import Path
import secrets
import subprocess

base = Path(__file__).resolve().parents[1]
project = base.parents[1]
units = Path.home() / ".config/systemd/user"
units.mkdir(parents=True, exist_ok=True)
credential = base / "config/manager.local.json"
if not credential.exists():
    with os.fdopen(os.open(credential, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600), "w") as output:
        json.dump({"token": secrets.token_urlsafe(48)}, output)
os.chmod(credential, 0o600)


def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'


content = {
    "d1max-session-manager.service": f'''[Unit]
Description=D1 Max local lifecycle manager (idle until explicit Connect)
StartLimitIntervalSec=30
StartLimitBurst=4

[Service]
Type=simple
WorkingDirectory={str(base).replace('%', '%%')}
ExecStart=/usr/bin/python3 -m manager.server --config {quote(base / "config/manager.json")} --credential {quote(credential)} --runtime-dir %t/d1max-session
RuntimeDirectory=d1max-session
RuntimeDirectoryMode=0700
RuntimeDirectoryPreserve=restart
Restart=on-failure
RestartSec=2
KillMode=control-group
TimeoutStopSec=8
UMask=0077

[Install]
WantedBy=default.target
''',
    "d1max-monitor-managed.service": f'''[Unit]
Description=D1 Max owned communication, SDK monitoring and PCD display
BindsTo=d1max-session-manager.service
After=d1max-session-manager.service

[Service]
Type=simple
WorkingDirectory={str(base).replace('%', '%%')}
ExecStart=/usr/bin/bash {quote(base / "scripts/start_live_monitor.sh")}
Environment=D1MAX_MANAGED_MONITOR=1
Restart=no
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=15
SendSIGKILL=yes
UMask=0077
''',
    "d1max-web-managed.service": f'''[Unit]
Description=D1 Max owned map-processing Web and all its child processes
BindsTo=d1max-session-manager.service
After=d1max-session-manager.service

[Service]
Type=simple
WorkingDirectory={str(project).replace('%', '%%')}
ExecStart=/usr/bin/bash {quote(project / "start_d1max_map_manager.sh")}
Restart=no
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=30
SendSIGKILL=yes
UMask=0077
''',
}
for name, text in content.items():
    target = units / name
    if target.exists() and "D1 Max" not in target.read_text():
        raise RuntimeError("Refusing to overwrite an unrelated user service: " + str(target))
    target.write_text(text, encoding="utf-8")
subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
subprocess.run(["systemctl", "--user", "enable", "--now", "d1max-session-manager.service"], check=True)
print("Installed idle local manager; monitor and Web remain manual. Credential value not printed.")
