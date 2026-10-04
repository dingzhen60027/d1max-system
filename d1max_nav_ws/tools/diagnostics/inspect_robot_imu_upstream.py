#!/usr/bin/env python3
"""Read-only NX inspection using the official guide, never logging credentials."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import paramiko

VENDOR = Path(os.environ.get('D1MAX_VENDOR_ROOT') or Path.home()/'智元四足机器人D1 Max二次开发文档资料包v0.1.0')
GUIDE = str(VENDOR/'二次开发文档/智元四足机器人D1 Max SDK开发指南V0.0.9.pdf')
HOST = '192.168.168.100'


def guide_password():
    page = subprocess.check_output(
        ['/usr/bin/pdftotext', '-f', '24', '-l', '24', '-layout', GUIDE, '-'], text=True)
    line = next(line for line in page.splitlines()
                if 'ssh robot@'+HOST in line and 'Orin NX' in line)
    secret = re.search(r'password\s*[:：=]\s*([^\s\u200b]+)', line)
    if not secret:
        raise RuntimeError('Official guide authentication format was not recognized')
    return secret.group(1)


def connect():
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(HOST, username='robot', password=guide_password(),
                   look_for_keys=False, allow_agent=False, timeout=5,
                   auth_timeout=5, banner_timeout=5)
    return client


def run_readonly_root_python(client, script, *, timeout=30, ros_environment=False):
    """Used only for bounded passive capture and protected driver configuration."""
    command = '/usr/bin/python3 -c '+shlex.quote(script)
    if ros_environment:
        command = 'bash -c '+shlex.quote('. /opt/runtime/env.bash; exec '+command)
    stdin, stdout, stderr = client.exec_command('sudo -S -p "" '+command, timeout=timeout)
    stdin.write(guide_password()+'\n')
    stdin.flush()
    stdin.channel.shutdown_write()
    output, error = stdout.read(2000000), stderr.read(65536)
    status = stdout.channel.recv_exit_status()
    stdin.close(); stdout.close(); stderr.close()
    if status:
        raise RuntimeError('Read-only NX command failed: '+error.decode(errors='replace'))
    return output.decode()


def main():
    client = connect()
    try:
        command = """id -un
hostname
cat /opt/release/version.yaml
ps -eo pid,ppid,pcpu,stat,args | grep -E '[r]slidar|[r]mw_zenohd'
ip -s link show dev enx546c503e29e1
ip -s link show dev enx546c503e2938
command -v tcpdump
sudo -n true
"""
        _, stdout, stderr = client.exec_command(command, timeout=10)
        print(stdout.read(65536).decode(errors='replace'))
        print(stderr.read(8192).decode(errors='replace'))
        print('exit_status:', stdout.channel.recv_exit_status())
    finally:
        client.close()


if __name__ == '__main__':
    main()
