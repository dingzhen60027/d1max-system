"""Opt-in QA for the exact localization Systemd launcher; no robot or map writes."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from backend import localization

if os.environ.get('D1MAX_LOCALIZATION_QA')!='1':raise SystemExit('Set D1MAX_LOCALIZATION_QA=1 for isolated process-cleanup QA')
unit='d1max-localization-qa-'+uuid.uuid4().hex[:12]+'.service'
localization.UNIT=unit
system=localization.Systemd()
with tempfile.TemporaryDirectory(prefix='d1max-loc-cgroup-') as temporary:
    session=Path(temporary);script=session/'fixture.sh'
    script.write_text("#!/bin/bash\nexec /usr/bin/python3 -c 'import os,signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); pid=os.fork(); os.setsid() if pid==0 else None; time.sleep(90)'\n")
    try:
        system.start(session,script,session/'unused.yaml',session/'unused.pcd')
        pids=[];end=time.monotonic()+5
        while time.monotonic()<end:
            value=system.show(unit);group=value.get('ControlGroup','')
            if group:
                path=Path('/sys/fs/cgroup')/group.lstrip('/')
                if (path/'cgroup.procs').exists():pids=(path/'cgroup.procs').read_text().split()
                if len(pids)==2:break
            time.sleep(.05)
        assert len(pids)==2,pids
        assert len({os.getsid(int(pid)) for pid in pids})==2
        before=time.monotonic();system.stop();after=system.show(unit)
        assert not system.populated(after) and int(after.get('MainPID',0))==0,after
        assert all(not (Path('/proc')/pid).exists() for pid in pids)
        print(json.dumps({'unit':unit,'passed':True,'separate_process_sessions':True,'cgroup_empty':True,'stop_seconds':round(time.monotonic()-before,2),'robot_commands':0}))
    finally:
        subprocess.run(['systemctl','--user','stop',unit],capture_output=True,timeout=18)
        subprocess.run(['systemctl','--user','reset-failed',unit],capture_output=True,timeout=5)
