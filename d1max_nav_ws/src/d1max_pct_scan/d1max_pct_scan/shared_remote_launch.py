"""Manual remote attached to the already-running, sole SDK backend.

No SDK connection, control acquisition or robot command is created by opening
the panel. The backend advertises manual-control capability; explicit takeover
retires navigation before manual commands, and handback retains the connection.
A legacy emergency-only backend remains emergency-only in the same panel.
A failed/closed panel must not stop the navigation core.
"""
import os
from pathlib import Path


def command(session, *, environment=None):
    env=os.environ if environment is None else environment
    # A planning-only session must not open a control surface. Keep the same
    # navigation graph, but do not offer manual movement alongside its preview.
    if session.get('purpose',session.get('execution_purpose','execution'))=='planning_only':
        return None
    if session.get('transport_mode')!='live' or session.get('execution_architecture')!='task_follower_v1':
        return None
    if not (env.get('DISPLAY') or env.get('WAYLAND_DISPLAY')):
        return None
    # The selected default carries the exact reviewed GUI. Do not compile a
    # different checkout while starting navigation. The explicit override and
    # legacy standalone launcher remain useful outside a packaged session.
    if not env.get('D1MAX_REMOTE_ROOT') and env.get('D1MAX_RELEASE'):
        binary=Path(env['D1MAX_RELEASE'])/'remote/bin/d1max_remote_gui'
        if binary.is_file() and os.access(binary,os.X_OK):
            return [str(binary.resolve()),'--shared']
    root=Path(env.get('D1MAX_REMOTE_ROOT',str(Path.home()/'d1max_sdk_remote')))
    entry=root/'gui.sh'
    if not entry.is_file() or not os.access(entry,os.X_OK):
        return None
    # Explicit shared mode cannot fall back to creating a second SDK session.
    # The existing Qt single-instance handshake activates an already-open panel.
    return [str(entry.resolve()),'--shared']
