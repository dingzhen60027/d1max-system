"""Bounded local presentation requests; never a navigation/motion interface."""
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import time
import uuid

VIEW_CONTRACT = 'single_window_layout_v1'
VIEWER_ENV = 'D1MAX_NAV_RVIZ_VIEWER_ID'
REQUEST_FILE = 'navigation-view-layout-request.json'
STATE_FILE = 'navigation-view-layout-state.json'
TOKEN = re.compile(r'[0-9a-f]{32}')


def write_layout_request(directory, *, session_id, viewer_id, layout):
    if layout not in ('global', 'local') or not TOKEN.fullmatch(viewer_id):
        raise ValueError('invalid_view_layout_request')
    if not isinstance(session_id, str) or not session_id or len(session_id) > 128:
        raise ValueError('invalid_view_session')
    directory = Path(directory).resolve(strict=True)
    path = directory / REQUEST_FILE
    if path.is_symlink():
        raise ValueError('view_request_not_private')
    if path.exists():
        info = path.stat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError('view_request_not_private')
    value = dict(schema=1, session_id=session_id, viewer_id=viewer_id,
                 request_id=uuid.uuid4().hex, layout=layout, stamp=time.time())
    raw = (json.dumps(value) + '\n').encode()
    fd, temporary = tempfile.mkstemp(prefix='.view-layout-', dir=directory)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return value


def viewer_environment(directory, session, layout, *, environment=None):
    """Web owns its requests; a direct CLI view issues its own initial request."""
    if session.get('view_contract') != VIEW_CONTRACT:
        raise ValueError('single_window_view_contract_required')
    result = dict(os.environ if environment is None else environment)
    viewer_id = result.get(VIEWER_ENV)
    if viewer_id is not None:
        if not TOKEN.fullmatch(viewer_id):
            raise ValueError('invalid_viewer_identity')
        # Do not overwrite the Web's newer request while the viewer starts.
    else:
        viewer_id = uuid.uuid4().hex
        result[VIEWER_ENV] = viewer_id
        write_layout_request(directory, session_id=session['id'], viewer_id=viewer_id, layout=layout)
    return result
