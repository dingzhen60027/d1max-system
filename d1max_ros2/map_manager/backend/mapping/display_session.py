"""Resolve a Web-owned MOLA session without importing ROS or visualization libraries."""
import json
from pathlib import Path
import re


def active_session(app):
    root=Path(app)/'data/mola'
    state=json.loads((root/'state.json').read_text())
    identity=state.get('id','')
    if not re.fullmatch('[a-f0-9]{32}',identity):raise ValueError('Invalid MOLA task identity')
    path=root/identity
    if path.is_symlink() or not (path/'task.yaml').is_file():raise ValueError('Missing MOLA task')
    return path.resolve()
