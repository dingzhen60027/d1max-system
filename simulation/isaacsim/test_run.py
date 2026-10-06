"""An accepted shutdown request must not become an invented retirement pass."""
import json

import pytest

from run import require_retired_shutdown


def test_shutdown_requires_matching_actual_retirement(tmp_path):
    record = dict(session_id='fixture', request_accepted=True, software_retired=False,
        physical_stop_confirmed=False, reason='task_owner_drain_timeout')
    path = tmp_path/'shutdown.json'
    path.write_text(json.dumps(record))
    with pytest.raises(RuntimeError, match='retirement unconfirmed'):
        require_retired_shutdown(tmp_path, 'fixture')
    record['software_retired'] = True
    path.write_text(json.dumps(record))
    with pytest.raises(RuntimeError, match='retirement unconfirmed'):
        require_retired_shutdown(tmp_path, 'other-session')
    accepted = require_retired_shutdown(tmp_path, 'fixture')
    assert accepted['software_retired'] is True
    assert accepted['physical_stop_confirmed'] is False
