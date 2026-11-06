"""Bounded history for the existing BT ``session.counter`` task identity.

The navigator is the sole task producer and uses an increasing uint64 counter
within one session. A high delivery sequence does not make an old task new.
Older compatibility callers without this identity retain an explicit bounded
set; forgetting arbitrary names would permit their resurrection.
"""

UINT64_MAX = (1 << 64) - 1


def task_ordinal(session_id, task_id):
    """Return the existing BT ordinal, or None for a legacy task name.

    The complete session prefix matters: a session may itself contain dots.
    Reserved-prefix aliases never fall back to the legacy identity domain.
    """
    if not isinstance(task_id, str) or not task_id:
        raise ValueError('task_identity_invalid')
    prefix = session_id + '.'
    if not task_id.startswith(prefix):
        return None
    suffix = task_id[len(prefix):]
    if (not suffix or suffix[0] == '0' or len(suffix) > 20
            or any(c < '0' or c > '9' for c in suffix)):
        raise ValueError('task_identity_sequence_invalid')
    ordinal = int(suffix)
    if ordinal > UINT64_MAX:
        raise ValueError('task_identity_sequence_invalid')
    return ordinal


class BoundedTaskHistory:
    """Monotonic formal-task watermark plus bounded legacy tombstones.

    ``legacy_key`` permits a caller to preserve its pre-existing legacy key
    semantics. Formal identities are always fenced at task granularity, even
    when a replay changes its route id/hash. No source time is stored/renewed.
    """
    def __init__(self, session_id, *, legacy_limit):
        if not session_id or type(legacy_limit) is not int or legacy_limit < 1:
            raise ValueError('task_history_configuration_invalid')
        self.session_id = session_id
        self.legacy_limit = legacy_limit
        self.through = 0
        self.legacy = set()

    def contains(self, task_id, *, legacy_key=None):
        ordinal = task_ordinal(self.session_id, task_id)
        if ordinal is not None:
            return ordinal <= self.through
        return (task_id if legacy_key is None else legacy_key) in self.legacy

    def can_remember(self, task_id, *, legacy_key=None):
        ordinal = task_ordinal(self.session_id, task_id)
        key = task_id if legacy_key is None else legacy_key
        return (ordinal is not None or key in self.legacy
                or len(self.legacy) < self.legacy_limit)

    def remember(self, task_id, *, legacy_key=None):
        ordinal = task_ordinal(self.session_id, task_id)
        if ordinal is not None:
            self.through = max(self.through, ordinal)
            return
        key = task_id if legacy_key is None else legacy_key
        if key not in self.legacy and len(self.legacy) >= self.legacy_limit:
            raise ValueError('task_history_capacity_exceeded')
        self.legacy.add(key)
