"""Ordered atomic navigation reception, without renewing measurement leases.

Malformed/foreign/queued packets are not localization events. A well-formed,
new source-timed ``usable=False`` envelope is an explicit unavailable event,
even when no valid pose or fresh IMU can be supplied after loss of tracking.
"""
from dataclasses import dataclass
import math

from .atomic_projection_state import stamp_ns, validate


# Exact producer reason codes, not keyword guesses from human diagnostics.
HARD_UNAVAILABLE_REASONS = frozenset(('hard_localization_lost', 'lio_reset', 'localization_reset'))


def uses_atomic_navigation(params):
    """State wire contract, independent of map topology or movement authority.

    Keep the existing single-floor execution profile compatible; the neutral
    name is used by cross-floor planning previews. Both consume schema-2 exact
    local/global state pairs. Neither option grants execution permission.
    """
    return params.get('pipeline_contract') in ('single_floor_v3', 'atomic_navigation_v3')


@dataclass(frozen=True)
class NavigationEvent:
    identity: tuple
    source_ns: int
    received: float
    state: object
    reason: str

    @property
    def hard_failure(self):
        return self.state is None and self.reason in HARD_UNAVAILABLE_REASONS


class AtomicNavigationInbox:
    def __init__(self):
        self.latest = None

    def accept(self, message, *, session_id, map_version_id, now_ns, monotonic):
        """Return a new trusted event, or None without modifying any watermark."""
        try:
            if (type(message.schema_version) is not int or message.schema_version != 2
                    or type(message.usable) is not bool
                    or message.session_id != session_id or message.map_version_id != map_version_id
                    or type(message.localization_epoch) is not int or message.localization_epoch < 1
                    or not isinstance(message.localization_seed_id, str)
                    or not 0 < len(message.localization_seed_id) <= 256
                    or not math.isfinite(monotonic)):
                return None
            source_ns = stamp_ns(message.source_stamp)
            if source_ns <= 0 or not 0 <= now_ns-source_ns <= 400_000_000:
                return None
            identity = (session_id, message.localization_epoch, message.localization_seed_id)
            if not navigation_event_is_new(identity,source_ns,self.latest):
                return None
            if message.usable:
                state = validate(message, session_id=session_id, map_version_id=map_version_id,
                                 now_ns=now_ns)
                reason = ''
            else:
                # No geometry is consumed from an unavailable event. In
                # particular a lost IMU must not prevent revocation merely
                # because its last measurement is now stale or absent.
                reason = getattr(message, 'reason', '')
                if not isinstance(reason, str) or len(reason) > 1024:
                    return None
                state, reason = None, reason or 'atomic_navigation_unavailable'
            event = NavigationEvent(identity, source_ns, monotonic, state, reason)
        except (ValueError, TypeError, AttributeError, OverflowError):
            return None
        self.latest = event
        return event

    def usable(self, *, now_ns, monotonic, receipt_timeout_s):
        event = self.latest
        if event is None or event.state is None:
            return False
        state = event.state
        # Match the production atomic input contract at every use, not only
        # when received. Repeated packets cannot extend any of these leases.
        return (0 <= monotonic-event.received <= receipt_timeout_s
                and 0 <= now_ns-state.source_ns <= min(400_000_000, round(receipt_timeout_s*1e9))
                and 0 <= now_ns-state.posterior_ns <= 400_000_000
                and 0 <= now_ns-state.imu_ns <= 100_000_000)


def navigation_event_is_new(identity,source_ns,previous):
    """Order epoch before source time; a fresh reset is not a queued old frame.

    Call only AFTER identity and source freshness validation. Admitting a new
    epoch here exposes its reset to downstream revocation; it grants nothing.
    """
    return (previous is None or identity[1]>previous.identity[1]
        or (identity[1]==previous.identity[1] and source_ns>previous.source_ns))
