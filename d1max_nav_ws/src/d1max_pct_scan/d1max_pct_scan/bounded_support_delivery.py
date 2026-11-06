"""Bounded delivery of immutable, non-authoritative support geometry.

One in-flight send and one replaceable waiting value. Cancellation removes
waiting work, never waits behind middleware. An already-entered send may
finish after cancel; Native's task/version tombstone rejects that static data.
This worker cannot send a proposal, command, grant, or retirement ACK.
"""
from dataclasses import dataclass
from threading import Condition, Thread


@dataclass(frozen=True)
class SupportDeliveryResult:
    key: object
    error: str = ''


class BoundedSupportDelivery:
    def __init__(self, publish, notify_ready=None):
        self.publish, self.notify_ready = publish, notify_ready
        self._cv = Condition()
        self._waiting = self._result = None
        self._inflight_key = None
        self._closed = False
        self._thread = Thread(target=self._run, name='reference-support-delivery', daemon=True)
        self._thread.start()

    def submit(self, key, support):
        # Ownership transfer: neither caller nor worker mutates this message.
        with self._cv:
            if self._closed:
                raise RuntimeError('support_delivery_closed')
            displaced = self._waiting[0] if self._waiting is not None else None
            self._waiting = (key, support)
            self._cv.notify()
            return displaced

    def invalidate(self):
        with self._cv:
            self._waiting = self._result = None

    def take(self):
        with self._cv:
            result, self._result = self._result, None
            return result

    def take_state(self):
        """One atomic completion/ownership snapshot, including dropped values.

        Completion is coalesced, never a retirement/acceptance fact. The owner
        can clear in-flight flags even when an older completion was replaced;
        it must not infer successful publication for those omitted keys.
        """
        with self._cv:
            result, self._result = self._result, None
            pending = tuple(key for key in (
                self._inflight_key,
                self._waiting[0] if self._waiting is not None else None)
                if key is not None)
            return result, pending

    def close(self, timeout=.5):
        with self._cv:
            self._closed = True
            self._waiting = self._result = None
            self._cv.notify_all()
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._closed or self._waiting is not None)
                if self._closed:
                    return
                key, value = self._waiting
                self._waiting = None
                self._inflight_key = key
            error = ''
            try:
                self.publish(value)
            except Exception as exc:
                error = type(exc).__name__ + ':' + str(exc)
            with self._cv:
                self._inflight_key = None
                if self._closed:
                    return
                self._result = SupportDeliveryResult(key, error)
                self._cv.notify_all()
            if self.notify_ready is not None:
                try:
                    self.notify_ready()
                except Exception:
                    pass
