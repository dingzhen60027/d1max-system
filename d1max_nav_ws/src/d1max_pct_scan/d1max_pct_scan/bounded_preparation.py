"""One worker, one replaceable waiting request, one result; no ROS ownership.

The caller owns admission/commit. A worker may wake that owner's executor, but
never delivers results or mutates ownership there. Source timestamps remain
literal; invalidate fences in-flight work.
"""
from dataclasses import dataclass
from threading import Condition, Thread


@dataclass(frozen=True)
class PreparedResult:
    ticket: int
    key: object
    value: object = None
    error: str = ''


class LatestPreparation:
    def __init__(self, prepare, notify_ready=None):
        self.prepare = prepare
        if notify_ready is not None and not callable(notify_ready):
            raise ValueError('preparation_ready_notification_must_be_callable')
        self.notify_ready = notify_ready
        self._cv = Condition()
        self._ticket = 0
        self._request = self._result = None
        self._closed = False
        self._thread = Thread(target=self._run, name='reference-support-preparation', daemon=True)
        self._thread.start()

    def submit(self, key, value):
        with self._cv:
            if self._closed:
                raise RuntimeError('preparation_worker_closed')
            self._ticket += 1
            self._request = (self._ticket, key, value)
            self._result = None
            self._cv.notify()
            return self._ticket

    def invalidate(self):
        with self._cv:
            self._ticket += 1
            self._request = self._result = None

    def take(self):
        with self._cv:
            result, self._result = self._result, None
            return result

    def close(self, timeout=.5):
        with self._cv:
            self._closed = True
            self._ticket += 1
            self._request = self._result = None
            self._cv.notify_all()
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._closed or self._request is not None)
                if self._closed:
                    return
                ticket, key, value = self._request
                self._request = None
            try:
                result = PreparedResult(ticket, key, self.prepare(value))
            except Exception as error:
                result = PreparedResult(ticket, key, error=type(error).__name__+':'+str(error))
            with self._cv:
                if not self._closed and ticket == self._ticket:
                    self._result = result
                    self._cv.notify_all()
                    notify = self.notify_ready
                else:
                    notify = None
            # Only wake the serialized owner; never deliver, commit or change
            # source/receipt timestamps on this worker thread. A concurrent
            # cancel may invalidate the slot before the owner takes it, which
            # is harmless. The ordinary owner timer remains a fallback if its
            # executor/context is already shutting down.
            if notify is not None:
                try:
                    notify()
                except Exception:
                    pass
