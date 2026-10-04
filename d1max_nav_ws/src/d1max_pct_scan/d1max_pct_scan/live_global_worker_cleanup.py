"""Bounded, non-blocking cleanup of cancelled native PCT children.

Only the ROS timer reaps retired workers. A retired worker is never reused,
and a stale result can never be promoted to a new planning generation.
"""
from __future__ import annotations

import time


class RetiredChildren:
    def __init__(self, *, terminate_grace_s=.5, clock=time.monotonic):
        if not 0 < terminate_grace_s <= 2:
            raise ValueError('terminate_grace_s must be bounded')
        self.terminate_grace_s = terminate_grace_s
        self.clock = clock
        self.pending = []
        self._channels = {}

    def retire(self, child, pipe):
        """Close the reply channel and signal an owned child without waiting."""
        if pipe is not None:
            pipe.close()
        if child is None or child.pid is None:
            return
        child.join(timeout=0)
        if not child.is_alive() and getattr(pipe, 'retirement_ready', True):
            return
        if child.is_alive():
            try:
                child.terminate()
            except ProcessLookupError:
                pass
        child.join(timeout=0)
        if child.is_alive() or not getattr(pipe, 'retirement_ready', True):
            self.pending.append((child, self.clock()))
            self._channels[id(child)] = pipe

    def reap(self):
        """Single timer tick: join exited workers; SIGKILL tardy workers."""
        now = self.clock()
        remaining = []
        for child, terminated_at in self.pending:
            child.join(timeout=0)
            if child.is_alive() and now-terminated_at >= self.terminate_grace_s:
                try:
                    child.kill()
                except ProcessLookupError:
                    pass
                child.join(timeout=0)
            channel = self._channels.get(id(child))
            if child.is_alive() or not getattr(channel, 'retirement_ready', True):
                remaining.append((child, terminated_at))
            else:
                self._channels.pop(id(child), None)
        self.pending = remaining
        return len(remaining)

    def drain_on_shutdown(self, *, timeout_s=1.):
        """Called only during node shutdown, never from a ROS callback."""
        deadline = self.clock() + timeout_s
        while self.pending and self.clock() < deadline:
            self.reap()
            if self.pending:
                time.sleep(.01)
        for child, _ in self.pending:
            try:
                child.kill()
            except ProcessLookupError:
                pass
            child.join(timeout=0)
        self.reap()
        return len(self.pending)
