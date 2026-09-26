"""Pure steady-clock translation watchdog, independent of ROS and the planner.

Net displacement from an anchor is used instead of summing every odometry
sample: sub-threshold localization jitter must not masquerade as progress.
This is not a goal-distance watchdog; legitimate detours still count.
"""
import math


class TranslationProgressWatchdog:
    def __init__(self, timeout_s=20.0, distance_m=0.10):
        for name, value in [('timeout_s', timeout_s), ('distance_m', distance_m)]:
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.timeout_s = float(timeout_s)
        self.distance_m = float(distance_m)
        self.deactivate()

    @staticmethod
    def _sample(xy, now):
        if len(xy) != 2 or not all(math.isfinite(value) for value in xy):
            raise ValueError('Progress sample must contain finite map-frame XY')
        if not math.isfinite(now):
            raise ValueError('Progress timestamp must be finite steady-clock seconds')
        return tuple(float(value) for value in xy), float(now)

    def activate(self, xy, now):
        self.anchor, self.last_progress_at = self._sample(xy, now)
        self.last_observed_at = self.last_progress_at
        self.active = True
        self.blocked = False

    def deactivate(self):
        self.active = False
        self.blocked = False
        self.anchor = None
        self.last_progress_at = None
        self.last_observed_at = None

    def observe(self, xy, now):
        """Return True once blocked, until explicit deactivate/new activation."""
        if not self.active:
            return False
        xy, now = self._sample(xy, now)
        if now < self.last_observed_at:
            raise ValueError('Progress steady clock moved backwards')
        self.last_observed_at = now
        if self.blocked:
            return True
        if math.hypot(xy[0] - self.anchor[0], xy[1] - self.anchor[1]) >= self.distance_m:
            self.anchor, self.last_progress_at = xy, now
        elif now - self.last_progress_at >= self.timeout_s:
            self.blocked = True
        return self.blocked

    def snapshot(self):
        return {'active': self.active, 'blocked': self.blocked,
                'timeout_s': self.timeout_s, 'distance_m': self.distance_m,
                'seconds_without_translation_progress':
                    self.last_observed_at - self.last_progress_at if self.active else 0.0}
