"""Bounded, identity-bound pause policy for visualization-only global goals.

This module deliberately preserves only a user goal, never a worker result,
native SCAN reference, spline, sensor packet, or stale robot pose.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GoalIdentity:
    session_id: str
    map_version_id: str
    epoch: int
    confirmed_seed: str
    frame_id: str
    tomogram_sha256: str


@dataclass(frozen=True)
class GoalIntent:
    user_stamp: float
    kind: str
    frame_id: str
    xyz: tuple[float, float, float]
    floor_id: str
    identity: GoalIdentity


def hard_identity_issue(expected, *, localizer, navigation, scan, body_frame=None):
    """Only explicit contradictory identity/fault evidence is permanent.

    A missing seed or temporarily invalid status is not proof of a new seed.
    Freshness/health admission is checked separately before any resumption.
    """
    if not isinstance(expected, GoalIdentity):
        return 'goal_identity_missing'
    sources = (localizer if isinstance(localizer, dict) else {},
               navigation if isinstance(navigation, dict) else {},
               scan if isinstance(scan, dict) else {})
    loc, nav, bridge = sources
    for name, value, wanted in (
        ('localizer_session_changed', loc.get('session_id'), expected.session_id),
        ('localizer_map_changed', loc.get('map_version_id'), expected.map_version_id),
        ('localizer_epoch_changed', loc.get('local_epoch'), expected.epoch),
        ('localizer_active_seed_changed', loc.get('active_seed_ns'), expected.confirmed_seed),
        ('localizer_confirmed_seed_changed', loc.get('confirmed_seed_ns'), expected.confirmed_seed),
        ('navigation_epoch_changed', nav.get('epoch'), expected.epoch),
        ('navigation_seed_changed', nav.get('seed_id'), expected.confirmed_seed),
        ('scan_session_changed', bridge.get('session_id'), expected.session_id),
        ('scan_localization_session_changed', bridge.get('localization_session_id'), expected.session_id),
        ('scan_epoch_changed', bridge.get('localization_epoch'), expected.epoch),
        ('scan_seed_changed', bridge.get('localization_seed_id'), expected.confirmed_seed),
        ('scan_frame_changed', bridge.get('frame_id'), expected.frame_id),
        ('body_frame_changed', body_frame, expected.frame_id),
    ):
        if value is not None and value != '' and value != wanted:
            return name
    frames = loc.get('frames') if isinstance(loc.get('frames'), dict) else {}
    if frames.get('map') not in (None, '', expected.frame_id):
        return 'localizer_map_frame_changed'
    for name, fault in (('localizer_fault', loc.get('local_fault')),
                        ('frontend_fault', (loc.get('frontend') or {}).get('fault')
                         if isinstance(loc.get('frontend'), dict) else None),
                        ('navigation_fault', nav.get('fault'))):
        if fault:
            return name
    if bridge.get('motion_enabled') is True:
        return 'scan_motion_contract_changed'
    if nav.get('reset_pending') is True or loc.get('reset_pending') is True:
        return 'localization_reset_pending'
    return None


def confirmed_goal_identity(localizer, *, session_id, frame_id, tomogram_sha256,
                            now, freshness_s):
    """Fresh explicit localizer proof; no identity inferred from nav.valid."""
    if not isinstance(localizer, dict) or localizer.get('session_id') != session_id:
        raise ValueError('fresh_localizer_session_required')
    stamp = localizer.get('wall_time')
    if (type(stamp) not in (int, float) or not math.isfinite(stamp)
            or not -.1 <= now-stamp <= freshness_s):
        raise ValueError('fresh_localizer_status_required')
    seed = localizer.get('confirmed_seed_ns')
    frames = localizer.get('frames')
    if (type(localizer.get('local_epoch')) is not int or localizer['local_epoch'] < 1
            or not isinstance(seed, str) or not seed
            or localizer.get('active_seed_ns') != seed
            or type(localizer.get('verified_confirmations')) is not int
            or localizer['verified_confirmations'] < 3
            or not isinstance(localizer.get('map_version_id'), str)
            or not localizer['map_version_id']
            or not isinstance(frames, dict) or frames.get('map') != frame_id
            or localizer.get('local_fault')
            or (isinstance(localizer.get('frontend'), dict)
                and localizer['frontend'].get('fault'))):
        raise ValueError('confirmed_localizer_identity_required')
    return GoalIdentity(session_id, localizer['map_version_id'],
                        localizer['local_epoch'], seed, frame_id, tomogram_sha256)


class BoundedGoalPause:
    """One immutable goal; a pause never authorizes an old path to reappear."""
    def __init__(self, ttl_s=2., good_samples=3, good_span_s=.2):
        if (not .2 <= ttl_s <= 2. or type(good_samples) is not int
                or not 2 <= good_samples <= 10 or not .1 <= good_span_s <= ttl_s):
            raise ValueError('invalid_bounded_pause_policy')
        self.ttl_s = ttl_s
        self.good_samples = good_samples
        self.good_span_s = good_span_s
        self.intent = None
        self.paused_at = None
        self.good_count = 0
        self.good_since = None
        self.last_nav_sample = None

    def install(self, intent):
        if not isinstance(intent, GoalIntent):
            raise ValueError('immutable_goal_intent_required')
        self.clear()
        self.intent = intent

    def clear(self):
        self.intent = None
        self.paused_at = None
        self.reset_good()

    def reset_good(self):
        self.good_count = 0
        self.good_since = None
        self.last_nav_sample = None

    def pause(self, now):
        if self.intent is None or not math.isfinite(now):
            return False
        first = self.paused_at is None
        if first:
            self.paused_at = now
        self.reset_good()
        return first

    def expired(self, now):
        return self.paused_at is not None and (not math.isfinite(now)
                                               or now-self.paused_at >= self.ttl_s)

    def observe_good(self, now, navigation_sample):
        if self.intent is None or self.paused_at is None or self.expired(now):
            return False
        if now < self.paused_at:
            return False
        if (type(navigation_sample) not in (int, float)
                or not math.isfinite(navigation_sample)
                or self.last_nav_sample is not None
                and navigation_sample <= self.last_nav_sample):
            return False
        if self.good_since is None:
            self.good_since = now
        self.last_nav_sample = navigation_sample
        self.good_count += 1
        return self.good_count >= self.good_samples and now-self.good_since >= self.good_span_s

    def resumed(self):
        self.paused_at = None
        self.reset_good()


class RetainedGlobalComputation:
    """Keep one goal/worker through soft input loss; gate only its next commit.

    This class does not extend a deadline. Native computation and a completed
    result have separate bounded timeouts in the global planner. It retains no
    sensor packet, local spline, path publication, or execution authority.
    """
    def __init__(self, good_samples=3, good_span_s=.2):
        if (type(good_samples) is not int or not 2 <= good_samples <= 10
                or type(good_span_s) not in (int, float)
                or not math.isfinite(good_span_s) or not .1 <= good_span_s <= 1.):
            raise ValueError('invalid_global_commit_gate')
        self.good_samples = good_samples
        self.good_span_s = float(good_span_s)
        self.intent = None
        self.paused_at = None
        self.good_count = 0
        self.good_since = None
        self.last_nav_sample = None

    def install(self, intent):
        if not isinstance(intent, GoalIntent):
            raise ValueError('immutable_goal_intent_required')
        self.clear()
        self.intent = intent

    def clear(self):
        self.intent = None
        self.paused_at = None
        self.reset_good()

    def reset_good(self):
        self.good_count = 0
        self.good_since = None
        self.last_nav_sample = None

    def pause(self, now):
        if self.intent is None or not math.isfinite(now):
            return False
        first = self.paused_at is None
        if first:
            self.paused_at = now
        self.reset_good()
        return first

    def observe_good(self, now, navigation_sample):
        if (self.intent is None or self.paused_at is None or not math.isfinite(now)
                or now < self.paused_at or type(navigation_sample) not in (int, float)
                or not math.isfinite(navigation_sample)
                or self.last_nav_sample is not None
                and navigation_sample <= self.last_nav_sample):
            return False
        if self.good_since is None:
            self.good_since = now
        self.last_nav_sample = navigation_sample
        self.good_count += 1
        return self.good_count >= self.good_samples and now-self.good_since >= self.good_span_s

    def resumed(self):
        self.paused_at = None
        self.reset_good()
