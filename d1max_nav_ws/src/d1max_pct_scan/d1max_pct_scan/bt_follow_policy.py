"""Bounded recovery for FollowRoute; never alters native collision decisions.

The action remains RUNNING through transient gaps. A healthy incumbent wins
over a failed successor, but only the production bridge can prove it healthy.
Budgets belong to an episode, not to each changing display/status string.
"""
from dataclasses import dataclass
import math


FOLLOW_DEFAULTS = dict(follow_pose_wait_s=30., follow_map_wait_s=15.,
    follow_reference_wait_s=5., follow_trajectory_wait_s=20.,
    follow_recovery_episode_s=30., follow_stable_reset_s=.6)


def validate_follow_policy(params):
    for key, default in FOLLOW_DEFAULTS.items():
        value = params.get(key, default)
        upper = 3. if key == 'follow_stable_reset_s' else 120.
        lower = .4 if key == 'follow_stable_reset_s' else 1.
        if (type(value) not in (int, float) or not math.isfinite(value)
                or not lower <= value <= upper):
            raise ValueError('invalid_follow_policy:'+key)
    if params.get('follow_stable_reset_s', .6) >= params.get('follow_recovery_episode_s', 30.):
        raise ValueError('invalid_follow_policy:stable_reset_must_fit_episode')


@dataclass(frozen=True)
class FollowStage:
    phase: str
    reason: str
    budget: str = ''
    terminal: bool = False


def classify_follow(*, pose_ready, map_ready, committed, subscriber_ready,
                    same_reference, scan, pending_native_phase=''):
    if not pose_ready:
        return FollowStage('waiting_localization', 'waiting_current_localization', 'pose')
    if not map_ready:
        return FollowStage('waiting_local_map', 'waiting_fresh_local_map', 'map')
    if committed and not same_reference and pending_native_phase:
        # Native receipt means delivery succeeded, NOT collision acceptance.
        # Expose the real current-generation wait while the candidate has no
        # validation/owner commit yet. It stays a bounded trajectory wait.
        if pending_native_phase in ('reference_rejected_frame','reference_rejected_geometry',
                                    'failed_reference_geometry','emergency_stop'):
            return FollowStage('local_contract_failed',pending_native_phase,terminal=True)
        return FollowStage('recovering_local_trajectory', pending_native_phase, 'trajectory')
    if not committed or not same_reference:
        return FollowStage('waiting_reference', 'waiting_reference_acceptance' if committed
                           else 'waiting_reference_subscriber' if not subscriber_ready
                           else 'delivering_committed_route', 'reference')
    # Candidate failure must not erase an independently revalidated incumbent.
    if scan.get('spline_visual_valid') is True:
        return FollowStage('following', 'following_validated_local_trajectory')
    phase = str(scan.get('local_debug_phase') or 'waiting_native_trajectory')[:120]
    if phase in ('reference_rejected_frame', 'reference_rejected_geometry',
                 'failed_reference_geometry', 'emergency_stop'):
        return FollowStage('local_contract_failed', phase, terminal=True)
    return FollowStage('recovering_local_trajectory', phase, 'trajectory')


class FollowRecovery:
    """Monotonic accumulated waits and a bounded episode; no deadline renewal.

    Clearing an episode requires sustained following and three distinct bridge
    reports. A single valid packet or switching wait reason cannot reset it.
    This is not a progress watchdog: stationary preview can follow indefinitely.
    """
    def __init__(self, params):
        validate_follow_policy(params)
        self.params = {k: params.get(k, v) for k, v in FOLLOW_DEFAULTS.items()}
        self.since = self.last = self.good_since = None
        self.previous = self.last_evidence = None
        self.good_samples = 0
        self.waits = {}

    def observe(self, stage, now, *, evidence_stamp=0.):
        if stage.terminal:
            return 'follow_contract_failure:'+stage.reason
        if self.last is not None and self.previous:
            self.waits[self.previous] = self.waits.get(self.previous, 0.)+max(0., now-self.last)
        self.last, self.previous = now, stage.budget
        if stage.budget:
            if self.since is None:
                self.since = now
            self.good_since = None
            self.good_samples = 0
        elif self.since is not None:
            if self.good_since is None:
                self.good_since = now
            if evidence_stamp > 0 and (self.last_evidence is None or evidence_stamp > self.last_evidence):
                self.good_samples += 1
                self.last_evidence = evidence_stamp
            if now-self.good_since >= self.params['follow_stable_reset_s'] and self.good_samples >= 3:
                self.since = self.good_since = None
                self.waits.clear()
                return ''
        for kind, elapsed in self.waits.items():
            if elapsed >= self.params['follow_'+kind+'_wait_s']:
                return 'follow_'+kind+'_timeout:'+stage.reason
        if self.since is not None and now-self.since >= self.params['follow_recovery_episode_s']:
            return 'follow_recovery_episode_timeout:'+stage.reason
        return ''
