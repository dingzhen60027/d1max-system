"""Position display only. Never navigation readiness, a seed or motion input."""
import math
from copy import deepcopy


class PoseContinuityView:
    def __init__(self, session, map_version):
        self.session, self.map_version = session, map_version
        self.sequence = 0
        self.sample = None
        self.received = -math.inf
        self.identity_blocked = False

    def withdraw(self):
        # A hard identity/fault event supersedes even a late model packet.
        # Keep the sequence fence; only a new accepted REAL typed pose reopens
        # this display cache. Neither operation grants navigation readiness.
        self.sample = None
        self.identity_blocked = True

    def bind_real_pose(self):
        self.sample = None
        self.identity_blocked = False

    def observe(self, value, *, identity, now_ns, monotonic):
        if (not isinstance(value,dict) or type(value.get('schema')) is not int or value.get('schema')!=1
                or value.get('session_id')!=self.session or value.get('map_version_id')!=self.map_version
                or type(value.get('sequence')) is not int or not self.sequence<value['sequence']<2**64
                or value.get('motion_usable') is not False or type(value.get('available')) is not bool
                or type(value.get('estimate_stamp_ns')) is not int
                or not 0<=now_ns-value['estimate_stamp_ns']<=100_000_000):
            return False
        if not value['available']:
            self.sequence=value['sequence'];self.sample=None;self.received=monotonic
            return True
        expected=(self.session,value.get('localization_epoch'),value.get('localization_seed_id'))
        t,p,i,u=(value.get(k) for k in ('estimate_stamp_ns','posterior_stamp_ns','imu_stamp_ns','valid_until_ns'))
        if (self.identity_blocked or identity!=expected or type(expected[1]) is not int or not 0<expected[1]<2**64
                or not isinstance(expected[2],str) or not 0<len(expected[2])<=128
                or value.get('frame_id')!='d1max_loc_map' or value.get('body_frame')!='d1max_loc_base_link'
                or value.get('quality')!='imu_gap_coasting_pose_only'
                or any(type(v) is not int or v<=0 for v in (t,p,i,u))
                or not p<=t<=now_ns<=u or not i<=t
                or u>min(p+750_000_000,i+500_000_000)):
            return False
        for name,count in (('position',3),('orientation',4),('pose_covariance',36)):
            a=value.get(name)
            if not isinstance(a,list) or len(a)!=count or any(type(v) not in (int,float) or not math.isfinite(v) for v in a):
                return False
        if abs(sum(v*v for v in value['orientation'])-1.)>.001:return False
        if max(abs(v) for v in value['position'])>10000:return False
        if any(value['pose_covariance'][7*j]<0 for j in range(6)):return False
        self.sequence=value['sequence'];self.sample=deepcopy(value);self.received=monotonic
        return True

    def lease(self, *, identity, now_ns, monotonic):
        s=self.sample
        if not s or identity!=(self.session,s['localization_epoch'],s['localization_seed_id']):return 0.
        if now_ns<s['estimate_stamp_ns'] or not 0<=monotonic-self.received<=.1:return 0.
        return max(0.,min((s['valid_until_ns']-now_ns)*1e-9,
            .08-(now_ns-s['estimate_stamp_ns'])*1e-9,.1-(monotonic-self.received)))
