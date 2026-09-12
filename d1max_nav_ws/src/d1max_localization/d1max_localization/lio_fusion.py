"""Pure SE(3) LIO-to-map alignment. No robot control or SDK connection."""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np
from .math_utils import rotate_vector

def rotate_pose_covariance(values, quaternion):
    covariance=np.asarray(values,dtype=float).reshape(6,6)
    if not np.isfinite(covariance).all():raise ValueError('nonfinite covariance')
    rotation=np.column_stack([rotate_vector(quaternion,axis) for axis in ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))])
    transform=np.zeros((6,6));transform[:3,:3]=rotation;transform[3:,3:]=rotation
    return (transform@covariance@transform.T).reshape(36).tolist()
from .math_utils import Pose3, compose, inverse, interpolate_pose, pose_innovation

@dataclass(frozen=True)
class LioLimits:
    local_timeout: float = .3
    correction_timeout: float = .75
    measurement_timeout: float = .5
    max_speed: float = 4.
    max_angular_rate: float = 5.
    tracking_xy: float = .6
    tracking_z: float = .35
    tracking_rotation: float = .7
    acquisition_xy: float = 3.
    acquisition_z: float = 1.
    acquisition_rotation: float = 1.8
    recovery_xy: float = 1.5
    recovery_z: float = .5
    recovery_rotation: float = .7
    recovery_after: float = 1.
    recovery_interval: float = 3.
    recovery_horizon: float = 15.
    recovery_attempts: int = 3

class LioMapState:
    def __init__(self, limits=LioLimits()):
        self.limits=limits
        for key,value in vars(limits).items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid limit '+key)
        if limits.recovery_after<limits.correction_timeout:raise ValueError('recovery must follow validity timeout')
        if not isinstance(limits.recovery_attempts,int) or limits.recovery_attempts>10:raise ValueError('recovery attempts must be an integer in 1..10')
        self.local=deque(maxlen=400)
        self.anchor=None
        self.seed_time=None
        self.last_correction=None
        self.last_raw=0.
        self.accepted=0
        self.rejected=0
        self.recovery=False
        self.recovery_count=0
        self.last_recovery=-math.inf
        self.fault=None
        self.local_epoch=0
        self.stream_valid=True

    def accept_epoch(self,epoch,valid):
        """Ordered local-sample channel; never carry a map anchor across an LIO reset."""
        if type(epoch) is not int or not 1<=epoch<2**64 or type(valid) is not bool or epoch<self.local_epoch:
            return False
        if epoch>self.local_epoch:
            self.local.clear();self.anchor=None;self.seed_time=None
            self.last_correction=None;self.last_raw=0.
            self.recovery=False;self.recovery_count=0;self.last_recovery=-math.inf
            self.fault=None;self.accepted=0;self.rejected=0;self.local_epoch=epoch
        self.stream_valid=valid
        return True

    @staticmethod
    def current_verification(value,seed):
        return (seed is not None and value.get('schema')==1 and value.get('seed_ns')==seed
                and isinstance(value.get('confirmations'),int) and value['confirmations']>=3)

    @staticmethod
    def valid_time(t,now,age):
        return math.isfinite(t) and t>0 and -.1<=now-t<=age

    def push_local(self,t,pose,now):
        if not self.valid_time(t,now,self.limits.local_timeout):return False
        if self.local:
            previous,old=self.local[-1]
            if t<=previous:
                if t<previous-.05:self.fault='local_clock_reset'
                return False
            innovation=pose_innovation(pose,old);dt=t-previous
            if (math.hypot(innovation.translation_xy,innovation.translation_z)>.03+self.limits.max_speed*dt
                    or innovation.rotation>.03+self.limits.max_angular_rate*dt):
                self.fault='local_pose_jump'
                return False
        if self.fault:return False
        self.local.append((t,pose))
        while len(self.local)>2 and t-self.local[1][0]>4.:self.local.popleft()
        return True

    def local_at(self,t):
        return interpolate_pose([(round(s*1e9),p) for s,p in self.local],round(t*1e9),250000000)

    def local_fresh(self,now):
        return self.stream_valid and bool(self.local) and not self.fault and self.valid_time(self.local[-1][0],now,self.limits.local_timeout)

    def seed(self,map_pose,now):
        if not self.local_fresh(now):raise ValueError('局部 LIO 尚未就绪')
        t,local=self.local[-1]
        self.anchor=compose(map_pose,inverse(local))
        self.seed_time=now
        self.last_correction=None;self.last_raw=0.
        self.recovery=False;self.recovery_count=0;self.last_recovery=-math.inf
        return map_pose

    def prediction(self,t):
        local=self.local_at(t)
        return compose(self.anchor,local) if self.anchor is not None and local is not None else None

    def accept(self,t,map_pose,now):
        def reject():
            self.rejected+=1
            return False
        if self.seed_time is None or t<self.seed_time or t<=self.last_raw:return reject()
        if not self.local_fresh(now) or not self.valid_time(t,now,self.limits.measurement_timeout):return reject()
        self.last_raw=t
        expected=self.prediction(t);local=self.local_at(t)
        if expected is None or local is None:return reject()
        d=pose_innovation(map_pose,expected);l=self.limits
        if self.recovery:xy,z,r=l.recovery_xy,l.recovery_z,l.recovery_rotation
        elif self.last_correction is None:xy,z,r=l.acquisition_xy,l.acquisition_z,l.acquisition_rotation
        else:xy,z,r=l.tracking_xy,l.tracking_z,l.tracking_rotation
        if d.translation_xy>xy or d.translation_z>z or d.rotation>r:return reject()
        self.anchor=compose(map_pose,inverse(local))
        self.last_correction=t;self.accepted+=1
        self.recovery=False;self.recovery_count=0
        return True

    def tracking(self,now):
        return (self.local_fresh(now) and not self.recovery and self.last_correction is not None
                and self.valid_time(self.last_correction,now,self.limits.correction_timeout))

    def begin_recovery(self,now):
        l=self.limits
        if (self.last_correction is None or not self.local_fresh(now) or
                not l.recovery_after<now-self.last_correction<=l.recovery_horizon or
                now-self.last_recovery<l.recovery_interval or self.recovery_count>=l.recovery_attempts):
            return None
        target=self.prediction(self.local[-1][0])
        if target is None:return None
        self.recovery=True;self.recovery_count+=1;self.last_recovery=now
        self.seed_time=now
        return target
