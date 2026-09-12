"""Pure, testable sensor admission and body-to-tracking velocity conversion."""
from math import cos, sin, isfinite

def finite_vector(values, limit):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError('expected three sensor components')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not isfinite(v) or abs(v)>limit for v in values):
        raise ValueError('invalid sensor value')
    return tuple(float(v) for v in values)

def valid_stamp(stamp, now, max_age):
    return isfinite(stamp) and stamp>0 and -.10 <= now-stamp <= max_age

def tracking_velocity(body_velocity, yaw, offset):
    vx,vy,wz=body_velocity
    # v_sensor = v_body + omega cross lever-arm, then rotate to tracking axes.
    vx,vy=vx-wz*offset[1],vy+wz*offset[0]
    c,s=cos(yaw),sin(yaw)
    return c*vx-s*vy,s*vx+c*vy,wz

class GyroBias:
    def __init__(self, samples=100, max_rate=.04, stationary_speed=.025, deadband=.01):
        self.samples=samples; self.max_rate=max_rate; self.stationary_speed=stationary_speed; self.deadband=deadband
        self.reset()
    def reset(self):
        self.count=0; self.total=0.; self.bias=None
    def update(self, gyro, speed):
        if self.bias is None:
            if speed is None or speed>self.stationary_speed or abs(gyro)>self.max_rate:
                self.count=0;self.total=0.;return None
            self.total+=gyro;self.count+=1
            if self.count<self.samples:return None
            self.bias=self.total/self.count
        corrected=gyro-self.bias
        if speed is not None and speed<=self.stationary_speed and abs(corrected)<=self.deadband:return 0.
        return corrected

def health_state(sensors_ready, calibrated, seeded, aligned, correction_age, local_fresh, global_fresh):
    if not sensors_ready:return 'waiting_sensors'
    if not calibrated:return 'calibrating'
    if not seeded:return 'waiting_initial_pose'
    if not aligned:return 'acquiring'
    if correction_age is None or correction_age>1. or not local_fresh or not global_fresh:return 'degraded'
    return 'tracking'


def tracking_twist(linear, angular, yaw, offset):
    """Full v_sensor = R * (v_body + omega_body cross r_body_sensor).

    R is the configured fixed tracking<-SDK rotation (currently yaw-only CAD).
    No startup gravity leveling, no SDK world pose, no duplicate gyro fusion.
    """
    vx, vy, vz = linear
    wx, wy, wz = angular
    rx, ry, rz = offset
    velocity = (vx + wy*rz-wz*ry, vy + wz*rx-wx*rz, vz + wx*ry-wy*rx)
    c, s = cos(yaw), sin(yaw)
    def rotate(v):
        return (c*v[0]-s*v[1], s*v[0]+c*v[1], v[2])
    return rotate(velocity), rotate(angular)


class GyroBias3:
    """One stationary bias for all three physical IMU axes and all consumers."""
    def __init__(self, samples=100, max_rate=.04, stationary_speed=.025, deadband=.01):
        self.samples = samples
        self.max_rate = max_rate
        self.stationary_speed = stationary_speed
        self.deadband = deadband
        self.reset()

    def reset(self):
        self.count = 0
        self.total = [0., 0., 0.]
        self.bias = None

    def update(self, gyro, speed):
        norm = sum(v*v for v in gyro)**.5
        if self.bias is None:
            if speed is None or speed > self.stationary_speed or norm > self.max_rate:
                self.count = 0
                self.total = [0., 0., 0.]
                return None
            self.total = [a+b for a, b in zip(self.total, gyro)]
            self.count += 1
            if self.count < self.samples:
                return None
            self.bias = tuple(v/self.count for v in self.total)
        corrected = tuple(g-b for g, b in zip(gyro, self.bias))
        if speed is not None and speed <= self.stationary_speed and sum(v*v for v in corrected)**.5 <= self.deadband:
            return (0., 0., 0.)
        return corrected
