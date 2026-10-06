"""Bounded functional liveness, separate from sensor/trajectory validity.

Only advancing producer timestamps renew liveness. This never authorizes
motion, repairs stale sensor data, restarts nodes, or owns a navigation task.
"""
import math


def create_functional_heartbeat_timer(node, callback):
    from rclpy.clock import Clock, ClockType
    # Executor liveness uses wall/steady time. ROS time may advance slowly or
    # pause, without stopping the function that reports its own availability.
    return node.create_timer(.2, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


class FunctionalHealth:
    def __init__(self, session_id, components, *, started, timeout_s=1.5, startup_s=30.):
        if (not session_id or not components or len(set(components))!=len(components)
                or not 0<timeout_s<=5 or not timeout_s<startup_s<=60):
            raise ValueError('invalid_component_health_contract')
        self.session,self.components=session_id,tuple(components)
        self.started,self.timeout,self.startup=started,timeout_s,startup_s
        self.last={};self.sequence=0;self.latched=''

    def observe(self, component, value, *, wall_now, monotonic):
        if component not in self.components or not isinstance(value,dict) or value.get('session_id')!=self.session:
            return False
        stamp=value.get('callback_wall_time',value.get('received_at_unix',value.get('wall_time',value.get('stamp'))))
        if (type(stamp) not in (int,float) or not math.isfinite(stamp)
                or not -.1<=wall_now-stamp<=self.timeout):
            return False
        previous=self.last.get(component)
        if previous is not None and stamp<=previous[0]:
            return False
        self.last[component]=(stamp,monotonic)
        return True

    def snapshot(self, *, wall_now, monotonic):
        missing=[name for name in self.components if name not in self.last]
        stalled=[name for name,(_stamp,receipt) in self.last.items() if monotonic-receipt>self.timeout]
        if not self.latched:
            if stalled:
                self.latched='component_executor_stalled:'+','.join(sorted(stalled))
            elif missing and monotonic-self.started>self.startup:
                self.latched='component_startup_timeout:'+','.join(missing)
        self.sequence+=1
        return dict(schema=1,session_id=self.session,sequence=self.sequence,
            ready=not missing and not stalled and not self.latched,fatal=bool(self.latched),
            reason=self.latched or ('waiting_components:'+','.join(missing) if missing else ''),
            observed_at_unix=wall_now,components=[dict(name=name,seen=name in self.last,
                callback_age_s=None if name not in self.last else monotonic-self.last[name][1]) for name in self.components],
            motion_authorized=False,sensor_validity_authorized=False)


class SupervisorHealth:
    """A file from a dead monitor cannot continuously renew core supervision."""
    def __init__(self, session_id, *, started, timeout_s=1.5, startup_s=30.):
        self.session,self.started,self.timeout,self.startup=session_id,started,timeout_s,startup_s
        self.sequence=0;self.received=None

    def check(self, value, *, monotonic):
        if isinstance(value,dict) and value.get('schema')==1 and value.get('session_id')==self.session:
            seq=value.get('sequence')
            if type(seq) is int and seq>self.sequence:
                self.sequence,self.received=seq,monotonic
                if value.get('fatal') is True:
                    return value.get('reason') or 'component_health_fault'
        if self.received is None:
            return 'component_health_monitor_startup_timeout' if monotonic-self.started>self.startup else ''
        return 'component_health_monitor_stalled' if monotonic-self.received>self.timeout else ''
