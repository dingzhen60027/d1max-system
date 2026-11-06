"""Bounded functional liveness, separate from sensor/trajectory validity.

Only advancing producer timestamps renew liveness. This never authorizes
motion, repairs stale sensor data, restarts nodes, or owns a navigation task.
"""
import math


class FunctionalHealth:
    def __init__(self, session_id, components, *, started, timeout_s=1.5, startup_s=30.,
                 stable_duration_s=.6, minimum_new_samples=3):
        values=(started,timeout_s,startup_s,stable_duration_s)
        if (not session_id or not components or len(set(components))!=len(components)
                or any(type(value) not in (int,float) or not math.isfinite(value) for value in values)
                or not 0<timeout_s<=5 or not timeout_s<startup_s<=60
                or not 0<stable_duration_s<=5
                or type(minimum_new_samples) is not int or not 3<=minimum_new_samples<=512):
            raise ValueError('invalid_component_health_contract')
        self.session,self.components=session_id,tuple(components)
        self.started,self.timeout,self.startup=started,timeout_s,startup_s
        self.stable_duration,self.minimum_new_samples=stable_duration_s,minimum_new_samples
        self.last={};self.sequence=0;self.latched=''
        self.recovering={}

    def observe(self, component, value, *, wall_now, monotonic):
        if component not in self.components or not isinstance(value,dict) or value.get('session_id')!=self.session:
            return False
        stamp=value.get('callback_wall_time',value.get('received_at_unix',value.get('wall_time',value.get('stamp'))))
        if (type(stamp) not in (int,float) or not math.isfinite(stamp)
                or type(wall_now) not in (int,float) or not math.isfinite(wall_now)
                or type(monotonic) not in (int,float) or not math.isfinite(monotonic)
                or not -.1<=wall_now-stamp<=self.timeout):
            return False
        previous=self.last.get(component)
        if previous is not None and (stamp<=previous[0] or monotonic<previous[1]):
            return False
        if previous is not None and monotonic-previous[1]>self.timeout:
            # Even when the monitor timer was delayed, a real producer gap
            # requires recovery. A snapshot is not the source of this evidence.
            self.recovering[component]=(None,0)
        self.last[component]=(stamp,monotonic)
        if component in self.recovering:
            first,count=self.recovering[component]
            first=monotonic if first is None else first
            count+=1
            if count>=self.minimum_new_samples and monotonic-first>=self.stable_duration:
                del self.recovering[component]
            else:
                self.recovering[component]=(first,count)
        return True

    def snapshot(self, *, wall_now, monotonic):
        missing=[name for name in self.components if name not in self.last]
        stalled=[name for name,(_stamp,receipt) in self.last.items() if monotonic-receipt>self.timeout]
        for name in stalled:
            self.recovering[name]=(None,0)
        if not self.latched and missing and monotonic-self.started>self.startup:
            # A component that never initialized still consumes the bounded
            # startup budget. Runtime gaps belong to the BT recovery episode,
            # not a second monitor-owned terminal navigation policy.
            self.latched='component_startup_timeout:'+','.join(missing)
        recovering=[name for name in self.components if name in self.recovering and name not in stalled]
        reason=self.latched
        if not reason:
            if stalled:reason='component_executor_stalled:'+','.join(sorted(stalled))
            elif missing:reason='waiting_components:'+','.join(missing)
            elif recovering:reason='component_recovering:'+','.join(recovering)
        self.sequence+=1
        return dict(schema=1,session_id=self.session,sequence=self.sequence,
            ready=not missing and not stalled and not recovering and not self.latched,fatal=bool(self.latched),
            reason=reason,
            observed_at_unix=wall_now,components=[dict(name=name,seen=name in self.last,
                callback_age_s=None if name not in self.last else monotonic-self.last[name][1],
                producer_source_time=None if name not in self.last else self.last[name][0],
                recovering=name in self.recovering,
                recovery_new_samples=self.recovering.get(name,(None,0))[1]) for name in self.components],
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
