"""Bounded asynchronous join of unchanged commands and native sweep evidence."""
from collections import OrderedDict
from copy import deepcopy

from .atomic_projection_state import stamp_ns
from .execution_safety import Decision


class CommandAdmission:
    def __init__(self,core):
        self.core=core
        self.pending=OrderedDict()
        self.recent=None

    def expire(self,now_ns):
        for key,message in tuple(self.pending.items()):
            if (now_ns-stamp_ns(message.source_stamp)>100_000_000
                    or now_ns>=stamp_ns(message.valid_until)):
                del self.pending[key]
        if self.recent is not None and (now_ns-stamp_ns(self.recent.source_stamp)>100_000_000
                or now_ns>=stamp_ns(self.recent.valid_until)):
            self.recent=None

    def demand(self,message,now_ns):
        self.expire(now_ns)
        decision=self.core.admit(message,now_ns)
        if decision.reason in ('waiting_command_sweep','waiting_native_curve_proof'):
            key=self.core.motion_key(message)
            self.pending[key]=deepcopy(message)
            self.pending.move_to_end(key)
            while len(self.pending)>8:self.pending.popitem(last=False)
            # Keep the previously verified command within its original lease.
            # Merely receiving a newer unverified candidate is not a stop event.
            return decision
        if decision.allowed:
            self.recent=deepcopy(message) if decision.output.safety_checked else None
            for key,old in tuple(self.pending.items()):
                if old.sequence<=message.sequence:del self.pending[key]
            return decision
        self.pending.pop(self.core.motion_key(message),None)
        self.recent=None
        stop=self.core.stop_for_rejection(message,now_ns,decision.reason)
        return Decision(False,decision.reason,stop.output if stop.allowed else None)

    def proof(self,message,now_ns):
        self.expire(now_ns)
        if not self.core.on_motion_validation(message):
            return Decision(False,'old_or_foreign_command_proof')
        demand=self.pending.get(self.core.motion_key(message,proof=True))
        if demand is None:
            if self.recent is not None and self.core.motion_key(self.recent)==self.core.motion_key(message,proof=True):
                return self.core.admit_evidence_upgrade(self.recent,now_ns)
            return Decision(False,'command_proof_without_pending_demand')
        return self.demand(demand,now_ns)

    def validation(self,message,now_ns):
        """Resolve only queued commands with their exact native proof delivery.

        This callback never generates a new command or renews a deadline.
        An event-driven newer sweep can upgrade the exact current command's
        evidence, without changing any control or source payload. Newest
        eligible command wins; ordinary
        positive evidence cannot cause older commands to supersede it.
        """
        self.expire(now_ns)
        if not self.core.on_validation(message):return ()
        results=[]
        for key,demand in reversed(tuple(self.pending.items())):
            if key not in self.core.motion_proofs:continue
            result=self.demand(demand,now_ns)
            results.append(result)
            if result.allowed:break
        if not results and self.recent is not None:
            result=self.core.admit_evidence_upgrade(self.recent,now_ns)
            if result.allowed:results.append(result)
        return tuple(results)
