"""No ROS/SDK imports: attended high-level motion policy for the optional bridge."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Decision:
    kind: str
    velocity: tuple = (0.0, 0.0, 0.0)
    reason: str = ""


class ControlPolicy:
    def __init__(self, enabled=False, max_forward=.15, max_lateral=.10, max_yaw=.25):
        limits = (max_forward, max_lateral, max_yaw)
        if not all(math.isfinite(v) and 0 < v <= cap for v, cap in zip(limits, (.5, .5, 1.0))):
            raise ValueError("Limits must be finite and positive, linear <=0.5 m/s, yaw <=1 rad/s")
        self.enabled = enabled
        self.limits = limits
        self.state = "DISARMED"
        self.reason = "explicit arm required"
        self.receipts = dict(state=-math.inf, guard=-math.inf, command=-math.inf)
        self.timeouts = dict(state=.5, guard=1.5, command=.35)
        self.robot_stamp = None
        self.guard_instance = None
        self.guard_sequence = -1
        self.guard_active = False
        self.command = (0., 0., 0.)
        self.arm_fence = math.inf
        self.deadman_at = -math.inf
        self.deadman_started = False
        self.stop_due = False
        self.last_now = -math.inf

    def engaged(self):
        return self.state in ("ARMING", "ARMED_WAIT_DEADMAN", "ACTIVE")

    def stop(self, reason="operator stop"):
        self.state = "DISARMED"
        self.reason = reason
        self.deadman_at = -math.inf
        self.deadman_started = False
        self.arm_fence = math.inf
        self.stop_due = self.enabled

    def fault(self, reason):
        self.stop(reason)
        self.state = "FAULT"

    def robot(self, stamp, now, values):
        if not all(math.isfinite(float(x)) for x in values):
            if self.engaged(): self.fault("nonfinite robot telemetry")
            return
        if stamp != self.robot_stamp:
            if self.robot_stamp is not None and stamp < self.robot_stamp and self.engaged():
                self.fault("robot timestamp regressed")
            self.robot_stamp = stamp
            self.receipts['state'] = now

    def guard(self, data, now, wall_ns):
        try:
            instance = data['node_instance_id']
            sequence = data['status_sequence']
            created = data['created_stamp_ns']
            if not isinstance(instance, str) or not instance or type(sequence) is not int or type(created) is not int:
                raise ValueError("invalid guard identity/stamp")
            age = (wall_ns - created) / 1e9
            valid_time = -.1 <= age <= 1.5
            active = data.get('mode') == 'live' and data.get('active') is True and data.get('state') == 'EXPLORING'
            changed = self.guard_instance is not None and instance != self.guard_instance
            if changed and self.engaged(): self.fault("guard process changed; re-arm required")
            if changed: self.guard_sequence = -1
            if not valid_time or sequence <= self.guard_sequence:
                if self.engaged(): self.fault("guard status stale/repeated")
                self.guard_active = False
                return
            self.guard_instance, self.guard_sequence = instance, sequence
            self.guard_active = active
            self.receipts['guard'] = now
            if not active and self.engaged(): self.fault("guard is not actively exploring")
        except (KeyError, ValueError, TypeError, OverflowError, AttributeError):
            self.guard_active = False
            if self.engaged(): self.fault("invalid guard status")

    def velocity(self, values, now):
        if len(values) != 6 or not all(math.isfinite(v) for v in values) or any(abs(values[i]) > 1e-9 for i in (2, 3, 4)):
            if self.engaged(): self.fault("invalid or nonplanar command")
            self.receipts['command'] = -math.inf
            return
        self.command = tuple(max(-limit, min(limit, value)) for value, limit in zip((values[0], values[1], values[5]), self.limits))
        self.receipts['command'] = now

    def ready(self, now):
        if not self.guard_active: return "guard is not actively exploring"
        for source, timeout in self.timeouts.items():
            if not 0 <= now - self.receipts[source] <= timeout: return source + " is stale"
        return ""

    def request_arm(self, now):
        if not self.enabled:
            self.reason = "preview only; --enable-motion required"
            return False
        reason = self.ready(now)
        if reason:
            self.reason = reason
            return False
        self.stop_due = False
        self.state, self.reason = "ARMING", "waiting for acknowledged StopMove"
        self.arm_fence = now
        self.deadman_at = -math.inf
        self.deadman_started = False
        return True

    def stop_ack(self, success):
        if self.state != 'ARMING': return
        if success:
            self.state, self.reason = 'ARMED_WAIT_DEADMAN', 'hold h to permit motion'
        else: self.fault('StopMove was not acknowledged successfully')

    def deadman(self, now):
        if self.state in ('ARMED_WAIT_DEADMAN', 'ACTIVE'):
            self.deadman_at = now
            self.deadman_started = True

    def competing_control(self):
        if self.engaged(): self.fault('another high-level controller sent an action')

    def tick(self, now):
        if now < self.last_now: self.fault('monotonic clock regressed')
        self.last_now = now
        if self.engaged():
            reason = self.ready(now)
            if reason: self.fault(reason)
            elif self.deadman_started and now - self.deadman_at > .6: self.fault('operator deadman expired')
        if self.stop_due:
            self.stop_due = False
            return Decision('stop', reason=self.reason)
        if self.state in ('ARMED_WAIT_DEADMAN', 'ACTIVE') and self.deadman_started:
            if all(stamp > self.arm_fence for stamp in self.receipts.values()):
                self.state, self.reason = 'ACTIVE', 'attended motion enabled'
                return Decision('move', self.command)
        return Decision('idle', reason=self.reason)
