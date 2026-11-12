"""Independent TARE stream guard; no fixed FAR goal and no hardware driver.

All incoming positions are in camera_init. TARE supplies a rolling lookahead,
not a final destination. Its finish flag stops the mission by default. Return
observation is an explicit simulation-only experiment, never an arrival certificate.
Ground-truth data is never accepted by this API.
"""
from collections import deque
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class GuardConfig:
    max_forward_m_s: float = .20
    max_lateral_m_s: float = .10
    max_yaw_rad_s: float = .35
    sensor_timeout_sec: float = 1.0
    tracking_timeout_sec: float = 1.0
    exploration_timeout_sec: float = 3.0
    startup_timeout_sec: float = 15.0
    path_timeout_sec: float = 1.0
    command_timeout_sec: float = .5
    clock_timeout_sec: float = .5
    future_tolerance_sec: float = .05
    max_waypoint_distance_m: float = 30.0
    min_x: float = -10.0
    max_x: float = 10.0
    min_y: float = -10.0
    max_y: float = 10.0
    home_distance_m: float = .5
    home_stable_duration_sec: float = 2.0
    home_speed_m_s: float = .05
    home_position_span_m: float = .10
    home_sample_gap_sec: float = .5

    def __post_init__(self):
        bounds = {'min_x', 'max_x', 'min_y', 'max_y'}
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(field.name+' must be finite numeric')
            if field.name not in bounds and value <= 0:
                raise ValueError(field.name+' must be positive')
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError('Boundary must have positive XY area')
        if max(self.max_forward_m_s, self.max_lateral_m_s) > 1 or self.max_yaw_rad_s > 2:
            raise ValueError('Velocity limits exceed explicit integration maximum')


def finite(values, length):
    return len(values) == length and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                        and math.isfinite(x) for x in values)


class ExplorationGuard:
    world_frame = 'camera_init'
    body_frame = 'base_link'
    path_frame = 'vehicle'
    planning_streams = ('tare_path', 'tare_global_path', 'tare_waypoint', 'path', 'command')

    def __init__(self, config=None, *, auto_resume_inputs=False, simulation_mode=False,
                 stop_on_finished=True):
        if any(type(v) is not bool for v in (auto_resume_inputs, simulation_mode, stop_on_finished)):
            raise ValueError('Input recovery flags must be bool')
        if auto_resume_inputs and not simulation_mode:
            raise ValueError('Automatic input recovery is allowed only in explicit simulation mode')
        if not stop_on_finished and not simulation_mode:
            raise ValueError('Autonomous return is only available as an explicit simulation experiment')
        self.cfg = config or GuardConfig()
        self.auto_resume_inputs = auto_resume_inputs
        self.stop_on_finished = stop_on_finished
        self.input_pause_wall = self.input_pause_stamp_ns = None
        self.input_fresh_since_wall = None
        self.resume_after_stamp_ns = None
        self.input_pause_count = self.input_resume_count = 0
        self.times, self.stamps = {}, {}
        self.clock_ns = self.clock_change_wall = None
        self.clock_reset_latched = False
        self.pose = self.home = None
        self.motion = deque(maxlen=4000)
        self.active = False
        self.state = 'STOPPED'
        self.reason = 'Explicit exploration start required'
        self.started_wall = self.start_stamp_ns = None
        self.waypoint = self.waypoint_ns = self.forwarded_waypoint_ns = None
        self.plan_valid = False
        self.plan_pose_count = 0
        self.global_plan_pose_count = 0
        self.plan_source = self.first_valid_plan_wall = self.last_valid_plan_wall = None
        self.plan_seen = False
        self.path_ns = None
        self.target = (0., 0., 0.)
        self.finish_received = False
        self.returned_home = False
        self.limits_applied = self.starts = self.waypoints_forwarded = 0

    def inside(self, point):
        return self.cfg.min_x <= point[0] <= self.cfg.max_x and self.cfg.min_y <= point[1] <= self.cfg.max_y

    def clock(self, ns, wall):
        if not isinstance(ns, int) or ns < 0 or not math.isfinite(wall):
            raise ValueError('Invalid clock')
        if self.clock_ns is not None and ns < self.clock_ns:
            self.clock_reset_latched = True
            self.stop('Clock moved backwards; restart the integration')
        if ns != self.clock_ns:
            self.clock_change_wall = wall
        self.clock_ns = ns

    def fresh_stamp(self, ns, timeout=None):
        if self.clock_ns is None or not isinstance(ns, int) or ns < 0:
            return False
        age = (self.clock_ns-ns)*1e-9
        return -self.cfg.future_tolerance_sec <= age < (timeout or self.cfg.sensor_timeout_sec)

    def fresh_wall(self, name, wall, timeout):
        return name in self.times and 0 <= wall-self.times[name] < timeout

    def remember(self, name, ns, wall, timeout=None):
        previous = self.stamps.get(name, -1)
        if not self.fresh_stamp(ns, timeout) or ns < previous:
            self.times.pop(name, None)
            return False
        if ns == previous:
            return name in self.times  # cached publications never extend freshness
        self.times[name], self.stamps[name] = wall, ns
        return True

    def odometry(self, ns, frame, child, position, quaternion, wall):
        valid = (frame == self.world_frame and child == self.body_frame
                 and finite(position, 3) and finite(quaternion, 4)
                 and abs(sum(x*x for x in quaternion)-1) < .002 and self.inside(position))
        previous = self.stamps.get('odom')
        if not valid or not self.remember('odom', ns, wall):
            self.times.pop('odom', None)
            if self.active:
                self.stop('Invalid, stale or out-of-bound body odometry')
            return False
        self.pose = tuple(position)
        if ns != previous:
            self.motion.append((ns, self.pose))
            cutoff = ns-round((self.cfg.home_stable_duration_sec+2)*1e9)
            while self.motion and self.motion[0][0] < cutoff:
                self.motion.popleft()
        return True

    def cloud(self, name, ns, frame, points, wall):
        if name not in ('scan', 'terrain', 'terrain_ext'):
            raise ValueError('Unknown cloud stream')
        valid = frame == self.world_frame and isinstance(points, int) and points > 0
        if valid and self.remember(name, ns, wall):
            return True
        self.times.pop(name, None)
        return False

    def tracking(self, valid, wall):
        if valid:
            self.times['tracking'] = wall
        else:
            self.times.pop('tracking', None)
            if self.active:
                self.stop('Point-LIO tracking invalid; explicit restart required')

    def input_ready(self, wall):
        return (not self.clock_reset_latched and self.pose is not None
                and self.clock_change_wall is not None
                and 0 <= wall-self.clock_change_wall < self.cfg.clock_timeout_sec
                and all(self.fresh_wall(name, wall, timeout) for name, timeout in
                        [('odom', self.cfg.sensor_timeout_sec), ('scan', self.cfg.sensor_timeout_sec),
                         ('terrain', self.cfg.sensor_timeout_sec), ('terrain_ext', self.cfg.sensor_timeout_sec),
                         ('tracking', self.cfg.tracking_timeout_sec)]))

    def start(self, wall):
        if self.active:
            return False, 'Exploration is already active'
        if self.returned_home:
            return False, 'Return completed; restart TARE and guard for a new mission'
        if self.finish_received and self.stop_on_finished:
            return False, 'Exploration finished; restart TARE and guard for a new mission'
        if not self.input_ready(wall):
            return False, 'Fresh localization, registered scan, terrain and tracking required'
        if self.home is None:
            self.home = self.pose  # keep original TARE home across pause / resume
        self.active, self.state = True, 'WAITING_PLAN'
        self.reason = 'Started; waiting for a new valid TARE path and lookahead'
        self.started_wall, self.start_stamp_ns = wall, self.clock_ns
        self.waypoint = self.waypoint_ns = self.forwarded_waypoint_ns = None
        self.plan_valid = self.plan_seen = False
        self.plan_pose_count = 0
        self.global_plan_pose_count = 0
        self.plan_source = self.first_valid_plan_wall = self.last_valid_plan_wall = None
        self.path_ns = None
        self.target = (0., 0., 0.)
        self.input_pause_wall = self.input_pause_stamp_ns = None
        self.input_fresh_since_wall = self.resume_after_stamp_ns = None
        for name in self.planning_streams:
            self.times.pop(name, None)
            self.stamps.pop(name, None)
        self.starts += 1
        return True, self.reason

    def tare_path(self, ns, frame, points, wall):
        return self._tare_plan('tare_path', ns, frame, points, wall)

    def tare_global_path(self, ns, frame, points, wall):
        """Accept upstream global_path_full, the route used by GetLookAheadPoint.

        Upstream global_path is trimmed for display and is not this input.
        A finish flag never substitutes for a real, fresh multi-point route.
        """
        return self._tare_plan('tare_global_path', ns, frame, points, wall)

    def _select_plan(self, wall):
        self.plan_source = None
        for name, count in (('tare_path', self.plan_pose_count),
                            ('tare_global_path', self.global_plan_pose_count)):
            if (count >= 2 and self.fresh_wall(name, wall, self.cfg.exploration_timeout_sec)
                    and self.fresh_stamp(self.stamps.get(name), self.cfg.exploration_timeout_sec)):
                self.plan_source = name
                break
        self.plan_valid = self.plan_source is not None
        return self.plan_valid

    def _tare_plan(self, name, ns, frame, points, wall):
        if (not self.active or self.input_pause_wall is not None
                or ns < self.start_stamp_ns):
            return False
        valid = (frame == self.world_frame and all(finite(p, 3) and self.inside(p) for p in points))
        if not valid or not self.remember(name, ns, wall, self.cfg.exploration_timeout_sec):
            self.plan_valid = False
            self.plan_source = None
            self.target = (0., 0., 0.)
            self.stop('Invalid, stale or out-of-bound TARE path: '+name)
            return False
        if name == 'tare_path':
            self.plan_pose_count = len(points)
        else:
            self.global_plan_pose_count = len(points)
        usable = len(points) >= 2
        if usable:
            # A cached publication cannot extend the valid-route watchdog.
            observed_wall = self.times[name]
            if not self.plan_seen:
                self.first_valid_plan_wall = observed_wall
            self.plan_seen = True
            self.last_valid_plan_wall = max(self.last_valid_plan_wall or observed_wall, observed_wall)
        if not self._select_plan(wall):
            self.target = (0., 0., 0.)
            self.times.pop('command', None)
            self.reason = 'TARE has no usable local or global path; commands held at zero'
        return usable

    def tare_waypoint(self, ns, frame, position, wall):
        if (not self.active or self.input_pause_wall is not None
                or ns < self.start_stamp_ns):
            return False
        valid = (frame == self.world_frame and finite(position, 3) and self.inside(position)
                 and self.pose is not None
                 and math.dist(position[:2], self.pose[:2]) <= self.cfg.max_waypoint_distance_m)
        if not valid or not self.remember('tare_waypoint', ns, wall, self.cfg.exploration_timeout_sec):
            self.stop('Invalid, stale or out-of-bound TARE lookahead')
            return False
        self.waypoint, self.waypoint_ns = tuple(position), ns
        return True

    def planning_ready(self, wall):
        return (self.active and self.input_pause_wall is None
                and self.input_ready(wall) and self._select_plan(wall) and self.waypoint is not None
                and self.fresh_wall('tare_waypoint', wall, self.cfg.exploration_timeout_sec)
                and self.fresh_stamp(self.waypoint_ns, self.cfg.exploration_timeout_sec))

    def take_waypoint(self, wall):
        """Return each valid planner lookahead once, preserving its source stamp."""
        if not self.planning_ready(wall) or self.waypoint_ns == self.forwarded_waypoint_ns:
            return None
        self.forwarded_waypoint_ns = self.waypoint_ns
        self.path_ns = None
        self.times.pop('path', None)
        self.times.pop('command', None)
        self.target = (0., 0., 0.)
        self.waypoints_forwarded += 1
        return self.waypoint_ns, self.waypoint

    def local_path(self, ns, frame, points, wall):
        valid = (self.active and self.input_pause_wall is None
                 and self.forwarded_waypoint_ns is not None
                 and (self.resume_after_stamp_ns is None or ns > self.resume_after_stamp_ns)
                 and ns >= self.forwarded_waypoint_ns and frame == self.path_frame and bool(points)
                 and all(finite(p, 3) for p in points))
        if valid and self.remember('path', ns, wall):
            self.path_ns = ns
            return True
        self.path_ns = None
        self.times.pop('path', None)
        self.times.pop('command', None)
        self.target = (0., 0., 0.)
        return False

    def command(self, ns, frame, velocity, wall):
        valid = (self.active and self.input_pause_wall is None
                 and (self.resume_after_stamp_ns is None or ns > self.resume_after_stamp_ns)
                 and self.path_ns is not None and ns >= self.path_ns
                 and frame == self.path_frame and self.fresh_stamp(ns) and finite(velocity, 3))
        if valid:
            self.times['command'], self.target = wall, tuple(velocity)
            return True
        self.times.pop('command', None)
        self.target = (0., 0., 0.)
        return False

    def finished(self, valid):
        if self.active and valid:
            self.finish_received = True
            if self.stop_on_finished:
                self.stop('TARE reports exploration finished; return disabled, explicit new mission required')
                self.state = 'EXPLORATION_FINISHED'
                return
            if self.input_pause_wall is None:
                self.state = 'RETURNING_HOME'
                self.reason = 'TARE reports exploration complete; observing return to estimated home'

    def home_motion(self):
        result = dict(distance_m=None, stable=False, samples=0, span_sec=0., speed_p95_m_s=None)
        if self.home is None or self.pose is None:
            return result
        result['distance_m'] = math.dist(self.pose[:2], self.home[:2])
        if len(self.motion) < 4:
            return result
        end = self.motion[-1][0]
        cutoff = end-round(self.cfg.home_stable_duration_sec*1e9)
        values = [v for v in self.motion if v[0] >= cutoff]
        if len(values) < 4:
            return result
        gaps = [(b[0]-a[0])*1e-9 for a, b in zip(values, values[1:])]
        speeds = [math.dist(b[1][:2], a[1][:2])/dt for a, b, dt in zip(values, values[1:], gaps) if dt > 0]
        span = (values[-1][0]-values[0][0])*1e-9
        speeds.sort()
        p95 = speeds[min(len(speeds)-1, math.ceil(.95*len(speeds))-1)] if speeds else float('inf')
        max_offset = max(math.dist(v[1][:2], values[0][1][:2]) for v in values)
        result.update(samples=len(values), span_sec=span, speed_p95_m_s=p95)
        result['stable'] = (span >= self.cfg.home_stable_duration_sec-.1
                            and max(gaps) <= self.cfg.home_sample_gap_sec
                            and p95 <= self.cfg.home_speed_m_s
                            and max_offset <= self.cfg.home_position_span_m)
        return result

    def pause_inputs(self, wall):
        """Simulation-only zero-output pause; no plans survive a sensor gap."""
        if self.input_pause_wall is None:
            self.input_pause_wall, self.input_pause_stamp_ns = wall, self.clock_ns
            self.resume_after_stamp_ns = self.clock_ns
            self.input_pause_count += 1
            self.waypoint = self.waypoint_ns = self.forwarded_waypoint_ns = None
            self.plan_valid = self.plan_seen = False
            self.plan_pose_count = 0
            self.global_plan_pose_count = 0
            self.plan_source = self.first_valid_plan_wall = self.last_valid_plan_wall = None
            self.path_ns = None
            self.target = (0., 0., 0.)
            for name in self.planning_streams:
                self.times.pop(name, None)
                self.stamps.pop(name, None)
        self.input_fresh_since_wall = None
        self.state = 'PAUSED_INPUTS'
        self.reason = 'Simulation inputs interrupted; zero output while awaiting stable fresh inputs'

    def recover_inputs(self, wall):
        """Return true only after a fresh-input dwell; motion still needs a new plan."""
        if wall-self.input_pause_wall >= 5.0:
            self.stop('Simulation input interruption exceeded 5 seconds; explicit restart required')
            return False
        if not self.input_ready(wall):
            self.input_fresh_since_wall = None
            return False
        if self.input_fresh_since_wall is None:
            self.input_fresh_since_wall = wall
        if wall-self.input_fresh_since_wall < .5:
            return False
        # Fence at recovery, not pause onset: messages produced during the fresh
        # dwell are still old authority. All four planning/control stages must
        # be generated after this recovery clock before movement can resume.
        self.resume_after_stamp_ns = self.clock_ns
        self.start_stamp_ns = self.resume_after_stamp_ns+1
        self.waypoint = self.waypoint_ns = self.forwarded_waypoint_ns = None
        self.plan_valid = self.plan_seen = False
        self.plan_pose_count = 0
        self.global_plan_pose_count = 0
        self.plan_source = self.first_valid_plan_wall = self.last_valid_plan_wall = None
        self.path_ns = None
        self.target = (0., 0., 0.)
        for name in self.planning_streams:
            self.times.pop(name, None)
            self.stamps.pop(name, None)
        self.started_wall = wall
        self.input_pause_wall = self.input_pause_stamp_ns = None
        self.input_fresh_since_wall = None
        self.input_resume_count += 1
        self.state = 'WAITING_PLAN'
        self.reason = 'Simulation inputs recovered; waiting for new TARE path, lookahead and local command'
        return True

    def stop(self, reason='Explicit stop; subsequent planner output cannot re-arm'):
        self.active = False
        self.input_pause_wall = self.input_pause_stamp_ns = None
        self.input_fresh_since_wall = None
        self.state = 'STOPPED'
        self.reason = reason
        self.target = (0., 0., 0.)
        self.path_ns = None
        self.times.pop('command', None)

    def output(self, wall):
        zero = (0., 0., 0.)
        if not self.active:
            return zero
        if self.input_pause_wall is not None:
            self.recover_inputs(wall)
            return zero
        if not self.input_ready(wall):
            if self.auto_resume_inputs:
                self.pause_inputs(wall)
            else:
                self.stop('Localization, terrain or clock unavailable; explicit restart required')
            return zero
        home = self.home_motion()
        if self.finish_received and home['distance_m'] <= self.cfg.home_distance_m and home['stable']:
            self.stop('TARE complete and stopped near estimated home; coverage accuracy is not certified')
            self.returned_home, self.state = True, 'RETURNED_HOME'
            return zero
        if not self.planning_ready(wall):
            # Local and global visualization messages are separate callbacks.
            # An empty/single local path is legal during global fallback: hold
            # zero while waiting for its route, without treating mission age as
            # an immediate timeout. Fresh empty paths do not renew authority.
            route_expired = (self.plan_seen and not self._select_plan(wall)
                             and wall-self.last_valid_plan_wall >= self.cfg.exploration_timeout_sec)
            waypoint_expired = (self.plan_seen
                                and not self.fresh_wall('tare_waypoint', wall, self.cfg.exploration_timeout_sec)
                                and wall-self.times.get('tare_waypoint', self.first_valid_plan_wall)
                                >= self.cfg.exploration_timeout_sec)
            if (route_expired or waypoint_expired
                    or (not self.plan_seen and wall-self.started_wall >= self.cfg.startup_timeout_sec)):
                self.stop('TARE path/lookahead unavailable; explicit restart required')
            return zero
        if any(not self.fresh_wall(name, wall, limit) for name, limit in
               [('path', self.cfg.path_timeout_sec), ('command', self.cfg.command_timeout_sec)]):
            self.reason = 'Waiting for current local path and velocity'
            return zero
        values = tuple(max(-limit, min(limit, value)) for value, limit in
                       zip(self.target, (self.cfg.max_forward_m_s, self.cfg.max_lateral_m_s, self.cfg.max_yaw_rad_s)))
        self.limits_applied += int(values != self.target)
        self.state = 'RETURNING_HOME' if self.finish_received else 'EXPLORING'
        self.reason = 'Following TARE lookahead through localPlanner'
        return values

    def status(self, wall):
        return dict(state=self.state, active=self.active, reason=self.reason,
                    stop_on_finished=self.stop_on_finished,
                    sim_auto_resume_inputs=self.auto_resume_inputs,
                    input_pause_count=self.input_pause_count, input_resume_count=self.input_resume_count,
                    input_pause_wall=self.input_pause_wall, input_pause_stamp_ns=self.input_pause_stamp_ns,
                    resume_after_stamp_ns=self.resume_after_stamp_ns,
                    input_fresh_since_wall=self.input_fresh_since_wall,
                    input_ready=self.input_ready(wall), planning_ready=self.planning_ready(wall),
                    clock_reset_latched=self.clock_reset_latched, home=self.home, pose=self.pose,
                    boundary=dict(min_x=self.cfg.min_x, max_x=self.cfg.max_x, min_y=self.cfg.min_y, max_y=self.cfg.max_y),
                    tare_finished=self.finish_received, returned_home=self.returned_home, home_observation=self.home_motion(),
                    tare_path_poses=self.plan_pose_count, tare_global_path_poses=self.global_plan_pose_count,
                    selected_tare_path=self.plan_source, first_valid_plan_wall=self.first_valid_plan_wall,
                    last_valid_plan_wall=self.last_valid_plan_wall,
                    lookahead=self.waypoint,
                    waypoint_stamp_ns=self.waypoint_ns, forwarded_waypoint_stamp_ns=self.forwarded_waypoint_ns,
                    local_path_stamp_ns=self.path_ns, stream_age_sec={k: wall-v for k, v in self.times.items()},
                    starts=self.starts, waypoints_forwarded=self.waypoints_forwarded,
                    limits_applied=self.limits_applied, hardware_control=False,
                    ground_truth_role='not an input', command_topic='/p9/exploration/cmd_vel')
