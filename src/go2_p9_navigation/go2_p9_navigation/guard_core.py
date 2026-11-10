"""ROS-independent goal and velocity gating. All watchdogs use monotonic time.

The only actuator-capable bridge in this package is explicitly selected Gazebo
/cmd_vel output. Otherwise commands remain on an isolated planning topic.
"""
from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class GuardConfig:
    max_forward_m_s: float = .25
    max_lateral_m_s: float = .2
    max_yaw_rad_s: float = .5
    sensor_timeout_sec: float = 1.
    tracking_timeout_sec: float = 1.
    graph_timeout_sec: float = 2.
    path_timeout_sec: float = 1.
    command_timeout_sec: float = .5
    clock_timeout_sec: float = .5
    future_tolerance_sec: float = .05
    goal_gate_sec: float = .2
    max_goal_distance_m: float = 30.
    goal_reached_distance_m: float = .4

    def __post_init__(self):
        for f in fields(self):
            v=getattr(self,f.name)
            if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<=0:
                raise ValueError(f.name+' must be finite and positive')
        if self.max_forward_m_s>1. or self.max_lateral_m_s>1. or self.max_yaw_rad_s>2.:
            raise ValueError('Command limits exceed this integration\'s explicit maximum')


class NavigationGuard:
    world_frame='camera_init'
    body_frame='base_link'
    path_frame='vehicle'
    def __init__(self, config=None):
        self.cfg=config or GuardConfig();self.times={};self.stamps={};self.clock_ns=None
        self.clock_change_wall=None;self.clock_reset_latched=False;self.pose=None
        self.goal=None;self.goal_gate_ns=None;self.first_waypoint_ns=None;self.path_ns=None
        self.target=(0.,0.,0.);self.reason='Waiting for localization, terrain and FAR graph'
        self.limits_applied=0;self.far_ready=False

    def clock(self, ns, wall):
        if not isinstance(ns,int) or ns<0 or not math.isfinite(wall):raise ValueError('Invalid clock')
        if self.clock_ns is not None and ns<self.clock_ns:
            self.clock_reset_latched=True;self.stop('Clock moved backwards: restart before accepting goals')
        if ns!=self.clock_ns:self.clock_change_wall=wall
        self.clock_ns=ns

    def fresh_stamp(self, stamp_ns):
        if self.clock_ns is None or not isinstance(stamp_ns,int) or stamp_ns<0:return False
        age=(self.clock_ns-stamp_ns)*1e-9
        return -self.cfg.future_tolerance_sec<=age<self.cfg.sensor_timeout_sec

    def _stamp(self, name, stamp_ns, wall):
        previous=self.stamps.get(name,-1)
        if not self.fresh_stamp(stamp_ns) or stamp_ns<previous:
            self.times.pop(name,None);return False
        # Re-publication of a cached frame is harmless, but is not new sensor data.
        # Do not extend its watchdog. In particular this cannot keep a dead
        # sensor alive by repeatedly forwarding an old terrain/path message.
        if stamp_ns==previous:return name in self.times
        self.stamps[name]=stamp_ns;self.times[name]=wall;return True

    def odometry(self, stamp_ns, frame, child, position, quaternion, wall):
        valid=(frame==self.world_frame and child==self.body_frame and len(position)==3 and len(quaternion)==4
               and all(math.isfinite(x) for x in (*position,*quaternion))
               and abs(sum(x*x for x in quaternion)-1.)<.002)
        if valid and self._stamp('odom',stamp_ns,wall):self.pose=tuple(position);return True
        self.times.pop('odom',None);self.stop('Invalid or stale body odometry');return False

    def cloud(self, name, stamp_ns, frame, points, wall):
        if name not in ('scan','terrain','terrain_ext'):raise ValueError('Unknown cloud stream')
        valid=frame==self.world_frame and isinstance(points,int) and points>0
        if valid and self._stamp(name,stamp_ns,wall):return True
        self.times.pop(name,None);return False

    def tracking(self, valid, wall):
        if valid:self.times['tracking']=wall
        else:self.times.pop('tracking',None);self.stop('Point-LIO tracking is invalid')

    def graph(self, runtime, wall):
        if math.isfinite(runtime) and runtime>=0:self.times['graph']=wall
        else:self.times.pop('graph',None)

    def graph_ready(self, valid, wall):
        self.far_ready=bool(valid)
        if valid:self.times['graph_ready']=wall
        else:
            self.times.pop('graph_ready',None)
            self.stop('FAR graph is not ready; submit a new goal after recovery')

    def localized(self, wall):
        required=[('odom',self.cfg.sensor_timeout_sec),('scan',self.cfg.sensor_timeout_sec),('tracking',self.cfg.tracking_timeout_sec)]
        return (not self.clock_reset_latched and self.clock_change_wall is not None
                and 0<=wall-self.clock_change_wall<self.cfg.clock_timeout_sec
                and all(k in self.times and 0<=wall-self.times[k]<timeout for k,timeout in required))

    def planning_ready(self, wall):
        return (self.localized(wall) and self.pose is not None and self.far_ready and
                all(k in self.times and 0<=wall-self.times[k]<timeout for k,timeout in
                    [('terrain',self.cfg.sensor_timeout_sec),('terrain_ext',self.cfg.sensor_timeout_sec),('graph',self.cfg.graph_timeout_sec),('graph_ready',self.cfg.graph_timeout_sec)]))

    def request_goal(self, frame, xy, wall):
        if frame!=self.world_frame or len(xy)!=2 or not all(math.isfinite(x) for x in xy):
            return False,'Goal must be finite XY in '+self.world_frame
        if not self.planning_ready(wall):return False,'Localization, terrain or FAR graph is not ready'
        if math.hypot(xy[0]-self.pose[0],xy[1]-self.pose[1])>self.cfg.max_goal_distance_m:
            return False,'Goal exceeds configured maximum distance'
        self.goal=(float(xy[0]),float(xy[1]),self.pose[2]);self.target=(0.,0.,0.)
        self.goal_gate_ns=self.clock_ns+round(self.cfg.goal_gate_sec*1e9)
        self.first_waypoint_ns=self.path_ns=None;self.times.pop('path',None);self.times.pop('command',None)
        self.reason='Goal accepted; waiting for a fresh waypoint, path and command'
        return True,self.reason

    def waypoint(self, stamp_ns, frame, xyz, wall):
        if (self.goal is not None and frame==self.world_frame and len(xyz)==3 and all(math.isfinite(x) for x in xyz)
                and stamp_ns>=self.goal_gate_ns and self.fresh_stamp(stamp_ns)):
            if self.first_waypoint_ns is None:self.first_waypoint_ns=stamp_ns
            return True
        return False

    def path(self, stamp_ns, frame, xyz, wall):
        valid=(self.goal is not None and self.first_waypoint_ns is not None
               and stamp_ns>=self.first_waypoint_ns and frame==self.path_frame and bool(xyz)
               and all(len(p)==3 and all(math.isfinite(x) for x in p) for p in xyz))
        if valid and self._stamp('path',stamp_ns,wall):self.path_ns=stamp_ns;return True
        self.times.pop('path',None);self.times.pop('command',None);self.target=(0.,0.,0.);return False

    def command(self, stamp_ns, frame, velocity, wall):
        valid=(self.path_ns is not None and frame==self.path_frame and stamp_ns>=self.path_ns
               and self.fresh_stamp(stamp_ns) and len(velocity)==3 and all(math.isfinite(x) for x in velocity))
        # pathFollower can issue repeated commands with the same odometry stamp.
        if valid:self.times['command']=wall;self.target=tuple(velocity);return True
        self.times.pop('command',None);self.target=(0.,0.,0.);return False

    def reached(self, valid):
        if valid and self.goal is not None and self.pose is not None and math.hypot(self.goal[0]-self.pose[0],self.goal[1]-self.pose[1])<self.cfg.goal_reached_distance_m:
            self.stop('Goal reached');return True
        return False

    def stop(self, reason='Stopped by request'):
        self.goal=None;self.goal_gate_ns=None;self.first_waypoint_ns=self.path_ns=None
        self.times.pop('path',None);self.times.pop('command',None);self.target=(0.,0.,0.);self.reason=reason

    def output(self, wall):
        if not self.localized(wall):
            if self.goal is not None:self.stop('Localization or clock unavailable; submit a new goal after recovery')
            return (0.,0.,0.)
        if self.goal is None or not self.planning_ready(wall):return (0.,0.,0.)
        if any(k not in self.times or wall-self.times[k]>=t for k,t in [('path',self.cfg.path_timeout_sec),('command',self.cfg.command_timeout_sec)]):return (0.,0.,0.)
        result=tuple(max(-lim,min(lim,v)) for v,lim in zip(self.target,(self.cfg.max_forward_m_s,self.cfg.max_lateral_m_s,self.cfg.max_yaw_rad_s)))
        self.limits_applied+=int(result!=self.target);self.reason='Following FAR waypoint and local path'
        return result

    def status(self, wall):
        return dict(ready=self.planning_ready(wall),localized=self.localized(wall),reason=self.reason,goal=self.goal,
                    clock_reset_latched=self.clock_reset_latched,limits_applied=self.limits_applied,far_ready=self.far_ready,
                    stream_age_sec={k:wall-v for k,v in self.times.items()},path_stamp_ns=self.path_ns,
                    command_topic='/p9/navigation/cmd_vel',hardware_control=False)
