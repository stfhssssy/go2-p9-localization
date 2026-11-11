#!/usr/bin/env python3
"""TARE-only navigation gate. It never starts or calls a robot hardware driver.

TARE /local_path -> /p9/exploration/tare_path
TARE /global_path_full -> /p9/exploration/global_path_full (untrimmed fallback)
TARE /way_point -> /p9/exploration/tare_waypoint
TARE /exploration_finish -> /p9/exploration/finished
FAR must not be launched in this mode. /way_point has one authority: this node.
"""
from dataclasses import fields
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from guard_core import ExplorationGuard, GuardConfig


def stamp_ns(stamp):
    return int(stamp.sec)*1000000000+int(stamp.nanosec)


def xyz(value):
    return (value.x, value.y, value.z)


class ExplorationGuardNode(Node):
    def __init__(self):
        super().__init__('p9_exploration_guard')
        self.declare_parameter('mode', 'preview')
        self.declare_parameter('sim_command_output', False)
        self.declare_parameter('sim_auto_resume_inputs', False)
        self.declare_parameter('output', '')
        mode = self.get_parameter('mode').value
        bridge = self.get_parameter('sim_command_output').value
        auto_resume = self.get_parameter('sim_auto_resume_inputs').value
        if mode not in ('preview', 'sim', 'live', 'replay'):
            raise ValueError('Unsupported mode')
        if bridge and mode != 'sim':
            raise ValueError('Only explicit sim mode may publish Gazebo /cmd_vel')
        if bridge and not self.get_parameter('use_sim_time').value:
            raise ValueError('Gazebo command output requires use_sim_time=true')
        if auto_resume and not (mode == 'sim' and bridge):
            raise ValueError('sim_auto_resume_inputs requires mode=sim and sim_command_output=true')
        params = {}
        for item in fields(GuardConfig):
            self.declare_parameter(item.name, float(item.default))
            params[item.name] = self.get_parameter(item.name).value
        self.core = ExplorationGuard(GuardConfig(**params), auto_resume_inputs=auto_resume,
                                     simulation_mode=mode == 'sim' and bridge)
        self.output_dir = Path(self.get_parameter('output').value) if self.get_parameter('output').value else None
        self.command_pub = self.create_publisher(Twist, '/p9/exploration/cmd_vel', 10)
        self.sim_pub = self.create_publisher(Twist, '/cmd_vel', 10) if bridge else None
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.start_pub = self.create_publisher(Bool, '/start_exploration', 1)
        self.status_pub = self.create_publisher(String, '/p9/exploration/status', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/state_estimation', self.odometry, qos_profile_sensor_data)
        for name, topic in [('scan', '/registered_scan'), ('terrain', '/terrain_map'), ('terrain_ext', '/terrain_map_ext')]:
            self.create_subscription(PointCloud2, topic, lambda msg, key=name: self.cloud(key, msg), qos_profile_sensor_data)
        self.create_subscription(Bool, '/pointlio/tracking_valid', self.tracking, qos_profile_sensor_data)
        self.create_subscription(PointStamped, '/p9/exploration/tare_waypoint', self.tare_waypoint, 10)
        self.create_subscription(PathMsg, '/p9/exploration/tare_path', self.tare_path, 5)
        self.create_subscription(PathMsg, '/p9/exploration/global_path_full', self.tare_global_path, 5)
        self.create_subscription(Bool, '/p9/exploration/finished', lambda msg: self.core.finished(msg.data), 10)
        self.create_subscription(PathMsg, '/path', self.local_path, 10)
        self.create_subscription(TwistStamped, '/p9/navigation/raw_cmd_vel', self.command, 10)
        self.create_service(Trigger, '/p9/exploration/start', self.start)
        self.create_service(Trigger, '/p9/exploration/stop', self.stop)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.tick, clock=self.steady)
        self.create_timer(.5, self.status, clock=self.steady)
        self.refresh_clock()
        self.get_logger().info('TARE guard stopped; explicit /p9/exploration/start required')

    def refresh_clock(self):
        self.core.clock(self.get_clock().now().nanoseconds, time.monotonic())

    def odometry(self, msg):
        self.refresh_clock()
        pose = msg.pose.pose
        valid = self.core.odometry(stamp_ns(msg.header.stamp), msg.header.frame_id, msg.child_frame_id,
                                   xyz(pose.position), (*xyz(pose.orientation), pose.orientation.w), time.monotonic())
        if valid:
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = 'vehicle'
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz(pose.position)
            transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.tf.sendTransform(transform)

    def cloud(self, name, msg):
        self.refresh_clock()
        self.core.cloud(name, stamp_ns(msg.header.stamp), msg.header.frame_id, int(msg.width)*int(msg.height), time.monotonic())

    def tracking(self, msg):
        self.core.tracking(msg.data, time.monotonic())

    def tare_waypoint(self, msg):
        self.refresh_clock()
        self.core.tare_waypoint(stamp_ns(msg.header.stamp), msg.header.frame_id, xyz(msg.point), time.monotonic())
        self.forward_waypoint()

    def tare_path(self, msg):
        self.refresh_clock()
        self.core.tare_path(stamp_ns(msg.header.stamp), msg.header.frame_id,
                            [xyz(p.pose.position) for p in msg.poses], time.monotonic())
        self.forward_waypoint()

    def tare_global_path(self, msg):
        self.refresh_clock()
        self.core.tare_global_path(stamp_ns(msg.header.stamp), msg.header.frame_id,
                                   [xyz(p.pose.position) for p in msg.poses], time.monotonic())
        self.forward_waypoint()

    def forward_waypoint(self):
        result = self.core.take_waypoint(time.monotonic())
        if result is None:
            return
        ns, position = result
        msg = PointStamped()
        msg.header.frame_id = self.core.world_frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
        msg.point.x, msg.point.y, msg.point.z = position
        self.waypoint_pub.publish(msg)

    def local_path(self, msg):
        self.refresh_clock()
        self.core.local_path(stamp_ns(msg.header.stamp), msg.header.frame_id,
                             [xyz(p.pose.position) for p in msg.poses], time.monotonic())

    def command(self, msg):
        self.refresh_clock()
        twist = msg.twist
        valid = all(math.isfinite(x) for x in (*xyz(twist.linear), *xyz(twist.angular)))
        valid = valid and twist.linear.z == 0 and twist.angular.x == 0 and twist.angular.y == 0
        velocity = (twist.linear.x, twist.linear.y, twist.angular.z) if valid else ()
        self.core.command(stamp_ns(msg.header.stamp), msg.header.frame_id, velocity, time.monotonic())

    def start(self, request, response):
        self.refresh_clock()
        if not self.start_pub.get_subscription_count() or not self.waypoint_pub.get_subscription_count():
            response.success = False
            response.message = 'TARE start or localPlanner waypoint subscription is not connected'
            return response
        ok, reason = self.core.start(time.monotonic())
        if ok:
            self.start_pub.publish(Bool(data=True))
        response.success, response.message = ok, reason
        self.status()
        return response

    def stop(self, request, response):
        self.core.stop()
        self.publish((0., 0., 0.))
        response.success = True
        response.message = 'Exploration commands stopped and latched; explicit start required to resume'
        self.status()
        return response

    def publish(self, velocity):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.command_pub.publish(msg)
        if self.sim_pub is not None:
            self.sim_pub.publish(msg)

    def tick(self):
        self.refresh_clock()
        self.publish(self.core.output(time.monotonic()))

    def status(self):
        result = self.core.status(time.monotonic())
        result.update(sim_command_output=self.sim_pub is not None,
                      tare_start_subscribers=self.start_pub.get_subscription_count(),
                      local_waypoint_subscribers=self.waypoint_pub.get_subscription_count())
        self.status_pub.publish(String(data=json.dumps(result, allow_nan=False)))
        if self.output_dir is not None:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            dest = self.output_dir/'exploration-status.json'
            temp = dest.with_suffix('.tmp')
            temp.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
            temp.replace(dest)


def main():
    rclpy.init()
    node = None
    try:
        node = ExplorationGuardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.core.stop('Guard shutdown')
            if rclpy.ok():
                node.publish((0., 0., 0.))
                node.status()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
