"""ROS transport for a fail-closed planner command gate (not a hardware driver)."""
from dataclasses import fields
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from .guard_core import GuardConfig, NavigationGuard


def stamp_ns(stamp):return int(stamp.sec)*1000000000+int(stamp.nanosec)
def xyz(v):return (v.x,v.y,v.z)


class GuardNode(Node):
    def __init__(self):
        super().__init__('p9_navigation_guard')
        self.declare_parameter('mode','replay');self.declare_parameter('sim_command_output',False)
        self.declare_parameter('output','')
        mode=self.get_parameter('mode').value;bridge=self.get_parameter('sim_command_output').value
        if bridge and mode!='sim':raise ValueError('Simulator bridge requires mode=sim')
        values={}
        for f in fields(GuardConfig):
            self.declare_parameter(f.name,float(f.default));values[f.name]=self.get_parameter(f.name).value
        self.core=NavigationGuard(GuardConfig(**values))
        self.vehicle_tf=TransformBroadcaster(self)
        self.output=Path(self.get_parameter('output').value) if self.get_parameter('output').value else None
        self.pub=self.create_publisher(Twist,'/p9/navigation/cmd_vel',10)
        self.sim_pub=self.create_publisher(Twist,'/cmd_vel',10) if bridge else None
        self.goal_pub=self.create_publisher(PointStamped,'/goal_point',10)
        self.result_pub=self.create_publisher(String,'/p9/navigation/goal_result',10)
        self.status_pub=self.create_publisher(String,'/p9/navigation/status',10)
        self.create_subscription(Odometry,'/state_estimation',self.odom,qos_profile_sensor_data)
        for name,topic in [('scan','/registered_scan'),('terrain','/terrain_map'),('terrain_ext','/terrain_map_ext')]:
            self.create_subscription(PointCloud2,topic,lambda m,n=name:self.cloud(n,m),qos_profile_sensor_data)
        self.create_subscription(Bool,'/pointlio/tracking_valid',lambda m:self.core.tracking(m.data,time.monotonic()),qos_profile_sensor_data)
        self.create_subscription(Bool,'/p9/navigation/far_ready',lambda m:self.core.graph_ready(m.data,time.monotonic()),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float32,'/runtime',lambda m:self.core.graph(m.data,time.monotonic()),10)
        self.create_subscription(PointStamped,'/way_point',lambda m:self.core.waypoint(stamp_ns(m.header.stamp),m.header.frame_id,xyz(m.point),time.monotonic()),10)
        self.create_subscription(PathMsg,'/path',self.path,10)
        self.create_subscription(TwistStamped,'/p9/navigation/raw_cmd_vel',self.command,10)
        self.create_subscription(PointStamped,'/p9/navigation/goal_request',self.goal,10)
        self.create_subscription(PoseStamped,'/p9/navigation/goal_pose',self.pose_goal,10)
        self.create_subscription(Bool,'/far_reach_goal_status',lambda m:self.core.reached(m.data),10)
        self.create_service(Trigger,'/p9/navigation/stop',self.stop)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(.05,self.tick,clock=self.steady)
        self.status_timer=self.create_timer(.5,self.status,clock=self.steady)
        self.refresh_clock()

    def refresh_clock(self):self.core.clock(self.get_clock().now().nanoseconds,time.monotonic())
    def odom(self,m):
        self.refresh_clock();p=m.pose.pose
        valid=self.core.odometry(stamp_ns(m.header.stamp),m.header.frame_id,m.child_frame_id,xyz(p.position),(*xyz(p.orientation),p.orientation.w),time.monotonic())
        if valid:
            # localPlanner and pathFollower use yaw-only XY coordinates.
            # A full-RPY base_link identity alias would draw a tilted path.
            q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            tf=TransformStamped();tf.header=m.header;tf.child_frame_id='vehicle'
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=xyz(p.position)
            tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
            self.vehicle_tf.sendTransform(tf)
    def cloud(self,name,m):
        self.refresh_clock();self.core.cloud(name,stamp_ns(m.header.stamp),m.header.frame_id,int(m.width)*int(m.height),time.monotonic())
    def path(self,m):
        self.refresh_clock();self.core.path(stamp_ns(m.header.stamp),m.header.frame_id,[xyz(p.pose.position) for p in m.poses],time.monotonic())
    def command(self,m):
        self.refresh_clock();v=m.twist
        valid=all(math.isfinite(x) for x in (*xyz(v.linear),*xyz(v.angular))) and v.linear.z==0 and v.angular.x==0 and v.angular.y==0
        self.core.command(stamp_ns(m.header.stamp),m.header.frame_id,(v.linear.x,v.linear.y,v.angular.z) if valid else (),time.monotonic())
    def goal(self,m):
        self.refresh_clock()
        if not self.goal_pub.get_subscription_count():ok,reason=False,'FAR goal subscription is not connected'
        else:ok,reason=self.core.request_goal(m.header.frame_id,(m.point.x,m.point.y),time.monotonic())
        if ok:
            out=PointStamped();out.header.frame_id='camera_init';out.header.stamp=self.get_clock().now().to_msg()
            out.point.x,out.point.y,out.point.z=self.core.goal;self.goal_pub.publish(out)
        else:self.get_logger().warning(reason)
        reply=String();reply.data=json.dumps(dict(request_id=stamp_ns(m.header.stamp),accepted=ok,reason=reason));self.result_pub.publish(reply)
    def pose_goal(self,m):
        p=PointStamped();p.header=m.header;p.point=m.pose.position;self.goal(p)
    def stop(self,request,response):
        self.core.stop();self.publish((0.,0.,0.));response.success=True;response.message='Goal cleared and isolated command stopped';return response
    def publish(self,v):
        m=Twist();m.linear.x,m.linear.y,m.angular.z=v;self.pub.publish(m)
        if self.sim_pub:self.sim_pub.publish(m)
    def tick(self):
        self.refresh_clock();self.publish(self.core.output(time.monotonic()))
    def status(self):
        data=self.core.status(time.monotonic());data['sim_command_output']=self.sim_pub is not None
        data['far_goal_subscribers']=self.goal_pub.get_subscription_count()
        data['ready']=data['ready'] and data['far_goal_subscribers']>0
        message=String();message.data=json.dumps(data);self.status_pub.publish(message)
        if self.output:
            self.output.mkdir(parents=True,exist_ok=True);p=self.output/'navigation-status.json';tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(p)


def main():
    rclpy.init();node=None
    try:node=GuardNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:
            if rclpy.ok():node.core.stop('Shutdown');node.publish((0.,0.,0.));node.status()
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
