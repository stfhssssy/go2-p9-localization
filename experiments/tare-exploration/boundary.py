#!/usr/bin/env python3
"""Task envelope relative to the initial estimated body pose; no ground-truth input."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
class Boundary(Node):
 def __init__(self):
  super().__init__('p9_exploration_boundary');self.origin=None
  self.pubs=[self.create_publisher(PolygonStamped,t,10) for t in ['/p9/exploration/navigation_boundary','/p9/exploration/coverage_boundary']]
  self.vis=self.create_publisher(Marker,'/p9/exploration/boundary_marker',10)
  self.create_subscription(Odometry,'/state_estimation',self.odom,qos_profile_sensor_data)
  self.create_timer(1.,self.tick)
 def odom(self,m):
  if self.origin is not None or m.header.frame_id!='camera_init':return
  p=m.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  self.origin=(p.position.x,p.position.y,p.position.z,yaw)
 def tick(self):
  if self.origin is None:return
  x,y,z,a=self.origin;c,s=math.cos(a),math.sin(a);msg=PolygonStamped();msg.header.frame_id='camera_init';msg.header.stamp=self.get_clock().now().to_msg()
  for dx,dy in [(-6.,-6.),(6.,-6.),(6.,6.),(-6.,6.)]:msg.polygon.points.append(Point32(x=x+c*dx-s*dy,y=y+s*dx+c*dy,z=z))
  for pub in self.pubs:pub.publish(msg)
  marker=Marker();marker.header=msg.header;marker.ns='task_boundary';marker.id=0;marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD;marker.scale.x=.035;marker.color.r=1.;marker.color.g=.85;marker.color.a=.7;marker.pose.orientation.w=1.
  from geometry_msgs.msg import Point
  for p in [*msg.polygon.points,msg.polygon.points[0]]:marker.points.append(Point(x=float(p.x),y=float(p.y),z=float(p.z)))
  self.vis.publish(marker)
def main():
 rclpy.init();n=Boundary()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
