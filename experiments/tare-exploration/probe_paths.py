import json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import TwistStamped,PointStamped
rclpy.init();n=Node('p9_path_probe');data={}
def cb(t,m):
 h={'frame':m.header.frame_id,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9}
 if isinstance(m,Path):h['poses']=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in m.poses]
 elif isinstance(m,Odometry):h['pos']=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
 elif isinstance(m,PointStamped):h['pos']=[m.point.x,m.point.y,m.point.z]
 else:h['cmd']=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
 data[t]=h
for t,k in [('/p9/exploration/global_path',Path),('/p9/exploration/global_path_full',Path),('/path',Path),('/p9/exploration/tare_path',Path),('/state_estimation',Odometry),('/p9/exploration/tare_waypoint',PointStamped),('/p9/navigation/raw_cmd_vel',TwistStamped)]:n.create_subscription(k,t,lambda m,t=t:cb(t,m),qos_profile_sensor_data)
start=time.monotonic()
while time.monotonic()-start<4:rclpy.spin_once(n,timeout_sec=.1)
print(json.dumps(data,indent=2));n.destroy_node();rclpy.shutdown()
