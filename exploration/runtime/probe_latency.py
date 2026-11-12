import json,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
rclpy.init();n=Node('p9_latency_probe',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
records={}
def cb(topic,msg):
 stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
 age=n.get_clock().now().nanoseconds/1e9-stamp
 records.setdefault(topic,[]).append(age)
for topic,kind in [('/p9/points',PointCloud2),('/pointlio/odom',Odometry),('/pointlio/registered_scan',PointCloud2),('/state_estimation',Odometry),('/registered_scan',PointCloud2),('/terrain_map',PointCloud2)]:
 n.create_subscription(kind,topic,lambda m,t=topic:cb(t,m),qos_profile_sensor_data)
start=time.monotonic()
while time.monotonic()-start<15:rclpy.spin_once(n,timeout_sec=.1)
print(json.dumps({k:{'n':len(v),'first_age':v[0],'last_age':v[-1],'max_age':max(v)} for k,v in records.items()},indent=2))
n.destroy_node();rclpy.shutdown()
