"""New entrypoint: legacy `p9` remains localization-only."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import time
from .configuration import load_navigation, share


def goal(x,y,frame):
    import rclpy
    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('p9_navigation_goal_client');pub=node.create_publisher(PointStamped,'/p9/navigation/goal_request',10)
    request_id=time.time_ns();response=[]
    def receive(message):
        data=json.loads(message.data)
        if data.get('request_id')==request_id:response.append(data)
    sub=node.create_subscription(String,'/p9/navigation/goal_result',receive,10)
    deadline=time.monotonic()+8
    while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    if pub.get_subscription_count():
        m=PointStamped();m.header.frame_id=frame;m.header.stamp.sec=request_id//1000000000;m.header.stamp.nanosec=request_id%1000000000;m.point.x=x;m.point.y=y;pub.publish(m)
        while not response and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node();rclpy.shutdown()
    if not response:raise SystemExit('No goal acknowledgement; verify p9nav and ROS_DOMAIN_ID')
    print(json.dumps(response[0],ensure_ascii=False));return 0 if response[0]['accepted'] else 2


def main():
    p=argparse.ArgumentParser(description='P9 B1 localization + mapping + FAR/local planning; isolated commands by default')
    sub=p.add_subparsers(dest='mode',required=True)
    for mode in ('replay','live','sim'):
        s=sub.add_parser(mode);s.add_argument('--config',required=True);s.add_argument('--navigation-config');s.add_argument('--output',required=True);s.add_argument('--rviz',action='store_true');s.add_argument('--sim-command-output',action='store_true')
        if mode=='replay':s.add_argument('--bag',required=True);s.add_argument('--rate',type=float,default=.5);s.add_argument('--exit-on-finish',action='store_true')
    s=sub.add_parser('goal');s.add_argument('x',type=float);s.add_argument('y',type=float);s.add_argument('--frame',default='camera_init')
    sub.add_parser('stop');a=p.parse_args()
    if a.mode=='stop':os.execvp('ros2',['ros2','service','call','/p9/navigation/stop','std_srvs/srv/Trigger','{}'])
    if a.mode=='goal':
        if not all(math.isfinite(v) for v in (a.x,a.y)):p.error('Goal coordinates must be finite')
        raise SystemExit(goal(a.x,a.y,a.frame))
    from go2_p9.configuration import load
    config=Path(a.config).expanduser().resolve();nav=Path(a.navigation_config or share()/'config/navigation.yaml').expanduser().resolve();output=Path(a.output).expanduser().resolve()
    try:
        core,_,_,_=load(config,a.mode);params,guard=load_navigation(nav,a.mode,a.sim_command_output,core_config=core)
        if a.mode=='replay' and (not Path(a.bag).exists() or not math.isfinite(a.rate) or a.rate<=0):raise ValueError('Replay needs an existing bag and finite positive rate')
        if output.exists() and any(output.iterdir()):raise ValueError('Choose a new empty --output directory')
    except (ValueError,TypeError,KeyError,OSError) as error:p.error(str(error))
    output.mkdir(parents=True,exist_ok=True)
    record=dict(created_utc=datetime.now(timezone.utc).isoformat(),mode=a.mode,sim_command_output=a.sim_command_output,hardware_control=False,core_config=str(config),core_config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),navigation_config=str(nav),navigation_config_sha256=hashlib.sha256(nav.read_bytes()).hexdigest(),parameters=params,guard=asdict(guard),world_frame='camera_init',body_frame='base_link',ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID'))
    (output/'effective-navigation.json').write_text(json.dumps(record,indent=2)+'\n')
    command=['ros2','launch','go2_p9_navigation','navigation.launch.py','mode:='+a.mode,'config:='+str(config),'navigation_config:='+str(nav),'output:='+str(output),'rviz:='+str(a.rviz).lower(),'sim_command_output:='+str(a.sim_command_output).lower()]
    if a.mode=='replay':command+=['bag:='+str(Path(a.bag).resolve()),'rate:='+str(a.rate),'exit_on_finish:='+str(a.exit_on_finish).lower()]
    os.execvp(command[0],command)
