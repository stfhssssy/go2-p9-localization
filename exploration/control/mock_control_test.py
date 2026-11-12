#!/usr/bin/env python3
"""Loopback-only ROS transport test; never connects to a robot network interface."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import time

# Set before ROS import, also inherited by the child. No host NIC is enabled.
os.environ['ROS_DOMAIN_ID']='101'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces></General></Domain></CycloneDDS>'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


def main():
    rclpy.init(); node=Node('p9_control_mock_unitree')
    q=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    replies=node.create_publisher(Response,'/api/sport/response',q)
    state_pub=node.create_publisher(SportModeState,'/lf/sportmodestate',q)
    guard_pub=node.create_publisher(String,'/p9/exploration/status',q)
    cmd_pub=node.create_publisher(Twist,'/p9/exploration/cmd_vel',q)
    requests=[]; transcript=[]; guard_on=True; command_on=True; sequence=0; active=True
    def request(msg):
        requests.append((time.monotonic(),int(msg.header.identity.api_id),json.loads(msg.parameter)))
        if msg.header.identity.api_id==1003:
            response=Response();response.header.identity=msg.header.identity;response.header.status.code=0;response.data='{}';replies.publish(response)
    sub=node.create_subscription(Request,'/api/sport/request',request,q)
    def spawn():
        master,slave=pty.openpty()
        child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('p9explore_control.py')),'--enable-motion'],stdin=slave,stdout=slave,stderr=slave,env=os.environ.copy())
        os.close(slave)
        return master,child
    master,child=spawn()
    def drain():
        if master is not None and select.select([master],[],[],0)[0]:
            try:
                text=os.read(master,65536).decode(errors='replace');transcript.append(text)
            except OSError: pass
    def run(seconds,hold=False):
        nonlocal sequence
        deadline=time.monotonic()+seconds;next_pub=0
        while time.monotonic()<deadline:
            now=time.monotonic()
            if now>=next_pub:
                next_pub=now+.05
                wall=time.time_ns();state=SportModeState();state.stamp.sec=wall//1000000000;state.stamp.nanosec=wall%1000000000;state.body_height=.33;state_pub.publish(state)
                if guard_on:
                    sequence+=1;guard=String();guard.data=json.dumps(dict(mode='live',state='EXPLORING',active=active,node_instance_id='loopback-mock',status_sequence=sequence,created_stamp_ns=wall));guard_pub.publish(guard)
                if command_on:
                    cmd=Twist();cmd.linear.x=.3;cmd.linear.y=.2;cmd.angular.z=.5;cmd_pub.publish(cmd)
                if hold: os.write(master,b'h')
            rclpy.spin_once(node,timeout_sec=.01);drain()
    def moves(): return len([row for row in requests if row[1]==1008])
    def stops(): return len([row for row in requests if row[1]==1003])
    try:
        run(2.5);assert stops()>0 and moves()==0,'startup must stop without Move'
        os.write(master,b'g');run(.7);assert moves()==0,'arm alone cannot move'
        run(1.0,hold=True);assert moves()>3,'armed deadman must forward guarded commands'
        for _,api,payload in requests:
            if api==1008: assert payload=={'x':.15,'y':.1,'z':.25},payload
            else: assert api==1003 and payload=={},(api,payload)
        guard_on=False;stop_before=stops();run(1.8,hold=True);assert stops()>stop_before,'stale guard must StopMove'
        after_fault=moves();guard_on=True;run(.7,hold=True);assert moves()==after_fault,'fresh data alone cannot re-arm'
        os.write(master,b'g');run(.7);run(.6,hold=True);assert moves()>after_fault,'explicit re-arm restores attended output'
        command_on=False;stop_before=stops();run(.6,hold=True);assert stops()>stop_before,'stale command must stop'
        stopped_moves=moves();command_on=True;run(.5,hold=True);assert moves()==stopped_moves
        os.write(master,b'g');run(.7);run(.6,hold=True);stop_before=stops();run(.8);assert stops()>stop_before,'released deadman must stop'
        stop_before=stops();os.write(master,b'q');run(.5);child.wait(timeout=3);assert stops()>stop_before,'shutdown must StopMove'
        assert child.returncode==0,child.returncode
        disconnect_cases=[]
        for disconnect in ('SIGHUP','PTY-master-close'):
            if master is not None: os.close(master)
            master,child=spawn()
            move_before=moves();run(2.0)
            assert moves()==move_before,'new session must not auto-arm'
            os.write(master,b'g');run(.7);run(.6,hold=True)
            assert moves()>move_before+2,'disconnect regression requires actual active Move output first'
            stop_before=stops();cutoff=time.monotonic()
            if disconnect=='SIGHUP': child.send_signal(signal.SIGHUP)
            else:
                os.close(master);master=None
            # Continue fresh inputs and replies: only terminal/signal failure should stop.
            run(1.0)
            child.wait(timeout=3)
            assert stops()>stop_before,disconnect+' must still publish shutdown StopMove'
            assert any(t>=cutoff and api==1003 for t,api,_ in requests),disconnect
            assert child.returncode==0,(disconnect,child.returncode)
            stop_index=max(i for i,row in enumerate(requests) if row[1]==1003)
            assert not any(row[1]==1008 for row in requests[stop_index+1:]),disconnect+' moved after stop'
            disconnect_cases.append(dict(trigger=disconnect,passed=True,returncode=child.returncode))
        result=dict(passed=True,test='loopback ROS Unitree mock',domain=101,interface='lo',robot_access=False,requests=len(requests),move_count=moves(),stop_count=stops(),disconnect_cases=disconnect_cases,checks=['startup-stop-no-motion','ack-before-arm','deadman-required','caps','stale-guard-stop','no-auto-rearm','manual-rearm','stale-command-stop','deadman-expiry-stop','shutdown-stop','SIGHUP-stop','PTY-disconnect-stop'])
        print(json.dumps(result,indent=2))
    finally:
        if child.poll() is None:
            child.terminate()
            try: child.wait(timeout=3)
            except subprocess.TimeoutExpired: child.kill();child.wait()
        drain()
        if master is not None: os.close(master)
        node.destroy_node();rclpy.shutdown()
        Path(__file__).with_name('mock-control-transcript.txt').write_text(''.join(transcript))

if __name__=='__main__':main()
