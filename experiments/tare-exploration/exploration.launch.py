"""TARE replaces FAR; P9 localization, terrain and local control stay unchanged."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from go2_p9.configuration import load
from go2_p9_navigation.configuration import load_navigation, share

def setup(context):
 out=Path(LaunchConfiguration('output').perform(context));base=Path('/output/integration')
 profile=str(base/'p9-exploration-sim.yaml')
 core,_,_,_=load(profile,'sim')
 params,_=load_navigation(share()/'config/navigation.yaml','sim',True,core_config=core)
 params['local_planner']['pathFolder']=str(Path(get_package_share_directory('local_planner'))/'paths')
 # TARE may select a nearby viewpoint (< 0.3 m) before the next path segment.
 # FAR's 0.3 m terminal-goal tolerance deadlocked on a valid 0.255 m local path.
 # This override belongs only to exploration; obstacle checks remain unchanged.
 params['path_follower']['stopDisThre']=0.10
 localization=ExecuteProcess(cmd=['p9','sim','--config',profile,'--output',str(out/'localization')],output='screen')
 nodes=[localization]
 isolate=[('/joy','/p9/exploration/joy_unused'),('/speed','/p9/exploration/speed_unused'),('/stop','/p9/exploration/stop_unused')]
 for pkg,exe,key,remap in [('terrain_analysis','terrainAnalysis','terrain_analysis',[]),('terrain_analysis_ext','terrainAnalysisExt','terrain_analysis_ext',[]),('local_planner','localPlanner','local_planner',[]),('local_planner','pathFollower','path_follower',[('/cmd_vel','/p9/navigation/raw_cmd_vel')])]:
  nodes.append(Node(package=pkg,executable=exe,name='p9_'+key,parameters=[params[key]],remappings=isolate+remap,output='screen'))
 nodes.append(Node(package='tare_planner',executable='tare_planner_node',name='tare_planner_node',parameters=[str(base/'tare-small-scene.yaml')],remappings=[('/local_path','/p9/exploration/tare_path'),('/global_path','/p9/exploration/global_path'),('/global_path_full','/p9/exploration/global_path_full')],output='screen'))
 nodes.append(ExecuteProcess(cmd=['python3',str(base/'guard_node.py'),'--ros-args','-p','use_sim_time:=true','-p','sim_auto_resume_inputs:=true','-p','mode:=sim','-p','sim_command_output:=true','-p','output:='+str(out)],output='screen'))
 nodes.append(ExecuteProcess(cmd=['python3',str(base/'boundary.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'))
 for n in list(nodes):nodes.append(RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='Required exploration process exited'))])))
 return nodes

def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('output'),OpaqueFunction(function=setup)])
