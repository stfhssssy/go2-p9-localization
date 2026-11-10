"""Explicit planner entry, with the unchanged P9 CLI as localization child."""
from dataclasses import asdict
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from go2_p9.configuration import load
from go2_p9_navigation.configuration import load_navigation, share


def setup(context):
    value=lambda n:LaunchConfiguration(n).perform(context)
    mode=value('mode');output=Path(value('output')).resolve();profile=Path(value('config')).resolve()
    bridge=value('sim_command_output')=='true'
    core,_,_,_=load(profile,mode)
    parameters,guard=load_navigation(value('navigation_config'),mode,bridge,core_config=core)
    parameters['local_planner']['pathFolder']=str(Path(get_package_share_directory('local_planner'))/'paths')
    command=['ros2','run','go2_p9','p9',mode,'--config',str(profile),'--output',str(output/'localization')]
    if mode=='replay':
        command+=['--bag',value('bag'),'--rate',value('rate')]
        if value('exit_on_finish')=='true':command+=['--exit-on-finish']
    localization=ExecuteProcess(cmd=command,output='screen')
    nodes=[localization]
    isolate=[('/joy','/p9/navigation/joy_unused'),('/speed','/p9/navigation/speed_unused'),('/stop','/p9/navigation/stop_unused')]
    specs=[('terrain_analysis','terrainAnalysis','terrain_analysis',[]),
           ('terrain_analysis_ext','terrainAnalysisExt','terrain_analysis_ext',[]),
           ('far_planner','far_planner','far_planner',[('/odom_world','/state_estimation'),('/terrain_cloud','/terrain_map_ext'),('/scan_cloud','/registered_scan'),('/terrain_local_cloud','/terrain_map')]),
           ('local_planner','localPlanner','local_planner',[]),
           ('local_planner','pathFollower','path_follower',[('/cmd_vel','/p9/navigation/raw_cmd_vel')])]
    essential=[localization]
    for package,executable,key,remaps in specs:
        n=Node(package=package,executable=executable,name='p9_'+key,parameters=[parameters[key]],remappings=isolate+remaps,output='screen')
        nodes.append(n);essential.append(n)
    n=Node(package='go2_p9_navigation',executable='p9_navigation_guard',name='p9_navigation_guard',parameters=[dict(asdict(guard),use_sim_time=mode!='live',mode=mode,sim_command_output=bridge,output=str(output))],output='screen')
    nodes.append(n);essential.append(n)
    for n in essential:
        nodes.append(RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='A required P9/navigation process exited'))])))
    if value('rviz')=='true':nodes.append(Node(package='rviz2',executable='rviz2',name='p9_navigation_rviz',arguments=['-d',str(share()/'config/navigation.rviz')],parameters=[{'use_sim_time':mode!='live'}]))
    return nodes


def generate_launch_description():
    args=[DeclareLaunchArgument('mode',default_value='replay'),DeclareLaunchArgument('config'),DeclareLaunchArgument('output'),
          DeclareLaunchArgument('navigation_config',default_value=str(share()/'config/navigation.yaml'))]
    args += [DeclareLaunchArgument(k,default_value=v) for k,v in [('bag',''),('rate','.5'),('exit_on_finish','false'),('rviz','false'),('sim_command_output','false')]]
    return LaunchDescription(args+[OpaqueFunction(function=setup)])
