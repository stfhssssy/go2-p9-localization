"""TARE replaces FAR; each required child exit shuts down the entire stack."""
import json
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    manifest = Path(LaunchConfiguration('manifest').perform(context)).resolve()
    data = json.loads(manifest.read_text())
    base = Path(__file__).resolve().parent
    if str(base) not in sys.path: sys.path.insert(0, str(base))
    from make_config import validate_options
    g = data['guard']; mode = data['mode']
    validate_options(mode, [g[k] for k in ('min_x', 'max_x', 'min_y', 'max_y')],
                     g['max_forward_m_s'], g['max_lateral_m_s'], g['max_yaw_rad_s'],
                     data['sim_command_output'], data['sim_auto_resume_inputs'], not data['stop_on_finished'])
    if data['use_sim_time'] != (mode != 'live') or not data['boundary_configured']:
        raise ValueError('Invalid clock or missing task boundary')
    output = Path(data['output']); params = data['parameters']
    params['local_planner']['pathFolder'] = str(Path(get_package_share_directory('local_planner'))/'paths')
    command = ['ros2', 'run', 'go2_p9', 'p9', mode, '--config', data['effective_core_config'],
               '--output', str(output/'localization')]
    if mode == 'replay':
        command += ['--bag', data['bag'], '--rate', str(data['rate'])]
        if data['exit_on_finish']: command += ['--exit-on-finish']
    essential = [ExecuteProcess(cmd=command, output='screen')]
    isolate = [('/joy', '/p9/exploration/joy_unused'), ('/speed', '/p9/exploration/speed_unused'),
               ('/stop', '/p9/exploration/stop_unused')]
    specs = [('terrain_analysis', 'terrainAnalysis', 'terrain_analysis', []),
             ('terrain_analysis_ext', 'terrainAnalysisExt', 'terrain_analysis_ext', []),
             ('local_planner', 'localPlanner', 'local_planner', []),
             ('local_planner', 'pathFollower', 'path_follower', [('/cmd_vel', '/p9/navigation/raw_cmd_vel')])]
    for pkg, exe, key, remap in specs:
        essential.append(Node(package=pkg, executable=exe, name='p9_'+key, parameters=[params[key]],
                              remappings=isolate+remap, output='screen'))
    essential.append(Node(package='tare_planner', executable='tare_planner_node', name='tare_planner_node',
                          parameters=[data['tare_config']], output='screen', remappings=[
                              ('/local_path', '/p9/exploration/tare_path'),
                              ('/global_path', '/p9/exploration/global_path'),
                              ('/global_path_full', '/p9/exploration/global_path_full')]))
    guard = dict(g, mode=mode, use_sim_time=data['use_sim_time'], output=str(output),
                 boundary_configured=True, sim_command_output=data['sim_command_output'],
                 sim_auto_resume_inputs=data['sim_auto_resume_inputs'], stop_on_finished=data['stop_on_finished'])
    # Python ROS nodes are installed beside this launch file rather than in the frozen P9 overlay.
    def process(script, parameters):
        args = ['python3', str(base/script), '--ros-args']
        for key, value in parameters.items():
            scalar = str(value).lower() if isinstance(value, bool) else str(value)
            args.extend(['-p', key+':='+scalar])
        return ExecuteProcess(cmd=args, output='screen')
    essential.append(process('guard_node.py', guard))
    essential.append(process('boundary.py', dict(use_sim_time=data['use_sim_time'],
                     **{k: g[k] for k in ('min_x', 'max_x', 'min_y', 'max_y')})))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='A required P9 exploration process exited'))])) for node in essential]
    # Register handlers first, so a startup failure cannot race past its fail-closed handler.
    nodes = handlers+essential
    if data['rviz']:
        nodes.append(Node(package='rviz2', executable='rviz2', name='p9_exploration_rviz',
                          arguments=['-d', str(base/'exploration.rviz')],
                          parameters=[{'use_sim_time': data['use_sim_time']}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('manifest'), OpaqueFunction(function=setup)])
