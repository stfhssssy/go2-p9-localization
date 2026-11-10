from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from go2_p9.configuration import home, load


def setup(context):
    args = {key: LaunchConfiguration(key).perform(context) for key in
            ('mode', 'config', 'output', 'rviz', 'bag', 'rate', 'exit_on_finish')}
    if args['mode'] not in ('live', 'replay', 'sim'):
        raise ValueError('mode must be live, replay or sim')
    _, sensor, mapping, pointlio = load(args['config'], args['mode'])
    mapping['output_dir'] = str(Path(args['output']).resolve())
    sensor['diagnostic_output_path'] = str(Path(args['output']).resolve()/'input_status.json')
    adapter = Node(package='go2_p9', executable='p9_adapter', parameters=[sensor], output='screen')
    engine = Node(package='point_lio_unilidar', executable='pointlio_mapping', name='pointlio_mapping',
                  parameters=[pointlio], output='screen', remappings=[
                      ('/cloud_registered', '/pointlio/registered_scan'),
                      ('/path', '/pointlio/path'),
                      ('/aft_mapped_to_init', '/pointlio/odom'),
                      ('/slam/tracking_valid', '/pointlio/tracking_valid'),
                      ('/slam/filter_diagnostics', '/pointlio/filter_diagnostics')])
    mapper = Node(package='go2_p9', executable='p9_mapping', parameters=[mapping], output='screen')
    actions = [adapter, engine, mapper]
    for node in (adapter, engine, mapper):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A required P9 node exited; inspect its log'))])))
    if args['rviz'] == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(home()/'config/p9.rviz')],
                            parameters=[{'use_sim_time': args['mode'] in ('replay', 'sim')}], output='screen'))
    if args['mode'] == 'replay':
        topics = [sensor.get('input_cloud_topic', '/utlidar/cloud')]
        if sensor.get('imu_mode', 'merged_linear') == 'raw_linear':
            topics += [sensor.get('gyro_topic', '/camera/camera/gyro/sample'),
                       sensor.get('accel_topic', '/camera/camera/accel/sample'), '/tf_static']
        else:
            topics += [sensor.get('input_imu_topic', '/camera/camera/imu')]
        player = ExecuteProcess(cmd=['ros2', 'bag', 'play', args['bag'], '--clock', '100',
                    '--rate', args['rate'], '--read-ahead-queue-size', '4000',
                    '--qos-profile-overrides-path', str(home()/'config/replay-qos.yaml'), '--topics', *topics], output='screen')
        def finished(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Bag playback failed: '+str(event.returncode)))]
            if args['exit_on_finish'] == 'true':
                return [TimerAction(period=5., actions=[EmitEvent(event=Shutdown(reason='Bag EOF; save final map'))])]
            return [LogInfo(msg='Bag EOF. Map remains available in RViz. Run p9 save or Ctrl-C to save and exit.')]
        actions += [RegisterEventHandler(OnProcessExit(target_action=player, on_exit=finished)),
                    TimerAction(period=4., actions=[player])]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in
        dict(mode='live', config='', output='./p9-output', rviz='false', bag='', rate='0.5', exit_on_finish='false').items()]
        + [OpaqueFunction(function=setup)])
