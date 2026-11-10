"""Only Gazebo/CHAMP and sensor normalization; no estimator or navigation stack."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

HERE = Path(__file__).resolve().parent


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('output'),
        DeclareLaunchArgument('sensor_start_delay', default_value='40.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE/'simulation.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'), 'use_sim_time': 'true',
                'gui': 'false', 'rviz': 'false', 'champ_odom': 'false',
                'disable_camera': 'true', 'disable_d455': 'true',
                'disable_velodyne_lidar': 'true', 'disable_lidar_l1': 'false',
                'unitree_go2_description_path': str(HERE/'sensor_model/go2_p9_camera_imu.urdf.xacro'),
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE/'sensor_model/bridges.launch.py'))),
        ExecuteProcess(cmd=['python3', str(HERE/'p9_sensor_adapter.py'), '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', ['sensor_start_delay:=', LaunchConfiguration('sensor_start_delay')],
            '-p', 'manifest:='+str(HERE/'sensor_model/model_manifest.json'),
            '-p', ['summary_path:=', PathJoinSubstitution([LaunchConfiguration('output'), 'sensor_summary.json'])]],
            output='screen'),
    ])
