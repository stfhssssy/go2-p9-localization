"""Bridge the two camera-link IMUs; launch no robot/controller/estimator."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='p9_camera_imu_sensor_bridge', output='screen',
        arguments=['/p9_sim/gyro@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   '/p9_sim/accel@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}])])
