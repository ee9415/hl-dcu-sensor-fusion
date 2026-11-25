from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_ip', default_value='169.254.100.228'),
        Node(package='hl_oak_perception', executable='bridge', output='screen',
             parameters=[{'device_ip': LaunchConfiguration('device_ip')}]),
        Node(package='hl_oak_perception', executable='postprocess', output='screen'),
    ])
