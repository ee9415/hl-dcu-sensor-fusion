from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('hl_camera_can'))/'config/camera_can.json')
    common = {'config_path': LaunchConfiguration('config_path')}
    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=config),
        DeclareLaunchArgument('enable_driver', default_value='false'),
        Node(package='hl_camera_can', executable='camera_path', output='screen', parameters=[common]),
        Node(package='hl_camera_can', executable='can_bridge', output='screen', parameters=[common]),
        Node(package='hl_camera_can', executable='can_driver', output='screen',
            parameters=[common, {'enabled': ParameterValue(LaunchConfiguration('enable_driver'), value_type=bool)}]),
    ])
