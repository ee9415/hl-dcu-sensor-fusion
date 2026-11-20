from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("perception_camera_py"),
        "config",
        "perception_camera.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="perception_camera_py",
                executable="camera_perception_node",
                name="camera_perception_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
