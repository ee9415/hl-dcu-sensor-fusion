from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    camera_config = os.path.join(
        get_package_share_directory("perception_camera_py"),
        "config",
        "perception_camera.yaml",
    )
    dcu_config = os.path.join(
        get_package_share_directory("pbv_dcu_core_ros2"),
        "config",
        "dcu_runtime.conf",
    )

    simulate_no_camera = LaunchConfiguration("simulate_no_camera")
    use_socketcan = LaunchConfiguration("use_socketcan")
    can_if = LaunchConfiguration("can_if")
    enable_csv_log = LaunchConfiguration("enable_csv_log")

    return LaunchDescription([
        DeclareLaunchArgument("simulate_no_camera", default_value="false"),
        DeclareLaunchArgument("use_socketcan", default_value="false"),
        DeclareLaunchArgument("can_if", default_value="vcan0"),
        DeclareLaunchArgument("enable_csv_log", default_value="false"),
        Node(
            package="perception_camera_py",
            executable="camera_perception_node",
            name="camera_perception_node",
            output="screen",
            parameters=[camera_config, {"simulate_no_camera": simulate_no_camera}],
        ),
        Node(
            package="pbv_dcu_core_ros2",
            executable="pbv_dcu_pipeline_node",
            name="pbv_dcu_pipeline_node",
            output="screen",
            parameters=[
                {
                    "config_path": dcu_config,
                    "use_socketcan": use_socketcan,
                    "can_if": can_if,
                    "enable_csv_log": enable_csv_log,
                }
            ],
        ),
    ])
