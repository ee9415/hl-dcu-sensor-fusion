"""Launch only the synchronized GPS/CAN/AURIX CSV logger."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "fix_topic",
            default_value="/gps1/fix",
        ),
        DeclareLaunchArgument(
            "log_directory",
            default_value=os.path.expanduser("~/ros2_ws/gps_logs"),
        ),
        DeclareLaunchArgument(
            "log_period_sec",
            default_value="0.05",
            description="CSV snapshot period; 0.05 s is 20 Hz.",
        ),
        DeclareLaunchArgument(
            "require_gps",
            default_value="true",
            description="Write rows only after the first GPS message.",
        ),
    ]

    logger = Node(
        package="gps_waypoint_tracker_ros2",
        executable="drive_csv_logger",
        name="drive_csv_logger",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "fix_topic": LaunchConfiguration("fix_topic"),
                "log_directory": LaunchConfiguration(
                    "log_directory"
                ),
                "log_period_sec": ParameterValue(
                    LaunchConfiguration("log_period_sec"),
                    value_type=float,
                ),
                "require_gps": ParameterValue(
                    LaunchConfiguration("require_gps"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [logger])
