"""Launch the ROS2 GPS waypoint tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("gps_waypoint_tracker_ros2")
    rviz_config = os.path.join(
        share,
        "rviz",
        "gps_waypoint_tracker.rviz",
    )

    csv_path = LaunchConfiguration("csv_path")
    fix_topic = LaunchConfiguration("fix_topic")
    enable_plot = LaunchConfiguration("enable_plot")
    enable_csv_log = LaunchConfiguration("enable_csv_log")
    enable_rviz = LaunchConfiguration("enable_rviz")
    log_directory = LaunchConfiguration("log_directory")
    log_period_sec = LaunchConfiguration("log_period_sec")
    dt = LaunchConfiguration("dt")
    tau = LaunchConfiguration("tau")

    arguments = [
        DeclareLaunchArgument(
            "csv_path",
            default_value=(
                "/home/jigu/ros2_ws/src/0730_gps_test/"
                "waypoint_csv/0730_waypoint_straight.csv"
            ),
            description="Absolute waypoint CSV path.",
        ),
        DeclareLaunchArgument(
            "fix_topic",
            default_value="/gps1/fix",
            description="sensor_msgs/msg/NavSatFix input topic.",
        ),
        DeclareLaunchArgument(
            "enable_plot",
            default_value="true",
            description="Show the matplotlib waypoint window.",
        ),
        DeclareLaunchArgument(
            "enable_csv_log",
            default_value="true",
            description="Start the synchronized drive CSV logger.",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Start RViz2 with waypoint and GPS displays.",
        ),
        DeclareLaunchArgument(
            "log_directory",
            default_value=os.path.expanduser("~/ros2_ws/gps_logs"),
            description="Directory for timestamped drive CSV files.",
        ),
        DeclareLaunchArgument(
            "log_period_sec",
            default_value="0.05",
            description="CSV snapshot period; 0.05 s is 20 Hz.",
        ),
        DeclareLaunchArgument(
            "dt",
            default_value="0.05",
            description="LPF sample period in seconds.",
        ),
        DeclareLaunchArgument(
            "tau",
            default_value="0.25",
            description="LPF time constant in seconds.",
        ),
    ]

    tracker = Node(
        package="gps_waypoint_tracker_ros2",
        executable="gps_waypoint_tracker",
        name="gps_waypoint_tracker",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONNOUSERSITE": "1"},
        parameters=[
            {
                "csv_path": csv_path,
                "fix_topic": fix_topic,
                "enable_plot": ParameterValue(
                    enable_plot,
                    value_type=bool,
                ),
                "dt": ParameterValue(dt, value_type=float),
                "tau": ParameterValue(tau, value_type=float),
            }
        ],
    )

    csv_logger = Node(
        package="gps_waypoint_tracker_ros2",
        executable="drive_csv_logger",
        name="drive_csv_logger",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_csv_log),
        parameters=[
            {
                "fix_topic": fix_topic,
                "log_directory": log_directory,
                "log_period_sec": ParameterValue(
                    log_period_sec,
                    value_type=float,
                ),
            }
        ],
    )

    visualizer = Node(
        package="gps_waypoint_tracker_ros2",
        executable="waypoint_rviz_visualizer",
        name="waypoint_rviz_visualizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_rviz),
        parameters=[
            {
                "csv_path": csv_path,
                "fix_topic": fix_topic,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="jigu_gps_rviz2",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        arguments + [tracker, csv_logger, visualizer, rviz]
    )
