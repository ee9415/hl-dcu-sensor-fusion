"""Launch the PPT Nominal Driving (0x314) SocketCAN bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "steering_topic",
            default_value="/filtered_steering_angle1",
        ),
        DeclareLaunchArgument(
            "waypoint_index_topic",
            default_value="/current_waypoint_index1",
        ),
        DeclareLaunchArgument(
            "route_completion_topic",
            default_value="/route_completed1",
        ),
        DeclareLaunchArgument(
            "heading_valid_topic",
            default_value="/heading_valid1",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
        ),
        DeclareLaunchArgument(
            "can_id",
            default_value="788",
            description="Standard CAN ID as decimal; 788 is 0x314 Nominal Driving.",
        ),
        DeclareLaunchArgument(
            "feedback_can_id",
            default_value="304",
            description="Standard CAN ID as decimal; 304 is 0x130.",
        ),
        DeclareLaunchArgument(
            "send_period_sec",
            default_value="0.05",
        ),
        DeclareLaunchArgument(
            "command_timeout_sec",
            default_value="0.20",
        ),
        DeclareLaunchArgument(
            "target_speed_kph",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "enable_drive",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "invert_steering",
            default_value="true",
            description=(
                "Convert the tracker topic sign back to AURIX "
                "+LEFT/-RIGHT."
            ),
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Publish debug payloads without opening SocketCAN.",
        ),
        DeclareLaunchArgument(
            "startup_rearm_frames",
            default_value="10",
            description=(
                "Number of valid inactive 0x19 frames sent before drive "
                "activation is allowed."
            ),
        ),
        DeclareLaunchArgument("crawl_speed_kph", default_value="1.0"),
        DeclareLaunchArgument("maximum_speed_kph", default_value="8.0"),
        DeclareLaunchArgument(
            "speed_ramp_kph_per_sec",
            default_value="1.0",
        ),
        DeclareLaunchArgument(
            "steering_command_scale",
            default_value="0.70",
            description=(
                "ROS-side compensation for the uploaded AURIX steering "
                "feed-forward overshoot."
            ),
        ),
        DeclareLaunchArgument(
            "maximum_tracker_steering_deg",
            default_value="7.0",
            description=(
                "Clamp the tracker request before applying steering scale."
            ),
        ),
        DeclareLaunchArgument(
            "saturated_steering_speed_kph",
            default_value="1.0",
            description=(
                "Speed cap while the tracker requests more steering than "
                "the compensated physical limit."
            ),
        ),
        DeclareLaunchArgument(
            "max_steering_rate_deg_per_sec",
            default_value="2.0",
        ),
        DeclareLaunchArgument("feedback_timeout_sec", default_value="0.20"),
        DeclareLaunchArgument(
            "max_steering_tracking_error_deg",
            default_value="5.0",
        ),
        DeclareLaunchArgument(
            "steering_error_timeout_sec",
            default_value="0.30",
        ),
        DeclareLaunchArgument(
            "steering_center_tolerance_deg",
            default_value="4.0",
        ),
        DeclareLaunchArgument(
            "steering_center_hold_sec",
            default_value="0.50",
        ),
        DeclareLaunchArgument(
            "pc_active_timeout_sec",
            default_value="0.50",
        ),
    ]

    bridge = Node(
        package="gps_waypoint_tracker_ros2",
        executable="aurix_can_bridge",
        name="aurix_can_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "steering_topic": LaunchConfiguration(
                    "steering_topic"
                ),
                "waypoint_index_topic": LaunchConfiguration(
                    "waypoint_index_topic"
                ),
                "route_completion_topic": LaunchConfiguration(
                    "route_completion_topic"
                ),
                "heading_valid_topic": LaunchConfiguration(
                    "heading_valid_topic"
                ),
                "can_interface": LaunchConfiguration(
                    "can_interface"
                ),
                "can_id": ParameterValue(
                    LaunchConfiguration("can_id"),
                    value_type=int,
                ),
                "feedback_can_id": ParameterValue(
                    LaunchConfiguration("feedback_can_id"),
                    value_type=int,
                ),
                "send_period_sec": ParameterValue(
                    LaunchConfiguration("send_period_sec"),
                    value_type=float,
                ),
                "command_timeout_sec": ParameterValue(
                    LaunchConfiguration("command_timeout_sec"),
                    value_type=float,
                ),
                "target_speed_kph": ParameterValue(
                    LaunchConfiguration("target_speed_kph"),
                    value_type=float,
                ),
                "enable_drive": ParameterValue(
                    LaunchConfiguration("enable_drive"),
                    value_type=bool,
                ),
                "invert_steering": ParameterValue(
                    LaunchConfiguration("invert_steering"),
                    value_type=bool,
                ),
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"),
                    value_type=bool,
                ),
                "startup_rearm_frames": ParameterValue(
                    LaunchConfiguration("startup_rearm_frames"),
                    value_type=int,
                ),
                "crawl_speed_kph": ParameterValue(
                    LaunchConfiguration("crawl_speed_kph"),
                    value_type=float,
                ),
                "maximum_speed_kph": ParameterValue(
                    LaunchConfiguration("maximum_speed_kph"),
                    value_type=float,
                ),
                "speed_ramp_kph_per_sec": ParameterValue(
                    LaunchConfiguration("speed_ramp_kph_per_sec"),
                    value_type=float,
                ),
                "steering_command_scale": ParameterValue(
                    LaunchConfiguration("steering_command_scale"),
                    value_type=float,
                ),
                "maximum_tracker_steering_deg": ParameterValue(
                    LaunchConfiguration("maximum_tracker_steering_deg"),
                    value_type=float,
                ),
                "saturated_steering_speed_kph": ParameterValue(
                    LaunchConfiguration("saturated_steering_speed_kph"),
                    value_type=float,
                ),
                "max_steering_rate_deg_per_sec": ParameterValue(
                    LaunchConfiguration("max_steering_rate_deg_per_sec"),
                    value_type=float,
                ),
                "feedback_timeout_sec": ParameterValue(
                    LaunchConfiguration("feedback_timeout_sec"),
                    value_type=float,
                ),
                "max_steering_tracking_error_deg": ParameterValue(
                    LaunchConfiguration("max_steering_tracking_error_deg"),
                    value_type=float,
                ),
                "steering_error_timeout_sec": ParameterValue(
                    LaunchConfiguration("steering_error_timeout_sec"),
                    value_type=float,
                ),
                "steering_center_tolerance_deg": ParameterValue(
                    LaunchConfiguration("steering_center_tolerance_deg"),
                    value_type=float,
                ),
                "steering_center_hold_sec": ParameterValue(
                    LaunchConfiguration("steering_center_hold_sec"),
                    value_type=float,
                ),
                "pc_active_timeout_sec": ParameterValue(
                    LaunchConfiguration("pc_active_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [bridge])
