from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "hl_global_mobility_team1_aurix"
    params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "aurix_can_bridge.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="can0"),
            DeclareLaunchArgument("drive_topic", default_value="/ackermann_cmd"),
            Node(
                package=package_name,
                executable="aurix_can_bridge_node",
                name="aurix_can_bridge_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "interface": LaunchConfiguration("interface"),
                        "drive_topic": LaunchConfiguration("drive_topic"),
                    },
                ],
            ),
        ]
    )
