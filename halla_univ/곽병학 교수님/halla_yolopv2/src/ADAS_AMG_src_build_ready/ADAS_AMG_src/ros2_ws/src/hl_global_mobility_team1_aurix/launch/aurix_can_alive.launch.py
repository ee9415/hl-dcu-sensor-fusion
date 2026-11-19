from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "hl_global_mobility_team1_aurix"
    params_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "aurix_can_alive.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="can0"),
            DeclareLaunchArgument("alive_period_ms", default_value="50.0"),
            DeclareLaunchArgument("reconnect_period_ms", default_value="200.0"),
            Node(
                package=package_name,
                executable="aurix_can_alive_node",
                name="aurix_can_alive_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "interface": LaunchConfiguration("interface"),
                        "alive_period_ms": LaunchConfiguration("alive_period_ms"),
                        "reconnect_period_ms": LaunchConfiguration("reconnect_period_ms"),
                    },
                ],
            ),
        ]
    )
