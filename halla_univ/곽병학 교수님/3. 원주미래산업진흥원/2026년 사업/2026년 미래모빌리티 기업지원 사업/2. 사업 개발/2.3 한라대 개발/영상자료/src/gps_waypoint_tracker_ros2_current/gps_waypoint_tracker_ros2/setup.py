import glob
import os

from setuptools import find_packages, setup


package_name = "gps_waypoint_tracker_ros2"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            [
                "package.xml",
                "README.md",
                "AURIX_CAN_ALIGNMENT.md",
            ],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob.glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jigu",
    maintainer_email="jigu@todo.todo",
    description=(
        "ROS2 port of the GPS waypoint tracker and steering publisher."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            (
                "gps_waypoint_tracker = "
                "gps_waypoint_tracker_ros2.gps_waypoint_tracker:main"
            ),
            (
                "aurix_can_bridge = "
                "gps_waypoint_tracker_ros2.aurix_can_bridge:main"
            ),
            (
                "drive_csv_logger = "
                "gps_waypoint_tracker_ros2.drive_csv_logger:main"
            ),
            (
                "waypoint_rviz_visualizer = "
                "gps_waypoint_tracker_ros2."
                "waypoint_rviz_visualizer:main"
            ),
        ],
    },
)
