from glob import glob
from setuptools import find_packages, setup

package_name = "hl_global_mobility_team1_aurix"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
        (f"share/{package_name}/systemd", glob("systemd/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HL Global Mobility Team 1",
    maintainer_email="team1@hlglobalmobility.local",
    description="ROS2 CAN alive bridge for the AURIX Ackermann vehicle.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aurix_can_alive_node = hl_global_mobility_team1_aurix.aurix_can_alive_node:main",
        ],
    },
)
