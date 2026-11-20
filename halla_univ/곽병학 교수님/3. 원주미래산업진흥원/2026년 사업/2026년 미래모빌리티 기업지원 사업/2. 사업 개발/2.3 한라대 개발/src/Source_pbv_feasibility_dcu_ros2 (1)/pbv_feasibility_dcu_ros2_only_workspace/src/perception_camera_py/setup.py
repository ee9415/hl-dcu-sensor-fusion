from setuptools import setup
from glob import glob
import os

package_name = 'perception_camera_py'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='openai@example.com',
    description='ROS 2 Python camera perception node for the PBV feasibility DCU ROS2-only workspace',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_perception_node = perception_camera_py.camera_perception_node:main',
        ],
    },
)
