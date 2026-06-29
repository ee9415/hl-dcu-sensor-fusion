import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hl_camera_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'blobs'), glob('blobs/*.blob')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qsentech',
    maintainer_email='ee9415@gmail.com',
    description='OAK-D Pro PoE camera bringup with YOLOv6n VPU inference',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov6n_node = hl_camera_bringup.yolov6n_node:main',
        ],
    },
)
