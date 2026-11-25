from setuptools import setup

setup(name='hl_camera_can', version='0.1.0', packages=['hl_camera_can'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/hl_camera_can']),
        ('share/hl_camera_can', ['package.xml']),
        ('share/hl_camera_can/config', ['config/camera_can.json']),
        ('share/hl_camera_can/launch', ['launch/camera_can.launch.py'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='qsentech', maintainer_email='ee9415@gmail.com', license='Apache-2.0',
    entry_points={'console_scripts': [
        'camera_path = hl_camera_can.nodes:camera_main',
        'can_bridge = hl_camera_can.nodes:bridge_main',
        'can_driver = hl_camera_can.nodes:driver_main']})
