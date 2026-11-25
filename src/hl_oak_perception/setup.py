from pathlib import Path
from setuptools import setup

core = Path('../../apps/yolopv2_standalone')
names = ('host_postprocess', 'perception_wire', 'image_boundaries', 'lane_boundaries',
         'lane_contours', 'lane_overlay', 'ego_lane_quadratic', 'stop_line',
         'virtual_guidance', 'top_midpoint_guidance', 'top_view_controls', 'skeleton_response')
setup(name='hl_oak_perception', version='0.1.0', packages=['hl_oak_perception'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hl_oak_perception']),
        ('share/hl_oak_perception', ['package.xml']),
        ('share/hl_oak_perception/launch', ['launch/oak_host.launch.py']),
        ('share/hl_oak_perception/core', [str(core/(name+'.py')) for name in names] +
            [str(core/'lane_boundaries.json'), str(core/'virtual_guidance.json')])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='qsentech', maintainer_email='ee9415@gmail.com', license='Apache-2.0',
    description='OAK4 raw inference receiver and host postprocessing',
    entry_points={'console_scripts': [
        'bridge = hl_oak_perception.bridge:main',
        'postprocess = hl_oak_perception.processor:main']})
