import glob
import os

from setuptools import find_packages, setup

package_name = 'farmbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eliot',
    maintainer_email='eliot.boda.2026@mumail.ie',
    description='Camera driver nodes for the ROS2 Farmbot vision system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'standard_camera = farmbot_vision.standard_camera:main',
            'plant_radius = farmbot_vision.plant_radius:main',
            'camera_calibration = farmbot_vision.camera_calibration:main',
            'image_stitcher = farmbot_vision.image_stitcher:main',
            # Disabled: DepthAI v2 API deprecated, needs updating before use.
            # 'luxonis_camera = farmbot_vision.luxonis_camera:main',
        ],
    },
)
