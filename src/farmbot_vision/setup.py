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
    maintainer_email='eliotboda@gmail.com',
    description='Camera driver nodes for the ROS2 Farmbot vision system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'standard_camera = farmbot_vision.standard_camera:main',
            'luxonis_camera = farmbot_vision.luxonis_camera:main',
        ],
    },
)
