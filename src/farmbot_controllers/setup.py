"""
Setup configuration for farmbot_controllers package.

Defines package metadata, dependencies, and console entry points for the
ROS2 Farmbot controllers package.
"""
import glob
import os

from setuptools import find_packages, setup
package_name = 'farmbot_controllers'

setup(
    name=package_name,
    version='1.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James',
    maintainer_email='jamespetri28@gmail.com',
    description='Package containing the main controllers and modules for the ROS2 Farmbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'farmbot_controller = farmbot_controllers.farmbot_controller:main',
            'param_conf_server = farmbot_controllers.config_managers:main'
        ],
    },
)
