"""Packaging for farmbot_hardware_comm.

Defines package metadata and console entry points for the ROS2 Farmbot hardware
communication package.
"""
import glob
import os

from setuptools import find_packages, setup


package_name = 'farmbot_hardware_comm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salome',
    maintainer_email='salome.deoliveira.2026@mumail.ie',
    description='Package containing the hardware communication for the ROS2 Farmbot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_controller = farmbot_hardware_comm.serial_controller:main',
            'gpio_controller = farmbot_hardware_comm.gpio_controller:main'
        ],
    },
)
