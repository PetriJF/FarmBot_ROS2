from setuptools import find_packages, setup
import os, glob

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
            "uart_controller = farmbot_hardware_comm.uart_controller:main",
            "gpio_controller = farmbot_hardware_comm.gpio_controller:main"
        ],
    },
)
