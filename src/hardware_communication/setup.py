from setuptools import find_packages, setup

package_name = 'hardware_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aura_ws',
    maintainer_email='salome.deoliveira.2026@mumail.ie',
    description='Package containing the hardware communication for the ROS2 Farmbot',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "uart_controller = hardware_communication.UART_controller:main",
        ],
    },
)
