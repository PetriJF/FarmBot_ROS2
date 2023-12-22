from setuptools import find_packages, setup

package_name = 'farmbot_command_handler'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farmbotdev',
    maintainer_email='jamespetri28@gmail.com',
    description='This Package deals with the command interpreter that encodes the tasks in the FarmBot GCode.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_command_handler = farmbot_command_handler.motor_cmd_handler:main",
            "device_command_handler = farmbot_command_handler.device_cmd_handler:main",
            "state_command_handler = farmbot_command_handler.state_cmd_handler:main",
            "uart_controller = farmbot_command_handler.UART_controller:main"
        ],
    },
)
