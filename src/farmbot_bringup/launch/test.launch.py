"""Launch description for the FarmBot ROS2 stack.

Defines the controller, parameter server, GPIO controller, and map
controller nodes with an optional camera flag.
"""
import os

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for the FarmBot ROS2 stack."""
    ws_path = LaunchConfiguration('ws_path')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_path',
            default_value=os.path.expanduser('~/FarmBot_ROS2/farmbot_data'),
            description='Path to the farmbot_data folder, which contains the configuration files.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for communicating with the FarmBot.'
        ),
        Node(
            package='farmbot_controllers',
            executable='test_client',
            name='test_client',
            output='screen'
        ),
        Node(
            package='map_handler',
            executable='map_controller',
            name='map_controller',
            output='screen',
            parameters=[
                {'ws_path': ws_path},
                {'folder_config_name': 'local_config'},
            ]
        ),
        Node(
            package='farmbot_hardware_comm',
            executable='gpio_controller',
            name='gpio_controller',
            output='screen',
            parameters=[
                {'flashing_frequency': 2.0},
            ]
        ),
        # Delay for 10 seconds
        TimerAction(
            period=10.0,
            actions=[
                # Start serial_controller after the delay
                Node(
                    package='farmbot_hardware_comm',
                    executable='serial_controller',
                    name='serial_controller',
                    output='screen',
                    parameters=[
                        {'serial_port': serial_port},
                        {'serial_speed': 115200},
                        {'check_serial_freq': 100},
                        {'ws_path': ws_path},
                        {'folder_config_name': 'local_config'},
                    ]
                )
            ]
        )
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
