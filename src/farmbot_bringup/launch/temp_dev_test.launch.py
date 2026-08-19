"""Temporary dev launch: only the refactored nodes, for testing the new stack."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description with only the refactored nodes."""
    ws_path = LaunchConfiguration('ws_path')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_path',
            default_value=os.path.expanduser('~/FarmBot_ROS2/farmbot_data'),
            description='Path to the farmbot_data folder, which contains the config files.'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for communicating with the FarmBot.'
        ),

        # The serial bridge
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
        ),
        # The GPIO controller
        Node(
            package='farmbot_hardware_comm',
            executable='gpio_controller',
            name='gpio_controller',
            output='screen',
            parameters=[
                {'flashing_frequency': 2.0},
            ]
        ),
        # The sequence engine
        Node(
            package='farmbot_controllers',
            executable='task_sequencer',
            name='task_sequencer',
            output='screen',
            parameters=[
                # TODO: placeholder, needs calibrating together with plant_radius's
                # mm_per_pixel (see standard.launch.py)
                {'radius_capture_z': 0.0},
            ]
        ),
    ])
