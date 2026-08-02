"""
Temporary dev launch: only the refactored nodes, for testing the new stack.

Brings up the two refactored, independently-launchable pieces of the current
architecture:
- serial_controller -- the typed hardware bridge (actions / services / triggers;
  the only node that speaks F-code; loads config via the ConfigServer helper).
- task_sequencer    -- the programmatic sequence engine (RunSequence action).

Legacy / not-yet-refactored nodes are intentionally excluded: the
farmbot_controller god-node, map_controller, gpio_controller, the camera stack
and the HRI nodes. Config is not a separate node any more (folded into the
bridge), so there is nothing else to launch.

Drive a sequence once it is up, e.g.:
    ros2 action send_goal /run_sequence farmbot_interfaces/action/RunSequence \\
        "{name: calibrate_axes}" --feedback
"""
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
            output='screen'
        ),
    ])
