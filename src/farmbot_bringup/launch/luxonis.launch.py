"""Launch description for FarmBot nodes on Luxonis hardware."""
import os

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for FarmBot nodes on Luxonis hardware."""
    ws_path = LaunchConfiguration('ws_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_path',
            default_value=os.path.expanduser('~/FarmBot_ROS2/farmbot_data')
        ),
        Node(
            package='farmbot_controllers',
            executable='param_conf_server',
            name='param_conf_server',
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
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='farmbot_controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='farmbot_orchestrator',
            name='farmbot_orchestrator',
            output='screen',
                    parameters=[
                        {'tx_freq': 10},
                    ]
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
            package='camera_handler',
            executable='camera_controller',
            name='camera_controller',
            output='screen'
        ),
        Node(
            package='camera_handler',
            executable='luxonis_camera',
            name='luxonis_camera',
            output='screen'
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
                        {'serial_port': '/dev/ttyACM0'},
                        {'serial_speed': 115200},
                        {'check_uart_freq': 100},
                    ]
                )
            ]
        )
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
