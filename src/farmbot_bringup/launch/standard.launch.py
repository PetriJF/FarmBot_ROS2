"""Launch description for the FarmBot ROS2 stack.

Defines the controller, parameter server, GPIO controller, and map
controller nodes with an optional camera flag.
"""
import os

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for the FarmBot ROS2 stack."""
    camera_choices = ['Standard', 'Luxonis']

    autonomous_mode = LaunchConfiguration('autonomous_mode')
    ws_path = LaunchConfiguration('ws_path')
    serial_port = LaunchConfiguration('serial_port')
    camera = LaunchConfiguration('camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'autonomous_mode',
            default_value='False',
            description='Enable autonomous mode for the FarmBot.'
        ),
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
        DeclareLaunchArgument(
            'camera',
            default_value='None',
            choices=['None'] + camera_choices,
            description='Select the camera type to use with the FarmBot.'
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
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'", camera, "'",
                    ' in ', str(camera_choices)
                ])
            )
        ),
        Node(
            package='camera_handler',
            executable='standard_camera',
            name='standard_camera',
            output='screen',
            parameters=[
                {'capture_freq': 1.0 / 30.0},  # 30 frames a second
            ],
            condition=IfCondition(
                PythonExpression([
                    "'", camera, "'",
                    " == 'Standard'"
                ])
            )
        ),
        Node(
            package='camera_handler',
            executable='luxonis_camera',
            name='luxonis_camera',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'", camera, "'",
                    " == 'Luxonis'"
                ])
            )
        ),
        Node(
            package='farmbot_hri',
            executable='autonomous_controller',
            name='autonomous_controller',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    autonomous_mode,
                    '== True'
                ])
            )
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
                        {'check_uart_freq': 100},
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
