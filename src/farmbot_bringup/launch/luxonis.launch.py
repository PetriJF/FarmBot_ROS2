"""Launch description for FarmBot nodes on Luxonis hardware."""
import os

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for FarmBot nodes on Luxonis hardware."""
    ws_path = LaunchConfiguration('ws_path')
    autonomous_mode = LaunchConfiguration('autonomous_mode')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ws_path',
            default_value=os.path.expanduser('~/FarmBot_ROS2/farmbot_data')
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0'
        ),
        DeclareLaunchArgument(
            'autonomous_mode',
            default_value='False'
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
            output='screen'
        ),
        Node(
            package='camera_handler',
            executable='luxonis_camera',
            name='luxonis_camera',
            output='screen'
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
                    ]
                )
            ]
        )
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
