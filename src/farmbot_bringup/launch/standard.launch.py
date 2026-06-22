"""Launch description for the FarmBot ROS2 stack.

Defines the controller, parameter server, GPIO controller, and map
controller nodes with an optional camera flag.
"""
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for the FarmBot ROS2 stack."""
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera',
            default_value='False'
        ),
        Node(
            package='farmbot_controllers',
            executable='param_conf_server',
            name='param_conf_server',
            output='screen'
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
            output='screen'
        ),
        Node(
            package='camera_handler',
            executable='camera_controller',
            name='camera_controller',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    use_camera,
                    '== True'
                ])
            )
        ),
        Node(
            package='camera_handler',
            executable='standard_camera',
            name='standard_camera',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    use_camera,
                    '== True'
                ])
            )
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
