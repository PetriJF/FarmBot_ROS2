"""Launch description for FarmBot nodes on Luxonis hardware."""
from launch import LaunchDescription, LaunchService
from launch.actions import TimerAction

from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description for FarmBot nodes on Luxonis hardware."""
    return LaunchDescription([
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
            package='map_handler',
            executable='map_controller',
            name='map_controller',
            output='screen'
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
                # Start uart_controller after the delay
                Node(
                    package='farmbot_hardware_comm',
                    executable='uart_controller',
                    name='uart_controller',
                    output='screen',
                    parameters=[
                        {'serial_port': '/dev/ttyACM0'},
                        {'serial_speed': 115200},
                        {'check_uart_freq': 100},
                        {'tx_freq': 10},
                    ]
                )
            ]
        )
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
