import launch
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch.actions import TimerAction, DeclareLaunchArgument

def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera',
            default_value = 'False'
        ),
        Node(
            package='parameters_manager',
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
            output='screen',
            condition = IfCondition(
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
            condition = IfCondition(
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
                # Start uart_controller after the delay
                Node(
                    package='farmbot_hardware_comm',
                    executable='uart_controller',
                    name='uart_controller',
                    output='screen',
                    parameters = [
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