import launch
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farmbot_controllers',
            executable='param_conf_server',
            name='param_conf_server',
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='panel_controller',
            name='panel_controller',
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='farmbot_controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='motor_command_handler',
            name='motor_command_handler',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='state_command_handler',
            name='state_command_handler',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='device_command_handler',
            name='device_command_handler',
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
            executable='luxonis_node',
            name='luxonis_node',
            output='screen'
        ),
        
        # Delay for 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                # Start uart_controller after the delay
                Node(
                    package='farmbot_command_handler',
                    executable='uart_controller',
                    name='uart_controller',
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()