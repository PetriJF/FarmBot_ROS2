from rclpy.node import Node
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart

import time

def generate_launch_description():
    param_conf_srv_node = Node(
        package = 'farmbot_controllers',
        executable = 'param_conf_server',
        name = 'param_conf_server'
    )

    panel_node = Node(
        package = 'farmbot_controllers',
        executable = 'panel_controller',
        name = 'panel_controller'
    )

    controller_node = Node(
        package = 'farmbot_controllers',
        executable = 'farmbot_controller',
        name = 'controller'
    )

    motor_interp_node = Node(
        package = 'farmbot_command_handler',
        executable = 'motor_command_handler',
        name = 'motor_command_handler'
    )

    state_interp_node = Node(
        package = 'farmbot_command_handler',
        executable = 'state_command_handler',
        name = 'state_command_handler'
    )

    device_interp_node = Node(
        package = 'farmbot_command_handler',
        executable = 'device_command_handler',
        name = 'device_command_handler'
    )

    map_ctrl_node = Node(
        package = 'map_handler',
        executable = 'map_controller',
        name = 'map_controller'
    )
    
    luxonis_publisher_node = Node(
        package = 'camera_handler',
        executable = 'luxonis_publisher',
        name = 'luxonis_publisher'
    )

    uart_ctrl_node = Node(
        package = 'farmbot_command_handler',
        executable = 'uart_controller',
        name = 'uart_controller'
    )

    already_started_nodes = set()

    def start_next_node(event: ProcessStarted, context: LaunchContext):
        print(f'node {event.process_name} started.')
        already_started_nodes.update([event.process_name])
        if len(already_started_nodes) == 8:
            print(f'all required nodes are up, starting uart_controller')
            time.sleep(5)
            return uart_ctrl_node

    return LaunchDescription([
        RegisterEventHandler(event_handler = OnProcessStart(target_action = param_conf_srv_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = panel_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = controller_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = motor_interp_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = state_interp_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = device_interp_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = map_ctrl_node,
                                                            on_start = start_next_node)),
        RegisterEventHandler(event_handler = OnProcessStart(target_action = luxonis_publisher_node,
                                                            on_start = start_next_node)),
        param_conf_srv_node,
        panel_node,
        controller_node,
        motor_interp_node,
        state_interp_node,
        device_interp_node,
        map_ctrl_node,
        luxonis_publisher_node,
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()