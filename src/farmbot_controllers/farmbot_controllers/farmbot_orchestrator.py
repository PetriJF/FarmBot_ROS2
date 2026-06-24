#!/usr/bin/env python3
"""
Farmbot Orchestrator module.

Provides the FarmbotOrchestrator ROS2 node for managing the transmission of commands to the UART
controller through priority and non-priority queues.
"""
from farmbot_interfaces.action import FarmbotComms

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from std_msgs.msg import Bool, String


class FarmbotOrchestrator(Node):
    """
    Orchestrate FarmBot command routing.

    This node receives commands from the farmbot_command topic and manages
    priority and non-priority commands for FarmbotComms action.
    """

    # Node contructor
    def __init__(self):
        """Initialize the FarmbotOrchestrator node."""
        super().__init__('FarmbotOrchestrator')

        self.declare_parameter('tx_freq', rclpy.Parameter.Type.INTEGER)
        tx_freq = self.get_parameter('tx_freq').get_parameter_value().integer_value

        # Initializing the queue
        self.queue = {
            'priority_cmd': [],
            'non_priority_cmd': []
        }

        # Node subscripters and publishers
        self.uart_tx_sub = self.create_subscription(String, 'farmbot_command',
                                                    self.farmbot_command_callback, 10)

        self.busy_state = False

        # Farmbot state publisher
        self.estop_pressed = Bool()
        self.estop_pressed.data = False
        self.farmbot_estop_pub = self.create_publisher(Bool, 'estop', 10)

        self.action_status_timer = self.create_timer(1.0 / tx_freq, self.check_action_status)

        self.farmbot_comm_client = ActionClient(self, FarmbotComms, 'farmbot_communication')

        while not self.farmbot_comm_client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for Action Server...')

        # Log the initialization
        self.get_logger().info('Farmbot Orchestrator Initialized..')

    def farmbot_command_callback(self, message):
        """
        Handle the queue.

        Analyze messages from the /farmbot_command topic and enqueue them according to their
        priority.
        """
        self.get_logger().info(f'message to send: {message.data}')

        if message.data in ['E', 'F09', '@']:
            self.queue['non_priority_cmd'].clear()
            self.queue['priority_cmd'].clear()
            self.queue['priority_cmd'].append(message.data)
            if message.data == 'E':
                self.estop_pressed.data = True
            elif message.data == 'F09':
                self.estop_pressed.data = False
            self.farmbot_estop_pub.publish(self.estop_pressed)
        else:
            self.queue['non_priority_cmd'].append(message.data)

    def check_action_status(self):
        """Check queued commands and busy state to determine if the next command can be sent."""
        command = ''

        if self.queue['priority_cmd']:
            if self.busy_state:
                self.goal_handle.cancel_goal_async()
            else:
                command = self.queue['priority_cmd'].pop(0)
                self.send_goal(command)

        elif self.queue['non_priority_cmd'] and not self.busy_state:
            command = self.queue['non_priority_cmd'].pop(0)
            self.send_goal(command)

        else:
            return

    def send_goal(self, cmd):
        """Send a command goal to the serial controller through the action server."""
        goal = FarmbotComms.Goal()
        goal.command = cmd
        self.get_logger().info(f'{cmd}')

        self.farmbot_comm_client.send_goal_async(
            goal,
            feedback_callback=self.goal_feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the action server's goal response."""
        self.goal_handle: ClientGoalHandle = future.result()

        if self.goal_handle.accepted:
            self.busy_state = True
            self.get_logger().info('Goal accepted')

            self.goal_handle.get_result_async().add_done_callback(
                self.goal_result_callback
            )

        else:
            self.get_logger().warn('Goal rejected')

    def goal_feedback_callback(self, feedback_msg):
        """Handle feedback messages from the FarmbotComms action server."""
        current_position = feedback_msg.feedback.current_position
        percentage = feedback_msg.feedback.percentage
        self.get_logger().info(f'Current postion : X{current_position[0]} \
                               Y{current_position[1]} Z{current_position[2]}')
        self.get_logger().info(f'Goal completion: {percentage}')

    def goal_result_callback(self, future):
        """Handle the final result from the FarmbotComms action server."""
        result = future.result().status
        self.busy_state = False

        if result == 'CANCELED':
            self.get_logger().info('The current command has been canceled by a estop request')
        if result == 'ABORTED':
            self.get_logger().info('Something happened the command has been aborted.')
        elif result == 'SUCCEED':
            self.get_logger().info('The command was successful and has been completed')

        self.get_logger().info('Farmbot is ready for the next command')


def main(args=None):
    """Initialize and run the Farmbot orchestrator node."""
    rclpy.init(args=args)

    main_ctrl_node = FarmbotOrchestrator()

    try:
        rclpy.spin(main_ctrl_node)
    except KeyboardInterrupt:
        main_ctrl_node.destroy_node()

    main_ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
