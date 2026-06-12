#!/usr/bin/env python3
"""
Farmbot serial controller module.

Handles ROS2 /farmbot_command input, forwards commands to the Farmduino over serial,
and publishes feedback and busy state updates.
"""
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_hardware_comm.fcode_encoder import DeviceCmdHandler, MotorCmdHandler, StateCmdHandler

from farmbot_interfaces.action import FarmbotControl
from farmbot_interfaces.msg import FBPanel
from farmbot_interfaces.srv import LedPanelHandler

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import serial

from std_msgs.msg import String

import yaml


class SerialController(Node):
    """
    Farmbot ROS2 node that handles the Serial messages going to and from the Farmduino.

    The Node receives commands through the FarmbotControl action and sends them to the
    Farmduino.

    When the node receives feedback from the Farmduino through Serial, the message is
    decoded and carried on to the relevant nodes.

    Output Topics:
        - /farmbot_feedback {String} -> The information that is received from serial and is carried
            on through the system.
    """

    # Node contructor
    def __init__(self):
        """Node Constructor."""
        super().__init__('SerialController')

        self.uart_cmd = String()
        self.temp = String()

        self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('serial_speed', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('check_uart_freq', rclpy.Parameter.Type.INTEGER)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_speed = self.get_parameter('serial_speed').get_parameter_value().integer_value
        self.check_uart_freq = self.get_parameter(
            'check_uart_freq').get_parameter_value().integer_value

        # Initializing farmbot command handler modules
        self.device_cmd_handler = DeviceCmdHandler(self)
        self.motor_cmd_handler = MotorCmdHandler(self)
        self.state_cmd_handler = StateCmdHandler(self)

        # UART receive publisher
        self.fb_feedback_pub = self.create_publisher(String, 'farmbot_feedback', 10)

        # Initialize the Action Server
        self.farmbot_control_server = ActionServer(
            self,
            FarmbotControl,
            'farmbot_control',
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Initialize Serial Communication
        self.ser = serial.Serial(serial_port, serial_speed, timeout=1)
        self.ser.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rx_timer = self.create_timer(1.0 / self.check_uart_freq, self.uart_receive)

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd = ''
        self.command_is_finished = False

        # Initialize the LED states
        self.LED_client(FBPanel.ESTOP_LED, FBPanel.ON)
        self.LED_client(FBPanel.UNLOCK_LED, FBPanel.ON)

        self.directory = os.path.join(
            get_package_share_directory('farmbot_hardware_comm'),
            'config'
        )
        self.non_immediate_cmds = yaml.safe_load(open(os.path.join(self.directory,
                                                                   'CommandsResponses.yaml'), 'r'))

        # Log the initialization
        self.get_logger().info('Serial Controller Initialized..')

    def goal_callback(self, goal_request):
        """Accept the goal request."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept the cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_callback(self, goal_handle):
        """Create a timer to track the command status."""
        self.goal_handle = goal_handle
        command = goal_handle.request.command

        self.farmbot_cmd_sender(command)
        self.get_logger().info('Executing goal...')
        self.check_status_timer = self.create_timer(1.0 / self.check_uart_freq, self.check_status)

    def check_status(self):
        """
        Check command execution status and handle completion or cancellation.

        Publishes feedback, handles goal cancellation requests, and triggers
        the next callback based on command completion state.
        """
        goal_handle = self.goal_handle

        # Create Feedback object
        # feedback = FarmbotControl.Feedback()
        # goal_handle.publish_feedback(feedback)

        if self.command_is_finished and not self.check_status_timer.is_canceled():
            self.get_logger().info('Goal completed.')
            goal_handle.execute()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute the FarmbotControl action goal.

        Checks if the command has finished executing. If so, cancels the
        status timer and returns the result. Otherwise, sends the command
        to the Farmduino and logs execution status.
        """
        self.command_is_finished = False
        self.check_status_timer.cancel()

        # Create Result object
        result = FarmbotControl.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result

        self.get_logger().info('Command is sucessful')
        goal_handle.succeed()
        return result

    def farmbot_cmd_sender(self, cmd: str):
        """
        Handle the commands that are queued to be sent to the farmbot through UART.

        Two cases:
            a) the command has priority (e.g. electronic-stop):
                The command bypasses the queue and the queue is reset
            b) standard command:
                The command is added at the end of the queue
        """
        command: list = cmd.split(' ')

        match command[0]:

            # Device Command Handler Cases
            case 'E':
                self.temp.data = 'E'
                self.LED_client(FBPanel.ESTOP_LED, FBPanel.OFF)
                self.LED_client(FBPanel.UNLOCK_LED, FBPanel.FLASHING)

            case 'F09':
                self.temp.data = 'F09'
                self.LED_client(FBPanel.ESTOP_LED, FBPanel.ON)
                self.LED_client(FBPanel.UNLOCK_LED, FBPanel.ON)

            case 'i2c_command':
                self.temp.data = self.device_cmd_handler.i2c_cmd(command[1:])

            case 'pin_command':
                self.temp.data = self.device_cmd_handler.pin_cmd(command[1:])

            case 'water_command':
                self.temp.data = self.device_cmd_handler.water_cmd(command[1:])

            # Motor Command Handler Cases
            case 'home_handler':
                self.temp.data = self.motor_cmd_handler.home_cmd(command[1:])

            case 'move_gantry':
                self.temp.data = self.motor_cmd_handler.gantry_cmd(command[1:])

            case 'move_servo':
                self.temp.data = self.motor_cmd_handler.servo_cmd(command[1:])

            # State Command Handler Cases
            case 'parameter_command':
                self.temp.data = self.state_cmd_handler.param_cmd(command[1:])

            case 'state_command':
                self.temp.data = self.state_cmd_handler.state_cmd(command[1:])

        # Ensure the endline char at the end of the command
        if self.temp.data[-1] != '\n':
            self.temp.data += '\n'

        # Record the transmitted command
        self.previous_cmd = (
            self.temp.data.split(' ')[0]
            if ' ' in self.temp.data else self.temp.data.split('\n')[0])
        self.get_logger().info(f'Sent message: {self.temp.data}')
        # Send through UART the command
        self.ser.write(self.temp.data.encode('utf-8'))

    def uart_receive(self):
        """Timer callback that reads from UART and handles the response codes and commands."""
        # Read from serial
        line = self.ser.readline().decode('utf-8').rstrip()
        # If a command is read, handle it
        if line:
            self.get_logger().info(f'Received message: {line}')

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message: str):
        """
        Handle the command lines that are received through serial.

        Args:
            message {str}: the command string
        """
        # Record the message
        self.uart_cmd.data = message

        # Extract the command code
        rep_code = (message).split(' ')[0]

        # If a running command has finished OR the response for a request was retrieved
        # OR the sent command was acknowledged by the farmbot
        if (
            (
                self.previous_cmd in self.non_immediate_cmds
                and rep_code in self.non_immediate_cmds[self.previous_cmd]['responses']
            )
            or rep_code == self.non_immediate_cmds['command_echo']
        ):
            # Lower the blocking flag
            self.command_is_finished = True

        # Send the reporting message for further processing by other nodes
        self.fb_feedback_pub.publish(self.uart_cmd)

    # Service Client
    def LED_client(self, led_pin, state):
        """Service client for switching an LED on or off."""
        client = self.create_client(LedPanelHandler, 'set_led')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for LED Handling Server...')

        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = client.call_async(request=request)
        future.add_done_callback(self.LED_panel_callback)

    def LED_panel_callback(self, future):
        """Service client callback once the LED switching server ends."""
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Failure in LED Panel Handling!')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    def destroy_node(self):
        """Close the UART when the node is destroyed."""
        self.ser.close()


def main(args=None):
    """Initialize and run the Serial controller node."""
    rclpy.init(args=args)

    serial_node = SerialController()
    executor = MultiThreadedExecutor()
    executor.add_node(serial_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        serial_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
