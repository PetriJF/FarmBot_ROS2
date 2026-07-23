#!/usr/bin/env python3
"""
Farmbot serial communications node.

Handles ROS2 /farmbot_command input, forwards commands to the Farmduino over serial,
and publishes feedback and busy state updates.
"""
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_hardware_comm.fcode_encoder import EncodeError, Encoder

from farmbot_interfaces.action import HomeAxes, MoveGantry
from farmbot_interfaces.srv import (ConfigurePin, LedPanelHandler, MoveServo,
                                    ReadI2C, ReadParameter, ReadPin, SetI2C,
                                    Watering, WriteParameter, WritePin)

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future

import serial

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger

import yaml


class SerialController(Node):
    """
    Farmbot ROS2 node that handles the Serial messages going to and from the Farmduino.

    The Node receives commands through different actions and services, and sends them to the
    Farmduino.

    When the node receives feedback from the Farmduino through Serial, the message is
    decoded and carried on to the relevant nodes.
    """

    # Node contructor
    def __init__(self):
        """Node Constructor."""
        super().__init__('SerialController')

        # self.uart_cmd = String()
        # self.temp = String()
        self.goal_handle = None

        self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('serial_speed', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('check_uart_freq', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_speed = self.get_parameter('serial_speed').get_parameter_value().integer_value
        self.check_uart_freq = self.get_parameter(
            'check_uart_freq').get_parameter_value().integer_value
        ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        folder_config_name = self.get_parameter(
            'folder_config_name').get_parameter_value().string_value

        config_path = os.path.join(ws_path, folder_config_name)

        # Initializing farmbot encoder module
        self.fcode_encoder = Encoder(config_path)

        # Initialize Serial Communication
        self.ser = serial.Serial(serial_port, serial_speed, timeout=1)
        self.ser.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rx_timer = self.create_timer(1.0 / self.check_uart_freq, self.uart_receive)

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd = ''
        self.code_response: Future = None

        self.mission = {
            'starting_position': [],
            'final_position': [],
        }

        self.directory = os.path.join(
            get_package_share_directory('farmbot_hardware_comm'),
            'config'
        )
        self.non_immediate_cmds = yaml.safe_load(open(os.path.join(self.directory,
                                                                   'CommandsResponses.yaml'), 'r'))

        self.fb_panel = yaml.safe_load(open(os.path.join(self.directory, 'FarmbotPanel.yaml'), 'r'))
        self.led_client = self.create_client(LedPanelHandler, 'set_led')
        # Initialize the LED states
        self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_ON'])
        self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_ON'])

        self.cmd_callback_group = ReentrantCallbackGroup()

        # Initialise device servers
        self.watering_server = self.create_service(Watering, 'watering',
                                                   self.watering_command_server,
                                                   callback_group=self.cmd_callback_group)
        self.read_i2c_server = self.create_service(ReadI2C, 'read_i2c',
                                                   self.read_i2c_command_server,
                                                   callback_group=self.cmd_callback_group)
        self.set_i2c_server = self.create_service(SetI2C, 'set_i2c',
                                                  self.set_i2c_command_server,
                                                  callback_group=self.cmd_callback_group)
        self.configure_pin_server = self.create_service(ConfigurePin, 'configure_pin',
                                                        self.configure_pin_command_server,
                                                        callback_group=self.cmd_callback_group)
        self.read_pin_server = self.create_service(ReadPin, 'read_pin',
                                                   self.read_pin_command_server,
                                                   callback_group=self.cmd_callback_group)
        self.write_pin_server = self.create_service(WritePin, 'write_pin',
                                                    self.write_pin_command_server,
                                                    callback_group=self.cmd_callback_group)

        # Initialise motor servers
        self.move_gantry_server = ActionServer(self, MoveGantry, 'move_gantry',
                                               goal_callback=self.goal_callback,
                                               execute_callback=self.gantry_execute_callback,
                                               callback_group=self.cmd_callback_group)
        self.home_axes_server = ActionServer(self, HomeAxes, 'home_axes',
                                             goal_callback=self.goal_callback,
                                             execute_callback=self.home_execute_callback,
                                             callback_group=self.cmd_callback_group)
        self.move_servo_server = self.create_service(MoveServo, 'move_servo',
                                                     self.move_servo_command_server,
                                                     callback_group=self.cmd_callback_group)

        # Initialise state servers
        self.read_parameter_server = self.create_service(ReadParameter, 'read_parameter',
                                                         self.read_parameter_command_server,
                                                         callback_group=self.cmd_callback_group)
        self.write_parameter_server = self.create_service(WriteParameter, 'write_parameter',
                                                          self.write_parameter_command_server,
                                                          callback_group=self.cmd_callback_group)
        self.list_all_parameter_server = self.create_service(Trigger, 'list_all_parameters',
                                                             self.list_all_command_server,
                                                             callback_group=self.cmd_callback_group)
        self.estop_trigger_server = self.create_service(Trigger, 'estop',
                                                        self.estop_command_server,
                                                        callback_group=self.cmd_callback_group)
        self.resume_trigger_server = self.create_service(Trigger, 'resume',
                                                         self.resume_command_server,
                                                         callback_group=self.cmd_callback_group)
        self.abort_trigger_server = self.create_service(Trigger, 'abort',
                                                        self.abort_command_server,
                                                        callback_group=self.cmd_callback_group)
        self.end_stop_trigger_server = self.create_service(Trigger, 'end_stop',
                                                           self.end_stop_command_server,
                                                           callback_group=self.cmd_callback_group)
        self.sw_version_trigger_server = self.create_service(Trigger, 'sw_version',
                                                             self.sw_version_command_server,
                                                             callback_group=self.cmd_callback_group)
        self.curr_pos_trigger_server = self.create_service(Trigger, 'curr_pos',
                                                           self.curr_position_command_server,
                                                           callback_group=self.cmd_callback_group)

        # Initialise publishers
        self.fb_position = PointStamped()
        self.fb_position.header.stamp = self.get_clock().now().to_msg()
        self.fb_position_pub = self.create_publisher(PointStamped, 'farmbot_position', 10)
        # Latched for late subscriptions of estop/abort state
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop_active = Bool()
        self.estop_active_pub = self.create_publisher(Bool, 'estop_active', latched_qos)
        self.estop_active_pub.publish(self.estop_active)
        self.abort_active = Bool()
        self.abort_active_pub = self.create_publisher(Bool, 'abort_active', latched_qos)
        self.abort_active_pub.publish(self.abort_active)
        self.serial_feedback = String()
        self.serial_feedback_pub = self.create_publisher(String, 'serial_feedback', 10)

        # Log the initialization
        self.get_logger().info('Serial Controller Initialized..')

    async def _run_command(self, fcode: str):
        if self.code_response is not None and not self.code_response.done():
            return False, 'busy', -1

        self.code_response = Future()
        self.farmbot_cmd_sender(fcode)
        result = await self.code_response
        self.code_response = None
        self.previous_cmd = ''
        return result[0], result[1], result[2]

    # Callbacks
    async def watering_command_server(self, request, response):
        """Handle the watering command service request."""
        try:
            fcode = self.fcode_encoder.encode_watering(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_i2c_command_server(self, request, response):
        """Handle the read I2C command service request."""
        response = ReadI2C.Response()
        try:
            fcode = self.fcode_encoder.encode_read_i2c(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def set_i2c_command_server(self, request, response):
        """Handle the read I2C command service request."""
        response = SetI2C.Response()
        try:
            fcode = self.fcode_encoder.encode_set_i2c(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def configure_pin_command_server(self, request, response):
        """Handle the configure pin command service request."""
        response = ConfigurePin.Response()
        try:
            fcode = self.fcode_encoder.encode_configure_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_pin_command_server(self, request, response):
        """Handle the read pin command service request."""
        response = ReadPin.Response()
        try:
            fcode = self.fcode_encoder.encode_read_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def write_pin_command_server(self, request, response):
        """Handle the write pin command service request."""
        response = WritePin.Response()
        try:
            fcode = self.fcode_encoder.encode_write_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    def goal_callback(self, goal_request):
        """
        Check whether the goal is valid.

        If the estop is active, the goal is rejected. Otherwise, it is accepted.
        """
        self.get_logger().info('Received goal request')

        if self.estop_active.data or self.abort_active.data:
            self.get_logger().info('Goal rejected because an estop or abort command is running')
            return GoalResponse.REJECT

        if self.code_response is not None and not self.code_response.done():
            self.get_logger().info('Goal rejected: another command is already in flight')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    async def gantry_execute_callback(self, goal_handle: ServerGoalHandle):
        """Handle the gantry execute callback action."""
        self.goal_handle = goal_handle
        result = MoveGantry.Result()

        try:
            fcode = self.fcode_encoder.encode_move_gantry(goal_handle.request)
            self.mission['starting_position'] = [self.fb_position.point.x,
                                                 self.fb_position.point.y,
                                                 self.fb_position.point.z]
            self.mission['final_position'] = [goal_handle.request.target.x,
                                              goal_handle.request.target.y,
                                              goal_handle.request.target.z]
        except EncodeError as e:
            result.code = MoveGantry.Result.REJECTED
            result.message = str(e)
            goal_handle.abort()
            return result

        response = [0, 0]
        response[0], response[1], _ = await self._run_command(fcode)

        if response[0]:
            result.code = MoveGantry.Result.OK
            result.message = response[1]
            goal_handle.succeed()
        elif response[1] == 'firmware error':
            result.code = MoveGantry.Result.FIRMWARE_ERROR
            result.message = response[1]
        elif response[1] == 'aborted':
            result.code = MoveGantry.Result.ABORTED
            result.message = response[1]
        elif response[1] == 'estopped':
            result.code = MoveGantry.Result.ESTOPPED
            result.message = response[1]
        else:
            result.code = MoveGantry.Result.REJECTED
            result.message = response[1]

        if not response[0]:
            goal_handle.abort()

        return result

    async def home_execute_callback(self, goal_handle: ServerGoalHandle):
        """Handle the home execute callback action."""
        self.goal_handle = goal_handle
        result = HomeAxes.Result()

        try:
            fcode = self.fcode_encoder.encode_home_axes(goal_handle.request)
        except EncodeError as e:
            result.code = HomeAxes.Result.REJECTED
            result.message = str(e)
            goal_handle.abort()
            return result

        response = [0, 0]
        response[0], response[1], _ = await self._run_command(fcode)

        if response[0]:
            result.code = HomeAxes.Result.OK
            result.message = response[1]
            goal_handle.succeed()
        elif response[1] == 'firmware error':
            result.code = HomeAxes.Result.FIRMWARE_ERROR
            result.message = response[1]
        elif response[1] == 'aborted':
            result.code = HomeAxes.Result.ABORTED
            result.message = response[1]
        elif response[1] == 'estopped':
            result.code = HomeAxes.Result.ESTOPPED
            result.message = response[1]
        else:
            result.code = HomeAxes.Result.REJECTED
            result.message = response[1]

        if not response[0]:
            goal_handle.abort()

        return result

    async def move_servo_command_server(self, request, response):
        """Handle the move servo command service request."""
        try:
            fcode = self.fcode_encoder.encode_move_servo(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_parameter_command_server(self, request, response):
        """Handle the read parameter command service request."""
        response = ReadParameter.Response()
        try:
            fcode = self.fcode_encoder.encode_read_parameter(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def write_parameter_command_server(self, request, response):
        """Handle the write parameter command service request."""
        response = WriteParameter.Response()
        try:
            fcode = self.fcode_encoder.encode_write_parameter(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def list_all_command_server(self, request, response):
        """Handle the list all parameter command service request."""
        response = Trigger.Response()

        response.success, response.message, _ = await self._run_command('F20')
        return response

    async def estop_command_server(self, request, response):
        """Handle the estop command service request."""
        response = Trigger.Response()

        self.farmbot_cmd_sender('E')

        self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_OFF'])
        self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_FLASHING'])
        self.estop_active.data = True
        self.estop_active_pub.publish(self.estop_active)

        response.success = True
        return response

    async def resume_command_server(self, request, response):
        """Handle the reset estop command service request."""
        response = Trigger.Response()

        response.success, response.message, _ = await self._run_command('F09')

        if response.success:
            self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_ON'])
            self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_ON'])
            self.estop_active.data = False
            self.estop_active_pub.publish(self.estop_active)

        return response

    async def abort_command_server(self, request, response):
        """Handle the abort command service request."""
        response = Trigger.Response()

        self.farmbot_cmd_sender('@')
        # Only latch when a command was actually interrupted (or unlatch a prior abort)
        if ((self.code_response is not None and not self.code_response.done())
                or self.abort_active.data):
            self.abort_active.data = not self.abort_active.data
            self.abort_active_pub.publish(self.abort_active)

        response.success = True
        return response

    async def end_stop_command_server(self, request, response):
        """Handle the end stops command service request."""
        response = Trigger.Response()

        response.success, response.message, _ = await self._run_command('F81')
        return response

    async def sw_version_command_server(self, request, response):
        """Handle the software version command service request."""
        response = Trigger.Response()

        response.success, response.message, _ = await self._run_command('F83')
        return response

    async def curr_position_command_server(self, request, response):
        """Handle the software version command service request."""
        response = Trigger.Response()

        response.success, response.message, _ = await self._run_command('F82')
        return response

    def farmbot_cmd_sender(self, cmd: str):
        """Send the commands to the farmbot through serial."""
        # Ensure the endline char at the end of the command
        if cmd[-1] != '\n':
            cmd += '\n'

        # Record the transmitted command
        self.previous_cmd = (cmd.split(' ')[0] if ' ' in cmd else cmd.split('\n')[0])

        self.get_logger().info(f'Sent message: {cmd}')
        #  Send through serial the command
        self.ser.write(cmd.encode('utf-8'))

    # Receiving messages from Farmbot
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
        self.serial_feedback.data = message

        # Extract the command code
        rep_code = (message).split(' ')[0]

        if rep_code == 'R82':
            code_position = (message).split(' ')
            self.fb_position.point.x = float(code_position[1][1:])
            self.fb_position.point.y = float(code_position[2][1:])
            self.fb_position.point.z = float(code_position[3][1:])
            self.fb_position.header.stamp = self.get_clock().now().to_msg()
            self.fb_position_pub.publish(self.fb_position)

        # E-Stop pressed on the robot
        if rep_code == 'R87' and not self.estop_active.data:
            self.estop_active.data = True
            self.estop_active_pub.publish(self.estop_active)

        # If a running command has finished OR the response for a request was retrieved
        # OR the sent command was acknowledged by the farmbot
        command_type = ''
        for cmd_type in self.non_immediate_cmds:
            if self.previous_cmd in self.non_immediate_cmds[cmd_type]:
                command_type = cmd_type
                break

        if self.goal_handle is None or not self.goal_handle.is_active:
            pass

        elif command_type == 'home_axes':
            feedback = HomeAxes.Feedback()
            feedback.position.x = self.fb_position.point.x
            feedback.position.y = self.fb_position.point.y
            feedback.position.z = self.fb_position.point.z
            self.goal_handle.publish_feedback(feedback)

        elif command_type == 'move_gantry':
            feedback = MoveGantry.Feedback()
            feedback.position.x = self.fb_position.point.x
            feedback.position.y = self.fb_position.point.y
            feedback.position.z = self.fb_position.point.z
            feedback.progress = float(self.percentage_calculation([feedback.position.x,
                                                                  feedback.position.y,
                                                                  feedback.position.z]))
            self.goal_handle.publish_feedback(feedback)

        if (command_type
           and rep_code in self.non_immediate_cmds[command_type][self.previous_cmd]['responses']
           and self.code_response and not self.code_response.done()):
            match rep_code:
                case 'R03':
                    result = [False, 'firmware error', -1]
                    self.code_response.set_result(result)
                case 'R86':
                    result = [False, 'aborted', -1]
                    self.code_response.set_result(result)
                case 'R02':
                    result = [True, '', -1]
                    self.code_response.set_result(result)
                case 'R87':
                    result = [False, 'estopped', -1]
                    self.code_response.set_result(result)
                case 'R08':
                    result = [True, '', -1]
                    self.code_response.set_result(result)
                case 'R41' | 'R21':
                    code = (message).split(' ')
                    value = int(code[2][1:])
                    result = [True, '', value]
                    self.code_response.set_result(result)
                case 'R81' | 'R82' | 'R83':
                    result = [True, message[4:], -1]
                    self.code_response.set_result(result)

        # Send the reporting message for further processing by other nodes
        self.serial_feedback_pub.publish(self.serial_feedback)

    def percentage_calculation(self, current_position: list):
        """Calculate the percentage of progress made in the movement."""
        axis_completion = [0.0, 0.0, 0.0]
        denominator = 0

        for i in range(3):
            if self.mission['final_position'][i] != self.mission['starting_position'][i]:
                axis_completion[i] = ((current_position[i]
                                       - self.mission['starting_position'][i]) /
                                      (self.mission['final_position'][i] -
                                       self.mission['starting_position'][i]))
                denominator += 1
        if denominator != 0:
            return sum(axis_completion) / denominator
        return 1

    # Service Client

    def LED_client(self, led_pin, state):
        """Service client for switching an LED on or off."""
        # Never block the executor waiting for the LED server (estop path!)
        if not self.led_client.service_is_ready():
            self.get_logger().warn('LED Handling Server not available!')
            return

        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = self.led_client.call_async(request=request)
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

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
