#!/usr/bin/env python3
"""
Farmbot serial communications node.

Handles ROS2 /farmbot_command input, forwards commands to the Farmduino over serial,
and publishes feedback and busy state updates.
"""
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_hardware_comm.fcode_encoder import EncodeError, Encoder

# from farmbot_interfaces.action import FarmbotComms
from farmbot_interfaces.action import HomeAxes, MoveGantry
from farmbot_interfaces.srv import (ConfigurePin, LedPanelHandler, MoveServo,
                                    ReadI2C, ReadParameter, ReadPin, SetI2C,
                                    Watering, WriteParameter, WritePin)

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.action import ActionServer, GoalResponse
# from rclpy.action import CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

# import serial

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger

import yaml


class SerialController(Node):
    """
    Farmbot ROS2 node that handles the Serial messages going to and from the Farmduino.

    The Node receives commands through the FarmbotComms action and sends them to the
    Farmduino.

    When the node receives feedback from the Farmduino through Serial, the message is
    decoded and carried on to the relevant nodes.
    """

    # Node contructor
    def __init__(self):
        """Node Constructor."""
        super().__init__('SerialController')

        # # self.uart_cmd = String()
        # # self.temp = String()
        self.goal_handle = None

        # self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        # self.declare_parameter('serial_speed', rclpy.Parameter.Type.INTEGER)
        # self.declare_parameter('check_uart_freq', rclpy.Parameter.Type.INTEGER)
        # self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        # self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)

        # serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        # serial_speed = self.get_parameter('serial_speed').get_parameter_value().integer_value
        # self.check_uart_freq = self.get_parameter(
        #     'check_uart_freq').get_parameter_value().integer_value
        # ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        # folder_config_name = self.get_parameter(
        #     'folder_config_name').get_parameter_value().string_value

        # config_path = os.path.join(ws_path, folder_config_name)

        # Initializing farmbot encoder module
        # self.fcode_encoder = Encoder(config_path)
        self.fcode_encoder = Encoder()

        # # # UART receive publisher
        # # self.fb_feedback_pub = self.create_publisher(String, 'farmbot_feedback', 10)

        # # # Initialize the Action Server
        # # self.farmbot_comm_server = ActionServer(
        # #     self,
        # #     FarmbotComms,
        # #     'farmbot_communication',
        # #     goal_callback=self.communication_goal_callback,
        # #     execute_callback=self.execute_callback,
        # #     cancel_callback=self.cancel_callback,
        # #     handle_accepted_callback=self.handle_callback,
        # # )

        # # Initialize Serial Communication
        # self.ser = serial.Serial(serial_port, serial_speed, timeout=1)
        # self.ser.reset_input_buffer()
        # # Create a timer to periodically check for incoming serial messages
        # self.rx_timer = self.create_timer(1.0 / self.check_uart_freq, self.uart_receive)

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd = ''
        # # self.status = ''

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
        # Initialize the LED states
        self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_ON'])
        self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_ON'])

        # Initialise device servers
        self.watering_server = self.create_service(Watering, 'watering',
                                                   self.watering_command_server)
        self.read_i2c_server = self.create_service(ReadI2C, 'read_i2c',
                                                   self.read_i2c_command_server)
        self.set_i2c_server = self.create_service(SetI2C, 'set_i2c',
                                                  self.set_i2c_command_server)
        self.configure_pin_server = self.create_service(ConfigurePin, 'configure_pin',
                                                        self.configure_pin_command_server)
        self.read_pin_server = self.create_service(ReadPin, 'read_pin',
                                                   self.read_pin_command_server)
        self.write_pin_server = self.create_service(WritePin, 'write_pin',
                                                    self.write_pin_command_server)

        # Initialise motor servers
        self.move_gantry_server = ActionServer(self, MoveGantry, 'move_gantry',
                                               goal_callback=self.goal_callback,
                                               execute_callback=self.gantry_execute_callback,
                                               handle_accepted_callback=self.handle_gantry_callback)
        self.home_axes_server = ActionServer(self, HomeAxes, 'home_axes',
                                             goal_callback=self.goal_callback,
                                             execute_callback=self.home_execute_callback,
                                             handle_accepted_callback=self.handle_home_callback)
        self.move_servo_server = self.create_service(MoveServo, 'move_servo',
                                                     self.move_servo_command_server)

        # Initialise state servers
        self.read_parameter_server = self.create_service(ReadParameter, 'read_parameter',
                                                         self.read_parameter_command_server)
        self.write_parameter_server = self.create_service(WriteParameter, 'write_parameter',
                                                          self.write_parameter_command_server)
        self.list_all_parameters_server = self.create_service(Trigger, 'list_all_parameters',
                                                              self.list_all_command_server)
        self.estop_trigger_server = self.create_service(Trigger, 'estop',
                                                        self.estop_command_server)
        self.resume_trigger_server = self.create_service(Trigger, 'resume',
                                                         self.resume_command_server)
        self.abort_trigger_server = self.create_service(Trigger, 'abort',
                                                        self.abort_command_server)

        # Initialise publishers
        self.fb_position = PointStamped()
        self.fb_position.header.stamp = self.get_clock().now().to_msg()
        self.fb_position_pub = self.create_publisher(PointStamped, 'farmbot_position', 10)
        self.estop_active = Bool()
        self.estop_active_pub = self.create_publisher(Bool, 'estop_active', 10)
        self.abort_active = Bool()
        self.abort_active_pub = self.create_publisher(Bool, 'abort_active', 10)
        self.serial_feedback = String()
        self.serial_feedback_pub = self.create_publisher(String, 'serial_feedback', 10)

        self.fake_serial_sub = self.create_subscription(String,
                                                        'test', self.uart_receive, 10)

        # Log the initialization
        self.get_logger().info('Serial Controller Initialized..')

    # Callbacks
    def watering_command_server(self, request, response):
        """Handle the watering command service request."""
        try:
            fcode = self.fcode_encoder.encode_watering(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

    def read_i2c_command_server(self, request, response):
        """Handle the read I2C command service request."""
        try:
            fcode = self.fcode_encoder.encode_read_i2c(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def set_i2c_command_server(self, request, response):
        """Handle the read I2C command service request."""
        try:
            fcode = self.fcode_encoder.encode_set_i2c(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def configure_pin_command_server(self, request, response):
        """Handle the configure pin command service request."""
        try:
            fcode = self.fcode_encoder.encode_configure_pin(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def read_pin_command_server(self, request, response):
        """Handle the read pin command service request."""
        try:
            fcode = self.fcode_encoder.encode_read_pin(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def write_pin_command_server(self, request, response):
        """Handle the write pin command service request."""
        try:
            fcode = self.fcode_encoder.encode_write_pin(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
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

        return GoalResponse.ACCEPT

    def handle_gantry_callback(self, goal_handle: ServerGoalHandle):
        """Handle the gantry execute callback action."""
        self.goal_handle = goal_handle

        try:
            fcode = self.fcode_encoder.encode_move_gantry(goal_handle.request)
            self.mission['starting_position'] += ([self.fb_position.point.x]
                                                  + [self.fb_position.point.y]
                                                  + [self.fb_position.point.z])
            self.mission['final_position'] += ([goal_handle.request.target.x]
                                               + [goal_handle.request.target.y]
                                               + [goal_handle.request.target.z])
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            result = MoveGantry.Result()
            result.code = MoveGantry.REJECTED
            result.message = str(e)
            return result

    def handle_home_callback(self, goal_handle: ServerGoalHandle):
        """Handle the home execute callback action."""
        self.goal_handle = goal_handle

        try:
            fcode = self.fcode_encoder.encode_home_axes(goal_handle.request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            result = HomeAxes.Result()
            result.code = 4
            result.message = str(e)

    def move_servo_command_server(self, request, response):
        """Handle the move servo command service request."""
        try:
            fcode = self.fcode_encoder.encode_move_servo(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def read_parameter_command_server(self, request, response):
        """Handle the read parameter command service request."""
        try:
            fcode = self.fcode_encoder.encode_read_parameter(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def write_parameter_command_server(self, request, response):
        """Handle the write parameter command service request."""
        try:
            fcode = self.fcode_encoder.encode_write_parameter(request)
            self.farmbot_cmd_sender(fcode)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
        return response

    def list_all_command_server(self, request, response):
        """Handle the list all parameter command service request."""
        self.farmbot_cmd_sender('F20')
        response.success = True
        return response

    def estop_command_server(self, request, response):
        """Handle the estop command service request."""
        self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_OFF'])
        self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_FLASHING'])
        self.farmbot_cmd_sender('E')

        self.estop_active.data = True
        self.estop_active_pub.publish(self.estop_active)

        response.success = True
        return response

    def resume_command_server(self, request, response):
        """Handle the reset estop command service request."""
        self.LED_client(self.fb_panel['ESTOP_LED'], self.fb_panel['LED_ON'])
        self.LED_client(self.fb_panel['UNLOCK_LED'], self.fb_panel['LED_ON'])
        self.farmbot_cmd_sender('F09')

        self.estop_active.data = False
        self.estop_active_pub.publish(self.estop_active)

        response.success = True
        return response

    def abort_command_server(self, request, response):
        """Handle the abort command service request."""
        self.farmbot_cmd_sender('@')

        if not self.abort_active:
            self.abort_active.data = True
        else:
            self.abort_active.data = False
        self.abort_active_pub.publish(self.abort_active)

        response.success = True
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
        # self.ser.write(cmd.encode('utf-8'))

    # Receiving messages from Farmbot
    def uart_receive(self, cmd: String):
        """Timer callback that reads from UART and handles the response codes and commands."""
        # # Read from serial
        # line = self.ser.readline().decode('utf-8').rstrip()
        line = cmd.data
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

        self.fb_response = ''
        match rep_code:
            case 'R82':
                code_position = (message).split(' ')
                self.fb_position.point.x = float(code_position[1][1:])
                self.fb_position.point.y = float(code_position[2][1:])
                self.fb_position.point.z = float(code_position[3][1:])
                self.fb_position_pub.publish(self.fb_position)
            case 'R03':
                self.fb_response = 'FIRMWARE_ERROR'
            case 'R86':
                self.fb_response = 'ABORTED'
            case 'R02':
                self.fb_response = 'OK'
            case 'R87':
                self.fb_response = 'ESTOPPED'
            case 'R08':
                self.fb_response = 'ECHO'
            case 'R41' | 'R21':
                code = (message).split(' ')
                self.fb_response = code[2][1:]

        # If a running command has finished OR the response for a request was retrieved
        # OR the sent command was acknowledged by the farmbot
        command_type = ''
        for cmd_type in self.non_immediate_cmds:
            if self.previous_cmd in self.non_immediate_cmds[cmd_type]:
                command_type = cmd_type
                break

        if command_type == 'home_axes':
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

        # Send the reporting message for further processing by other nodes
        self.serial_feedback_pub.publish(self.serial_feedback)
        if self.fb_response != '':
            self.response_handler(command_type, rep_code)

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

    def response_handler(self, cmd_type: str, rep_code: str):
        """Dispatch a command response to the appropriate sender."""
        if rep_code in self.non_immediate_cmds[cmd_type][self.previous_cmd]['responses']:
            match cmd_type:
                case 'watering':
                    self.watering_response()
                case 'read_i2c':
                    self.read_i2c_response()
                case 'set_i2c':
                    self.set_i2c_response()
                case 'configure_pin':
                    self.configure_pin_response()
                case 'read_pin':
                    self.read_pin_response()
                case 'write_pin':
                    self.write_pin_response()
                case 'move_gantry':
                    self.gantry_execute_callback()
                case 'home_axes':
                    self.home_execute_callback()
                case 'move_servo':
                    self.move_servo_response()
                case 'read_parameter':
                    self.read_parameter_response()
                case 'write_parameter':
                    self.write_parameter_response()
                case 'trigger_command':
                    return
                case _:
                    self.get_logger().info(f'{cmd_type}: This type of command is not supported by '
                                           'the response handler')
                    return

    def watering_response(self):
        """Create the standard response for the watering server."""
        response = Watering.Response()
        response.success = True
        response.message = ''
        return response

    def read_i2c_response(self):
        """Create the standard response for the read_i2c server."""
        response = ReadI2C.Response()
        if self.fb_response not in ['ESTOPPED', 'ABORTED', 'ECHO', 'OK', 'FIRMWARE_ERROR']:
            response.success = True
            response.value = int(self.fb_response)
            response.message = ''
        else:
            response.success = False
            response.message = self.fb_response
        return response

    def set_i2c_response(self):
        """Create the standard response for the set_i2c server."""
        response = SetI2C.Response()
        if self.fb_response == 'OK':
            response.success = True
            response.message = ''
        else:
            response.success = False
            response.message = self.fb_response
        return response

    def configure_pin_response(self):
        """Create the standard response for the configure_pin server."""
        response = ConfigurePin.Response()
        response.success = True
        response.message = ''
        return response

    def read_pin_response(self):
        """Create the standard response for the read_pin server."""
        response = ReadPin.Response()
        if self.fb_response not in ['ESTOPPED', 'ABORTED', 'ECHO', 'OK', 'FIRMWARE_ERROR']:
            response.success = True
            response.value = int(self.fb_response)
            response.message = ''
        else:
            response.success = False
            response.message = self.fb_response
        return response

    def write_pin_response(self):
        """Create the standard response for the write_pin server."""
        response = WritePin.Response()
        if self.previous_cmd == 'F44':
            if self.response == 'OK':
                response.success = True
                response.message = ''
            else:
                response.success = False
                response.message = self.response
            return response
        else:
            response.success = True
            response.message = ''
            return response

    def gantry_execute_callback(self):
        """Create the standard response for the move_gantry action server."""
        result = MoveGantry.Result()
        if self.fb_response == 'OK':
            result.code = MoveGantry.OK
            result.message = ''
        elif self.fb_response == 'FIRMWARE_ERROR':
            result.code = MoveGantry.FIRMWARE_ERROR
            result.message = self.fb_response
        elif self.fb_response == 'ABORTED':
            result.code = MoveGantry.ABORTED
            result.message = self.fb_response
        elif self.fb_response == 'ESTOPPED':
            result.code = MoveGantry.ESTOPPED
            result.message = self.fb_response
        return result

    def home_execute_callback(self):
        """Create the standard response for the home_axes action server."""
        result = HomeAxes.Result()
        if self.fb_response == 'OK':
            result.code = HomeAxes.OK
            result.message = ''
        elif self.fb_response == 'FIRMWARE_ERROR':
            result.code = HomeAxes.FIRMWARE_ERROR
            result.message = self.fb_response
        elif self.fb_response == 'ABORTED':
            result.code = HomeAxes.ABORTED
            result.message = self.fb_response
        elif self.fb_response == 'ESTOPPED':
            result.code = HomeAxes.ESTOPPED
            result.message = self.fb_response
        return result

    def move_servo_response(self):
        """Create the standard response for the move_servo server."""
        response = MoveServo.Response()
        response.success = True
        response.message = ''
        return response

    def read_parameter_response(self):
        """Create the standard response for the read_parameter server."""
        response = ReadParameter.Response()
        if self.fb_response not in ['ESTOPPED', 'ABORTED', 'ECHO', 'OK', 'FIRMWARE_ERROR']:
            response.success = True
            response.value = int(self.fb_response)
            response.message = ''
        else:
            response.success = False
            response.message = self.fb_response
        return response

    def write_parameter_response(self):
        """Create the standard response for the write_parameter server."""
        response = WriteParameter.Response()
        response.success = True
        response.message = ''
        return response

####################################################################################################

    # Goal Callback
    # def communication_goal_callback(self, goal_request):
    #     """
    #     Check whether the goal is valid.

    #     If the command type is not among those accepted by Farmbot, the goal is rejected.
    #     Otherwise, it is accepted.
    #     """
    #     self.get_logger().info('Received goal request')
    #     command_id = (goal_request.command).split(' ')[0]

    #     valid_commands = ['E', 'F09', '@', 'i2c_command', 'pin_command', 'water_command',
    #                       'home_handler', 'move_gantry', 'move_servo', 'parameter_command',
    #                       'state_command']

    #     if (command_id not in valid_commands
    #        or (self.previous_cmd == 'E' and command_id != 'F09')
    #        or (self.previous_cmd == '@' and command_id not in ['E', '@'])
    #        or (self.status == 'IS_RUNNING')):
    #         return GoalResponse.REJECT

    #     return GoalResponse.ACCEPT

    # # Cancel callback
    # def cancel_callback(self, goal_handle):
    #     """Accept the cancel request."""
    #     self.get_logger().info('Received cancel request')

    #     return CancelResponse.ACCEPT

    # Handle execute callback
    # def handle_callback(self, goal_handle: ServerGoalHandle):
    #     """Create a timer to track the command status."""
    #     self.goal_handle = goal_handle
    #     command = goal_handle.request.command

    #     if self.previous_cmd == '@' and command == '@':
    #         self.status = 'ABORT_ENDED'
    #         self.previous_cmd = ''
    #         goal_handle.execute()
    #         return

    #     self.mission['starting_position'] = self.mission['current_position'][:]

    #     self.farmbot_command_sender(command)

    #     self.get_logger().info('Executing goal...')
    #     self.check_status_timer = self.create_timer(1.0 / self.check_uart_freq, self.check_status)
    #     self.feedback_timer = self.create_timer(1.0 / 2, self.send_feedback)

    # def check_status(self):
    #     """
    #     Check command execution status and handle completion or cancellation.

    #     Publishes feedback, handles goal cancellation requests, and triggers
    #     the next callback based on command completion state.
    #     """
    #     if not self.check_status_timer.is_canceled():
    #         if self.goal_handle.is_cancel_requested:
    #             self.get_logger().info('Goal canceled.')
    #             self.status = 'CANCELED'
    #             self.goal_handle.execute()

    #         elif self.status == 'ABORTED':
    #             self.get_logger().warn('Goal aborted.')
    #             self.goal_handle.execute()

    #         elif self.status == 'ERROR':
    #             self.get_logger().warn('Goal finished with error.')
    #             self.goal_handle.execute()

    #         elif self.status == 'SUCCEED':
    #             self.get_logger().info('Goal completed.')
    #             self.goal_handle.execute()

    # def send_feedback(self):
    #     """Publish the feedback with the current position and the completion percentage."""
    #     # Create Feedback object
    #     feedback = FarmbotComms.Feedback()

    #     if self.previous_cmd in self.non_immediate_cmds['long_term']:
    #         self.mission['completion'] = float(self.percentage_calculation())

    #     feedback.current_position = self.mission['current_position']
    #     feedback.percentage = self.mission['completion']

    #     self.goal_handle.publish_feedback(feedback)

    # Execute callback
    # def execute_callback(self, goal_handle: ServerGoalHandle):
    #     """
    #     Execute the FarmbotComms action goal.

    #     Checks if the command has finished executing. If so, cancels the
    #     status timer and returns the result. Otherwise, sends the command
    #     to the Farmduino and logs execution status.
    #     """
    #     self.mission['starting_position'] = []
    #     self.mission['final_position'] = []
    #     self.mission['completion'] = 0.0
    #     self.check_status_timer.cancel()
    #     self.feedback_timer.cancel()

    #     # Create Result object
    #     result = FarmbotComms.Result()

    #     if self.status == 'CANCELED':
    #         goal_handle.canceled()
    #         result.status = self.status
    #         self.status = ''
    #         return result

    #     elif self.status == 'ABORTED':
    #         goal_handle.abort()
    #         result.status = self.status
    #         self.status = ''
    #         return result

    #     else:
    #         goal_handle.succeed()
    #         result.status = self.status
    #         self.status = ''
    #         return result

    # def farmbot_command_sender(self, cmd: str):
    #     """
    #     Handle the commands that are queued to be sent to the farmbot through UART.

    #     Two cases:
    #         a) the command has priority (e.g. electronic-stop):
    #             The command bypasses the queue and the queue is reset
    #         b) standard command:
    #             The command is added at the end of the queue
    #     """
    #     command: list = cmd.split(' ')

    #     match command[0]:

    #         # Device Command Handler Cases
    #         case 'E':
    #             self.temp.data = 'E'

    #         case 'F09':
    #             self.temp.data = 'F09'

    #         case '@':
    #             self.temp.data = '@'

    #         case 'i2c_command':
    #             self.temp.data = self.device_cmd_handler.i2c_cmd(command[1:])

    #         case 'pin_command':
    #             self.temp.data = self.device_cmd_handler.pin_cmd(command[1:])

    #         case 'water_command':
    #             self.temp.data = self.device_cmd_handler.water_cmd(command[1:])

    #         # Motor Command Handler Cases
    #         case 'home_handler':
    #             self.temp.data = self.motor_cmd_handler.home_cmd(command[1:])

    #         case 'move_gantry':
    #             self.temp.data = self.motor_cmd_handler.gantry_cmd(command[1:])

    #         case 'move_servo':
    #             self.temp.data = self.motor_cmd_handler.servo_cmd(command[1:])

    #         # State Command Handler Cases
    #         case 'parameter_command':
    #             self.temp.data = self.state_cmd_handler.param_cmd(command[1:])

    #         case 'state_command':
    #             self.temp.data = self.state_cmd_handler.state_cmd(command[1:])

    #         case _:
    #             self.get_logger().warn(f'This command type is not recognized {cmd}'
    #                                    "Ensure you don't have a typo!")
    #             self.status = 'ERROR'
    #             return

    #     if not self.temp.data:
    #         self.status = 'ERROR'
    #         return

    #     self.status = 'IS_RUNNING'
    #     self.find_final_position(self.temp.data)

    #     # Ensure the endline char at the end of the command
    #     if self.temp.data[-1] != '\n':
    #         self.temp.data += '\n'

    #     # Record the transmitted command
    #     self.previous_cmd = (
    #         self.temp.data.split(' ')[0]
    #         if ' ' in self.temp.data else self.temp.data.split('\n')[0])

    #     self.get_logger().info(f'Sent message: {self.temp.data}')
    #     #  Send through UART the command
    #     self.ser.write(self.temp.data.encode('utf-8'))

    # def find_final_position(self, cmd: str):
    #     """Obtain the final position for motion command."""
    #     self.mission['final_position'] = []

    #     goal = cmd.split(' ')
    #     match goal[0]:
    #         case 'G00' | 'G01':
    #             self.mission['final_position'] = [float(goal[1][1:]), float(goal[2][1:]),
    #                                               float(goal[3][1:])]
    #         case 'G28':
    #             self.mission['final_position'] = [0.0, 0.0, 0.0]
    #         case 'F11':
    #             self.mission['final_position'] = [0.0] + self.mission['starting_position'][1:]
    #         case 'F12':
    #             self.mission['final_position'] = [self.mission['starting_position'][0], 0.0,
    #                                               self.mission['starting_position'][2]]
    #         case 'F13':
    #             self.mission['final_position'] = self.mission['starting_position'][:2] + [0.0]
####################################################################################################

    # Service Client

    def LED_client(self, led_pin, state):
        """Service client for switching an LED on or off."""
        client = self.create_client(LedPanelHandler, 'set_led')
        delay = 0

        while not client.wait_for_service(1.0):
            delay += 1
            self.get_logger().warn('Waiting for LED Handling Server...')
            if delay >= 5:
                self.get_logger().error('LED Handling Server not available!')
                return

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

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
