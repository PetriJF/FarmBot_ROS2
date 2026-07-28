#!/usr/bin/env python3
"""
FarmBot serial communication node.

This node acts as the communication bridge between ROS 2 and the FarmBot
controller. It handles service and action requests, converts commands into
FCode, sends them to the Farmduino over the serial connection, and processes
received messages to provide feedback and update the system state.
"""
from farmbot_hardware_comm.config_managers import ConfigServer
from farmbot_hardware_comm.fcode_encoder import Encoder
from farmbot_hardware_comm.modules.exceptions import EncodeError, ServerError, YAMLError
from farmbot_hardware_comm.modules.yaml_handler import YAMLHandler

from farmbot_interfaces.action import HomeAxes, LoadingParameters, MoveGantry
from farmbot_interfaces.srv import (ConfigurePin, LedPanelHandler, MoveServo,
                                    ReadI2C, ReadParameter, ReadPin, SetI2C,
                                    Watering, WriteParameter, WritePin)

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future

import serial

from std_msgs.msg import Bool, String

from std_srvs.srv import Trigger


class SerialController(Node):
    """
    Farmbot ROS2 node that handles the Serial messages going to and from the Farmduino.

    The Node receives commands through different actions and services, and sends them to the
    Farmduino.

    When the node receives feedback from the Farmduino through Serial, the message is
    decoded and carried on to the relevant nodes.
    """

    # Node constructor
    def __init__(self):
        """Node Constructor."""
        super().__init__('SerialController')

        self.goal_handle = None

        self.declare_parameter('serial_port', rclpy.Parameter.Type.STRING)
        self.declare_parameter('serial_speed', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('check_serial_freq', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_speed = self.get_parameter('serial_speed').get_parameter_value().integer_value
        self.check_serial_freq = self.get_parameter(
            'check_serial_freq').get_parameter_value().integer_value
        ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        folder_config_name = self.get_parameter(
            'folder_config_name').get_parameter_value().string_value

        # Initialise Serial Communication
        self.ser = serial.Serial(serial_port, serial_speed, timeout=1)
        self.ser.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rx_timer = self.create_timer(1.0 / self.check_serial_freq, self.serial_receive)

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd = ''
        self.code_response: Future = None

        self.mission = {
            'starting_position': [],
            'final_position': [],
        }

        config_path = YAMLHandler.join_path(ws_path, folder_config_name)
        YAMLHandler.make_dir(config_path)

        # Initialising modules
        self.config_server = ConfigServer(self, config_path)
        self.fcode_encoder = Encoder(config_path)

        self.directory = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')

        try:
            self.non_immediate_cmds = YAMLHandler.load_yaml(self.directory,
                                                            'CommandsResponses.yaml')
            self.fb_panel = YAMLHandler.load_yaml(self.directory, 'FarmbotPanel.yaml')
        except YAMLError as e:
            self.get_logger().warn(f'yaml error: {e}')
            return

        self.led_client = self.create_client(LedPanelHandler, 'set_led')
        # Initialise the LED states
        self.switch_led(self.fb_panel['estop_led'], self.fb_panel['led_on'])
        self.switch_led(self.fb_panel['unlock_led'], self.fb_panel['led_on'])

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
        self.load_params_server = ActionServer(self, LoadingParameters, 'loading_params',
                                               goal_callback=self.goal_callback,
                                               execute_callback=self.load_params_execute_callback,
                                               handle_accepted_callback=self.handle_load_callback,
                                               callback_group=self.cmd_callback_group)
        self.result_loading = LoadingParameters.Result()
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

        # Initialise WriteParameter client
        self.write_param_client = self.create_client(WriteParameter, 'write_parameter')

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

        # Log the Initialisation
        self.get_logger().info('Serial Controller Initialised..')

    async def _run_command(self, fcode: str) -> tuple[bool, str, int]:
        if self.code_response is not None and not self.code_response.done():
            return False, 'busy', -1

        self.code_response = Future()
        self.farmbot_cmd_sender(fcode)
        result = await self.code_response
        self.code_response = None
        self.previous_cmd = ''
        return result[0], result[1], result[2]

    # Callbacks
    async def watering_command_server(self, request: Watering.Request,
                                      response: Watering.Response) -> Watering.Response:
        """
        Handle a watering command service request.

        Encodes the watering request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {Watering.Request}: Watering command parameters.
            response {Watering.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_watering(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_i2c_command_server(self, request: ReadI2C.Request,
                                      response: ReadI2C.Response) -> ReadI2C.Response:
        """
        Handle a i2c reading command service request.

        Encodes the i2c reading request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {ReadI2C.Request}: ReadI2C command parameters.
            response {ReadI2C.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_read_i2c(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def set_i2c_command_server(self, request: SetI2C.Request,
                                     response: SetI2C.Response) -> SetI2C.Response:
        """
        Handle a i2c setting command service request.

        Encodes the i2c setting request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {SetI2C.Request}: SetI2C command parameters.
            response {SetI2C.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_set_i2c(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def configure_pin_command_server(
            self, request: ConfigurePin.Request,
            response: ConfigurePin.Response) -> ConfigurePin.Response:
        """
        Handle a configure pin command service request.

        Encodes the configure pin request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {ConfigurePin.Request}: ConfigurePin command parameters.
            response {ConfigurePin.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_configure_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_pin_command_server(self, request: ReadPin.Request,
                                      response: ReadPin.Response) -> ReadPin.Response:
        """
        Handle a read pin command service request.

        Encodes the read pin request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {ReadPin.Request}: ReadPin command parameters.
            response {ReadPin.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_read_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def write_pin_command_server(self, request: WritePin.Request,
                                       response: WritePin.Response) -> WritePin.Response:
        """
        Handle a write pin command service request.

        Encodes the write pin request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {WritePin.Request}: WritePin command parameters.
            response {WritePin.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_write_pin(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    def goal_callback(self, goal_request) -> GoalResponse:
        """
        Check whether an incoming action goal can be accepted.

        A goal is rejected if an emergency stop or abort command is active,
        or if another command is already being executed.
        Otherwise, the goal is accepted.

        Args:
        goal_request: Incoming action goal request.
        """
        self.get_logger().info('Received goal request')

        if self.estop_active.data or self.abort_active.data:
            self.get_logger().info('Goal rejected because an estop or abort command is running')
            return GoalResponse.REJECT

        if self.code_response is not None and not self.code_response.done():
            self.get_logger().info('Goal rejected: another command is already in flight')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    async def gantry_execute_callback(self, goal_handle: ServerGoalHandle) -> MoveGantry.Result:
        """
        Execute a gantry movement action.

        The goal is encoded into FCode and sent to the FarmBot.
        The action result is updated according to the command execution status.

        Args:
            goal_handle {ServerGoalHandle}: Accepted action goal.
        """
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

    async def home_execute_callback(self, goal_handle: ServerGoalHandle) -> HomeAxes.Result:
        """
        Execute a home axes action.

        The goal is encoded into FCode and sent to the FarmBot.
        The action result is updated according to the command execution status.

        Args:
            goal_handle {ServerGoalHandle}: Accepted action goal.
        """
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

    async def move_servo_command_server(self, request: MoveServo.Request,
                                        response: MoveServo.Response) -> MoveServo.Response:
        """
        Handle a move servo command service request.

        Encodes the move servo request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {MoveServo.Request}: MoveServo command parameters.
            response {MoveServo.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_move_servo(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def read_parameter_command_server(
            self, request: ReadParameter.Request,
            response: ReadParameter.Response) -> ReadParameter.Response:
        """
        Handle a read parameter command service request.

        Encodes the read parameter request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {ReadParameter.Request}: ReadParameter command parameters.
            response {ReadParameter.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_read_parameter(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            response.value = -1
            return response

        response.success, response.message, response.value = await self._run_command(fcode)
        return response

    async def write_parameter_command_server(
            self, request: WriteParameter.Request,
            response: WriteParameter.Response) -> WriteParameter.Response:
        """
        Handle a write parameter command service request.

        Encodes the write parameter request into FCode and sends it
        to the FarmBot command sender.

        Args:
            request {WriteParameter.Request}: WriteParameter command parameters.
            response {WriteParameter.Response}: Service response object.
        """
        try:
            fcode = self.fcode_encoder.encode_write_parameter(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            return response

        response.success, response.message, _ = await self._run_command(fcode)
        return response

    async def list_all_command_server(self, request: Trigger.Request,
                                      response: Trigger.Response) -> Trigger.Response:
        """
        Handle the list all parameter command service request.

        Sends the F20 command to the FarmBot.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success, response.message, _ = await self._run_command('F20')
        return response

    async def handle_load_callback(self, goal_handle: ServerGoalHandle):
        """
        Handle an accepted calibration parameter loading goal.

        Initialises the loading context and starts the timer that processes the
        pending parameters.
        """
        self.goal_handle = goal_handle
        self.nb_params = len(self.goal_handle.request.params)
        self.load_param_timer = self.create_timer(1.0 / self.check_serial_freq, self.load_timer,
                                                  callback_group=MutuallyExclusiveCallbackGroup())

    async def load_timer(self):
        """
        Timer callback that advances the calibration parameter loading.

        Writes the next pending parameter, publishes loading progress feedback, and
        completes the action when all parameters have been processed or if a loading
        error occurs.
        """
        self.get_logger().info("LOAD TIMER CALLED")
        feedback = LoadingParameters.Feedback()
        if self.goal_handle.request.params and not self.load_param_timer.is_canceled():
            if not self.write_param_client.wait_for_service(1.0):
                self.node.get_logger().fatal('WriteParam Server not available!')
                raise ServerError('Parameter module failed: server unavailable')

            request = WriteParameter.Request()
            request.param = self.goal_handle.request.params.pop(0)
            request.value = self.goal_handle.request.values.pop(0)
            self.get_logger().info(
            f"Loading {request.param}, remaining={len(self.goal_handle.request.params)}"
            )
            request.during_calibration = False

            future = self.write_param_client.call_async(request=request)
            service_response = await future
            if not service_response.success:
                result = self.result_loading
                if service_response.message == 'firmware error':
                    result.code = LoadingParameters.Result.FIRMWARE_ERROR
                    result.message = (f'{service_response.message} triggered by the parameter'
                                      f'{request.param} and its value {request.value}')
                elif service_response.message == 'aborted':
                    result.code = LoadingParameters.Result.ABORTED
                    result.message = service_response.message
                elif service_response.message == 'estopped':
                    result.code = LoadingParameters.Result.ESTOPPED
                    result.message = service_response.message
                else:
                    result.code = LoadingParameters.Result.REJECTED
                    result.message = service_response.message
                self.goal_handle.execute()

            feedback.progress = (1-len(self.goal_handle.request.params)/self.nb_params)
            self.goal_handle.publish_feedback(feedback)
            self.get_logger().info("SETTING ACTION RESULT")
        elif not self.load_param_timer.is_canceled():
            result = self.result_loading
            result.code = LoadingParameters.Result.OK
            result.message = ''
            self.goal_handle.execute()

    async def load_params_execute_callback(
            self, goal_handle: ServerGoalHandle) -> LoadingParameters.Result:
        """
        Execute a loading parameters action.

        The goal is encoded into FCode and sent to the FarmBot.
        The action result is updated according to the command execution status.

        Args:
            goal_handle {ServerGoalHandle}: Accepted action goal.
        """
        self.load_param_timer.cancel()
        if self.result_loading.code == LoadingParameters.Result.OK:
            goal_handle.succeed()
            return self.result_loading

        goal_handle.abort()
        return self.result_loading

    async def estop_command_server(self, request: Trigger.Request,
                                   response: Trigger.Response) -> Trigger.Response:
        """
        Handle the emergency stop command service request.

        Sends the emergency stop command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        self.farmbot_cmd_sender('E')

        self.switch_led(self.fb_panel['estop_led'], self.fb_panel['LED_OFF'])
        self.switch_led(self.fb_panel['unlock_led'], self.fb_panel['LED_FLASHING'])
        self.estop_active.data = True
        self.estop_active_pub.publish(self.estop_active)

        response.success = True
        return response

    async def resume_command_server(self, request: Trigger.Request,
                                    response: Trigger.Response) -> Trigger.Response:
        """
        Handle the emergency stop reset command service request.

        Sends the emergency stop reset command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success, response.message, _ = await self._run_command('F09')

        if response.success:
            self.switch_led(self.fb_panel['estop_led'], self.fb_panel['led_on'])
            self.switch_led(self.fb_panel['unlock_led'], self.fb_panel['led_on'])
            self.estop_active.data = False
            self.estop_active_pub.publish(self.estop_active)

        return response

    async def abort_command_server(self, request: Trigger.Request,
                                   response: Trigger.Response) -> Trigger.Response:
        """
        Handle the abort command service request.

        Sends the abort command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        self.farmbot_cmd_sender('@')
        # Only latch when a command was actually interrupted (or unlatch a prior abort)
        if ((self.code_response is not None and not self.code_response.done())
                or self.abort_active.data):
            self.abort_active.data = not self.abort_active.data
            self.abort_active_pub.publish(self.abort_active)

        response.success = True
        return response

    async def end_stop_command_server(self, request: Trigger.Request,
                                      response: Trigger.Response) -> Trigger.Response:
        """
        Handle the end stops command service request.

        Sends the end stops command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success, response.message, _ = await self._run_command('F81')
        return response

    async def sw_version_command_server(self, request: Trigger.Request,
                                        response: Trigger.Response) -> Trigger.Response:
        """
        Handle the software version command service request.

        Sends the software version command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success, response.message, _ = await self._run_command('F83')
        return response

    async def curr_position_command_server(self, request: Trigger.Request,
                                           response: Trigger.Response) -> Trigger.Response:
        """
        Handle the current position command service request.

        Sends the current position command to the FarmBot command sender.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success, response.message, _ = await self._run_command('F82')
        return response

    def farmbot_cmd_sender(self, cmd: str):
        """
        Send an FCode command to the FarmBot over the serial connection.

        Ensures that the command is newline-terminated, stores the command name
        for response tracking, and writes the encoded command to the serial port.

        Args:
            cmd {str}: FCode command to send to the FarmBot.
        """
        # Ensure the endline char at the end of the command
        if cmd[-1] != '\n':
            cmd += '\n'

        # Record the transmitted command
        self.previous_cmd = (cmd.split(' ')[0] if ' ' in cmd else cmd.split('\n')[0])

        self.get_logger().info(f'Sent message: {cmd}')
        #  Send through serial the command
        self.ser.write(cmd.encode('utf-8'))

    # Receiving messages from Farmbot
    def serial_receive(self):
        """
        Read and process incoming messages from the FarmBot serial connection.

        This timer callback reads a line from the serial port. If a message is
        received, it is logged and forwarded to the message handler.
        """
        # Read from serial
        line = self.ser.readline().decode('utf-8').rstrip()
        # If a command is read, handle it
        if line:
            self.get_logger().info(f'Received message: {line}')

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message: str):
        """
        Process a message received from the FarmBot serial connection.

        The received message is parsed to update the robot state, publish action
        feedback, resolve pending command responses, and forward the raw message
        to other ROS 2 nodes.

        Args:
            message {str}: Raw message received from the FarmBot.
        """
        # Record the message
        self.serial_feedback.data = message

        if message == 'R99 ARDUINO STARTUP COMPLETE':
            self.config_server.retrieve_config()

        # Extract the command code
        rep_code = (message).split(' ')[0]

        if rep_code == 'R21' or rep_code == 'R23':
            code = message.split(' ')
            self.get_logger().info(f'Updated parameter {code[1][1:]} to {code[2][1:]}')
            self.config_server.set_value(int(float(code[1][1:])), int(float(code[2][1:])))

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
                case 'R41':
                    code = (message).split(' ')
                    value = int(code[2][1:])
                    result = [True, '', value]
                    self.code_response.set_result(result)
                case 'R21' | 'R23':
                    code = (message).split(' ')
                    value = int(code[2][1:])
                    self.get_logger().info(f'Updated parameter {code[1][1:]} to {code[2][1:]}')
                    self.config_server.set_value(int(float(code[1][1:])), int(float(code[2][1:])))
                    result = [True, '', value]
                    self.code_response.set_result(result)
                case 'R81' | 'R82' | 'R83':
                    result = [True, message[4:], -1]
                    self.code_response.set_result(result)

        # Send the reporting message for further processing by other nodes
        self.serial_feedback_pub.publish(self.serial_feedback)

    def percentage_calculation(self, current_position: list) -> float:
        """
        Calculate the progress of the current movement.

        The progress is computed as the average normalized completion of all axes
        that are moving. If no axis is moving, the function returns 1.0.

        Args:
            current_position {list}: Current [x, y, z] position of the FarmBot.
        """
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
        return 1.0

    # Service Client
    def switch_led(self, led_pin: int, state: bool):
        """
        Send a request to switch an LED on or off.

        The request is sent asynchronously. If the LED handling service is not
        available, the request is ignored to avoid blocking the executor.

        Args:
            led_pin {int}: GPIO pin of the LED to control.
            state {bool}: Desired LED state (True for on, False for off).
        """
        # Never block the executor waiting for the LED server (estop path!)
        if not self.led_client.service_is_ready():
            self.get_logger().warn('LED Handling Server not available!')
            return

        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = self.led_client.call_async(request=request)
        future.add_done_callback(self.led_panel_callback)

    def led_panel_callback(self, future):
        """
        Handle the response from the LED handling service.

        Logs a warning if the service reports a failure or an error if the service
        call itself fails.

        Args:
            future: Future containing the service response.
        """
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Failure in LED Panel Handling!')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    def destroy_node(self):
        """Close the serial when the node is destroyed."""
        self.ser.close()
        super().destroy_node()


def main(args=None):
    """Initialise and run the Serial controller node."""
    rclpy.init(args=args)

    serial_node = SerialController()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.destroy_node()
    except Exception as e:
        serial_node.get_logger().info(f'{e}')
        serial_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
