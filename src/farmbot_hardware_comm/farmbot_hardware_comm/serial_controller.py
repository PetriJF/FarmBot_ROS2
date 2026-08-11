#!/usr/bin/env python3
"""
FarmBot serial communication node.

This node acts as the communication bridge between ROS 2 and the FarmBot
controller. It handles service and action requests, converts commands into
FCode, sends them to the Farmduino over the serial connection, and processes
received messages to provide feedback and update the system state.
"""
from farmbot_hardware_comm.command_servers import CommandServers
from farmbot_hardware_comm.config_managers import ConfigServer
from farmbot_hardware_comm.fcode_encoder import Encoder
from farmbot_hardware_comm.led_panel import LedPanel
from farmbot_hardware_comm.modules.exceptions import EncodeError, YAMLError
from farmbot_hardware_comm.modules.yaml_handler import YAMLHandler

from farmbot_interfaces.action import HomeAxes, LoadingParameters, MoveGantry

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

        self.fb_startup = False
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

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd = ''
        self.command_type = ''
        self.code_response: Future = None

        self.mission = {
            'starting_position': [],
            'final_position': [],
        }

        self.request_result = {
            'R03': [False, 'firmware error', -1],
            'R86': [False, 'aborted', -1],
            'R02': [True, '', -1],
            'R87': [False, 'estopped', -1],
            'R08': [True, '', -1],
            'R41': [True, '', -1],
            'R21': [True, '', -1],
            'R23': [True, '', -1],
            'R81': [True, '', -1],
            'R82': [True, '', -1],
            'R83': [True, '', -1],
        }

        self.config_path = YAMLHandler.join_path(ws_path, folder_config_name)
        YAMLHandler.make_dir(self.config_path)

        if not YAMLHandler.existing_path(YAMLHandler.join_path(self.config_path,
                                                               'active_map.yaml')):
            self.get_logger().warn('The active_map.yaml file was not found at '
                                   f'{self.config_path}. Gantry boundary checking stays disabled '
                                   'until the map is set up.')

        # Initialising modules
        self.fcode_encoder = Encoder(self.config_path, log=self.get_logger)
        self.config_server = ConfigServer(self, self.config_path,
                                          run_command=self._run_command,
                                          encoder=self.fcode_encoder)

        self.directory = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')

        try:
            self.non_immediate_cmds = YAMLHandler.load_yaml(self.directory,
                                                            'CommandsResponses.yaml')
            fb_panel = YAMLHandler.load_yaml(self.directory, 'FarmbotPanel.yaml')
        except YAMLError as e:
            self.get_logger().warn(f'yaml error: {e}')
            return

        self.led_panel = LedPanel(self, fb_panel)
        self.led_panel.show_ready()

        self.cmd_callback_group = ReentrantCallbackGroup()

        self.command_servers = CommandServers(self, self.fcode_encoder, self._run_command,
                                              lambda: self.config_server.param_vals,
                                              self.cmd_callback_group)

        # Initialise motor servers
        self.move_gantry_server = ActionServer(self, MoveGantry, 'move_gantry',
                                               goal_callback=self.goal_callback,
                                               execute_callback=self.gantry_execute_callback,
                                               callback_group=self.cmd_callback_group)
        self.home_axes_server = ActionServer(self, HomeAxes, 'home_axes',
                                             goal_callback=self.goal_callback,
                                             execute_callback=self.home_execute_callback,
                                             callback_group=self.cmd_callback_group)
        self.load_params_server = ActionServer(self, LoadingParameters, 'loading_params',
                                               goal_callback=self.goal_callback,
                                               execute_callback=(
                                                   self.config_server
                                                   .load_params_execute_callback),
                                               callback_group=self.cmd_callback_group)

        # Initialise state servers
        self.estop_trigger_server = self.create_service(Trigger, 'estop',
                                                        self.estop_command_server,
                                                        callback_group=self.cmd_callback_group)
        self.resume_trigger_server = self.create_service(Trigger, 'resume',
                                                         self.resume_command_server,
                                                         callback_group=self.cmd_callback_group)
        self.abort_trigger_server = self.create_service(Trigger, 'abort',
                                                        self.abort_command_server,
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

        # Initialise Serial Communication
        self.ser = serial.Serial(serial_port, serial_speed, timeout=1)
        self.ser.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rx_timer = self.create_timer(1.0 / self.check_serial_freq, self.serial_receive)

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
        self.command_type = ''
        return result[0], result[1], result[2]

    # Callbacks
    def goal_callback(self, goal_request) -> GoalResponse:
        """
        Check whether an incoming action goal can be accepted.

        A goal is rejected if an emergency stop or abort command is active, if a
        parameter load is running, or if another command is already being executed.
        Otherwise, the goal is accepted.

        Args:
        goal_request: Incoming action goal request.
        """
        self.get_logger().info('Received goal request')

        if self.estop_active.data or self.abort_active.data:
            self.get_logger().info('Goal rejected because an estop or abort command is running')
            return GoalResponse.REJECT

        if self.config_server.loading:
            self.get_logger().info('Goal rejected: a parameter load is running')
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
            fcode = self.fcode_encoder.encode_move_gantry(goal_handle.request,
                                                          self.config_server.param_vals)
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

        self.led_panel.show_estopped()
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
            self.led_panel.show_ready()
            self.estop_active.data = False
            self.estop_active_pub.publish(self.estop_active)
            self.abort_active.data = False
            self.abort_active_pub.publish(self.abort_active)

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
        for cmd_type in self.non_immediate_cmds:
            if self.previous_cmd in self.non_immediate_cmds[cmd_type]:
                self.command_type = cmd_type
                break

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
        Process a message received from the FarmBot firmware.

        Handles unsolicited reports, publishes the raw message, and processes
        responses for active commands.

        Args:
            message {str}: Raw message received from the FarmBot.
        """
        # Record the message
        self.serial_feedback.data = message
        if message == 'R99 ARDUINO STARTUP COMPLETE':
            self.fb_startup = True
            self.config_server.schedule_startup_load()

        code = (message).split(' ')

        self.handle_unsolicited_reports(code)

        # Send the reporting message for further processing by other nodes
        self.serial_feedback_pub.publish(self.serial_feedback)

        if self.command_type:
            self.handle_command_response(code)

    def handle_unsolicited_reports(self, code: list):
        """
        Process a message received from the FarmBot serial connection.

        The received message is parsed to update the robot state, publish action
        feedback, resolve pending command responses, and forward the raw message
        to other ROS 2 nodes.

        Args:
            message {str}: Raw message received from the FarmBot.
        """
        match code[0]:
            case 'R21' | 'R23':
                if self.fb_startup:
                    self.get_logger().info(f'Updated parameter {code[1][1:]} to {code[2][1:]}')
                    self.config_server.set_value(int(float(code[1][1:])), int(float(code[2][1:])))
            case 'R87':
                if not self.estop_active.data:
                    self.estop_active.data = True
                    self.estop_active_pub.publish(self.estop_active)
            case 'R82':
                self.fb_position.point.x = float(code[1][1:])
                self.fb_position.point.y = float(code[2][1:])
                self.fb_position.point.z = float(code[3][1:])
                self.fb_position.header.stamp = self.get_clock().now().to_msg()
                self.fb_position_pub.publish(self.fb_position)
            case 'R88':
                self.get_logger().warn('No configuration files were found in '
                                       f'{self.config_path}. Use the C_1 '
                                       'command to create these files (see the documentation).')

    def handle_command_response(self, code: list):
        """
        Handle firmware responses for active commands.

        Publishes action feedback and resolves pending command responses.

        Args:
            code {list}: Parsed firmware response.
        """
        if (self.command_type in ['home_axes', 'move_gantry']
                and self.goal_handle is not None and self.goal_handle.is_active):
            self.action_feedback_publisher(self.command_type)

        if (code[0] in self.non_immediate_cmds[self.command_type][self.previous_cmd]['responses']
           and self.code_response and not self.code_response.done()):
            if code[0] in self.request_result.keys():
                rep_code = code[0]
                if rep_code in ['R41', 'R21', 'R23']:
                    self.request_result[rep_code][2] = int(code[2][1:])
                elif rep_code == 'R83':
                    self.request_result[rep_code][1] = code[1]
                elif rep_code in ['R81', 'R82', 'R83']:
                    self.request_result[rep_code][1] = f'{code[1]} {code[2]} {code[3]}'
                self.code_response.set_result(self.request_result[rep_code])

    def action_feedback_publisher(self, cmd_type: str):
        """
        Publish feedback for the active ROS 2 action.

        Updates the action feedback with the current FarmBot position and, for
        gantry movements, the current execution progress.

        Args:
            cmd_type {str}: Type of the active command.
        """
        x_pos = self.fb_position.point.x
        y_pos = self.fb_position.point.y
        z_pos = self.fb_position.point.z
        if cmd_type == 'home_axes':
            feedback = HomeAxes.Feedback()
        else:
            feedback = MoveGantry.Feedback()
            feedback.progress = float(self.percentage_calculation([x_pos, y_pos, z_pos]))
        feedback.position.x = x_pos
        feedback.position.y = y_pos
        feedback.position.z = z_pos
        self.goal_handle.publish_feedback(feedback)

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
