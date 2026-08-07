#!/usr/bin/env python3
"""
Keyboard teleoperation node for ROS2 Farmbot.

Publishes keyboard commands to the farmbot controller for execution and
demonstrates command priority handling versus sequencer commands.
"""
import readline

from farmbot_controllers.sequences.calibration import calibrate_axes
from farmbot_controllers.sequences.find_home import find_home
from farmbot_controllers.sequences.single_call import single_call

from farmbot_interfaces.action import RunSequence

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger

_RESULT_OK, _RESULT_ESTOPPED, _RESULT_ABORTED = 0, 2, 3


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class UserCLI(Node):
    """
    ROS2 node that publishes keyboard commands for the Farmbot controller.

    Node used for recording keyboard commands and sending them forward to the
    farmbot controller for interpretation and execution.
    """

    # Node contructor
    def __init__(self):
        """Initialize the UserCLI node and its ROS2 publishers."""
        super().__init__('UserCLI')

        # Keyboard publisher
        self.cmd = String()
        self.input_pub = self.create_publisher(String, 'input_topic', 10)
        self.priority_pub = self.create_publisher(String, 'farmbot_command', 10)

        # Initialisation of the service clients for priority commands (estop, abort, resume)
        self.estop_client = self.create_client(Trigger, 'estop')
        self.abort_client = self.create_client(Trigger, 'abort')
        self.resume_client = self.create_client(Trigger, 'resume')

        # Initialisation of the run sequence client for sending commands to the sequencer
        self._run_client = ActionClient(self, RunSequence, 'run_sequence')

        # Farmbot Position Subscriber
        self.fb_position_sub = self.create_subscription(PointStamped, 'farmbot_position',
                                                        self.fb_position_callback, 10)
        # Memory
        self.cur_increment = 10.0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        self.line_number_list = []

        # Log the initialization
        self.get_logger().info('UserCLI Initialized..')

        self.get_logger().info("""\n
                               This is a keyboard-based controller for the ROS 2 FarmBot.
                               The accepted commands are documented under "High-Level Commands".

                               NOTE:
                               Commands entered here do NOT automatically enter the sequencer
                               because they take execution priority. The exception is sequencing
                               commands, such as `P_4` for watering.

                               Main Commands:
                               -------------------
                               C_0  - Calibrate the FarmBot
                               C_1  - Load parameter configuration
                               M    - Move the FarmBot
                               H_0  - Return the FarmBot to the home position
                               P_4  - Water all plants (sequenced command)
                               E    - Emergency stop
                               R    - Restart the robot after an emergency stop
                               @    - Pause the FarmBot""")

    def _send(self, client, on_done=None):
        """Check the server, send a Trigger request and complete via _complete."""
        if client.service_is_ready():
            client.call_async(Trigger.Request()).add_done_callback(
                              lambda future: self._complete(future, on_done))

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_server(1.0):
            self.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Task_Sequencer failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report, never leave on_done hanging
            self.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.get_logger().warn('Command failure!')
        elif not response.success:
            self.get_logger().warn(f'Command failed: {response.message}')
        elif response.message:
            self.get_logger().info(response.message)
        else:
            self.get_logger().info('Command successful')
        if on_done is not None:
            on_done(response)

    def _run_client(self, name: str, on_done=None):
        """
        Send a RunSequence goal to the FarmBot action server.

        Creates and sends a RunSequence goal containing the name of the sequence
        to execute.

        Args:
            name (str): Name of the sequence to execute.
            on_done (callable, optional): Callback function called when the goal
                response is received.
        """
        self._server_availability('run_sequence', self._run_client)

        goal = RunSequence.Goal()
        goal.name = name

        self._run_client.send_goal_async(
            goal,
            feedback_callback=self._run_client_feedback_callback
        ).add_done_callback(lambda future: self._goal_response_callback(future, on_done))

    def _run_client_feedback_callback(self, feedback_msg: RunSequence.Feedback):
        """
        Handle feedback messages from the RunSequence action server.

        Logs the goal status during the sequence execution.
        """
        status = feedback_msg.feedback.status
        self.get_logger().info(f'Status of the current command : {status}')

    def _goal_response_callback(self, future, on_done=None):
        """
        Handle the action server goal response.

        Retrieves the result asynchronously when the goal is accepted; otherwise
        signals completion with None (rejected or send failed) so a chained
        caller (on_done) is never left waiting.
        """
        try:
            goal_handle: ClientGoalHandle = future.result()
        except Exception as error:  # send failed - report, never leave on_done hanging
            self.get_logger().error(f'Goal send failed: {error}')
            if on_done is not None:
                on_done(None)
            return

        if goal_handle.accepted:
            self.get_logger().info('Goal accepted')
            goal_handle.get_result_async().add_done_callback(
                lambda result_future: self._goal_result_callback(result_future, on_done)
            )
        else:
            self.get_logger().warn('Goal rejected')
            # A rejected goal never produces a result so signal it so on_done resolves.
            if on_done is not None:
                on_done(None)

    def _goal_result_callback(self, future, on_done=None):
        """
        Handle the final result from the action server.

        Logs the returned status and forwards the result (or None on failure) to
        the completion callback so a chained caller always resolves.
        """
        try:
            result = future.result().result
        except Exception as error:  # result failed - report, never leave on_done hanging
            self.get_logger().error(f'Result retrieval failed: {error}')
            if on_done is not None:
                on_done(None)
            return

        cmd_status = result.status

        if cmd_status == _RESULT_ESTOPPED:
            self.get_logger().info('The current command has been stopped by a estop request')

        elif cmd_status == _RESULT_ABORTED:
            self.get_logger().info('The Farmbot has been paused.')

        elif cmd_status == _RESULT_OK:
            self.get_logger().info('The command was successful and has been completed')

        if on_done is not None:
            on_done(result)

    def fb_position_callback(self, position: PointStamped):
        """
        Update the current FarmBot position from an incoming position message.

        The current x, y, and z coordinates are extracted from the received
        PointStamped message and stored as the robot's current position.

        Args:
            position {PointStamped}: message containing the current position.
        """
        self.cur_x = position.point.x
        self.cur_y = position.point.y
        self.cur_z = position.point.z

    def check_input(self):
        """
        Check the input stream for valid commands and sends them on.

        Commands can be simple keys (e.g. T_1_1) or compound commands (e.g. P_1), where the
        key is followed by more information that is to be used by the farmbot
        """
        # Record the user input
        user_input = input('\nEnter command: ')
        code = user_input.split(' ')
        line_number = 2  # By default, 2 lines are printed for each command (input + \n)

        match code[0]:
            case 'E':
                self._send(self.estop_client)
                self.get_logger().info('ESTOP button pressed')
                line_number += 1
            case 'R':
                self._send(self.resume_client)
                self.get_logger().info('RESET button pressed')
                line_number += 1
            case '@':
                self._send(self.abort_client)
                self.get_logger().info('Abort movement')
                line_number += 1
            case 'SW_VER':
                single_call('software version', lambda hw,
                            cb: hw.states.request_sw_version(on_done=cb))
            case 'M':
                if len(code) != 4:
                    self.get_logger().warning('You need to include all 3 coordinates! '
                                              'Command ignored!')
                    line_number += 1
                else:
                    x = float(code[1])
                    y = float(code[2])
                    z = float(code[3])
                    single_call('move', lambda hw,
                                cb: hw.movement.move_gantry_abs(x, y, z, on_done=cb))
            case 'M_S':
                if len(code) != 5:
                    self.get_logger().warning('You need to include all 3 coordinates and a '
                                              'speed percentage! Command ignored!')
                    line_number += 1
                else:
                    x = float(code[1])
                    y = float(code[2])
                    z = float(code[3])
                    speed = float(code[4])
                    single_call('move_speed', lambda hw,
                                cb: hw.movement.move_gantry_s(x, y, z, speed, on_done=cb))
            case 'w' | 's':
                x = self.cur_x + (self.cur_increment * (-1 if code[0] == 's' else 1))
                single_call('x_axis_increment', lambda hw,
                            cb: hw.movement.move_gantry_abs(x, self.cur_y, self.cur_z, on_done=cb))
            case 'a' | 'd':
                y = self.cur_y + (self.cur_increment * (-1 if code[0] == 'a' else 1))
                single_call('y_axis_increment', lambda hw,
                            cb: hw.movement.move_gantry_abs(self.cur_x, y, self.cur_z, on_done=cb))
            case '1':
                self.cur_increment = 10.0
            case '2':
                self.cur_increment = 100.0
            case '3':
                self.cur_increment = 500.0
            case 'H_0':
                single_call('go_home', lambda hw, cb: hw.movement.go_home(on_done=cb))
            case 'H_1':
                find_home()
            case 'H_2':
                if len(code) != 2:
                    self.get_logger().warning('You need to specify the axes on which you want to '
                                              'determine the origin position! '
                                              'Command ignored!')
                    line_number += 1
                find_home(x=True if code[1] == 'X' else False,
                          y=True if code[1] == 'Y' else False,
                          z=True if code[1] == 'Z' else False)

            # TODO: case 'CONF':

            case 'C_0':  # C_0 for calib. all axis, C_0 X for calib. x axis and so on
                if len(code) == 1:
                    calibrate_axes()
                else:
                    calibrate_axes(x=True if code[1] == 'X' else False,
                                   y=True if code[1] == 'Y' else False,
                                   z=True if code[1] == 'Z' else False)

            # TODO: case 'C_1':
            # TODO: case 'C_2':
            # TODO: case 'T_1_0' | 'T_2_0' | 'T_3_0':
            # TODO: case 'T_1_1' | 'T_1_2' | 'T_2_1' | 'T_2_2' | 'T_3_1' | 'T_3_2':
            # TODO: case 'P_1':
            # TODO: case 'P_2':
            # TODO: case 'P_3' | 'P_4' | 'P_5' | 'P_9':
            # TODO: case 'S_1_0' | 'S_2_0' | 'S_3_0':
            # TODO: case 'I_0':
            # TODO: case 'I_1':
            # TODO: case 'I_2':
            # TODO: case 'I_3':
            # TODO: case 'I_4':

            case 'D_L_1' | 'D_L_0':
                light_pin = 7
                single_call('led_strip', lambda hw,
                            cb: hw.devices.set_pin_value(light_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_V_0' | 'D_V_1':
                vacuum_pin = 9
                single_call('vacuum_pump', lambda hw,
                            cb: hw.devices.set_pin_value(vacuum_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_W_0' | 'D_W_1':
                water_pin = 8
                single_call('water_pump', lambda hw,
                            cb: hw.devices.set_pin_value(water_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_WP':
                water_pin = 8
                single_call('water_pulses', lambda hw,
                            cb: hw.devices.set_pin_value(water_pin, value=1, pin_mode=False,
                                                         pulse=True, value2=0, delay_ms=500,
                                                         on_done=cb))
            case 'D_C':
                single_call('check_tool_mounting', lambda hw,
                            cb: hw.devices.read_pin(63, False, on_done=cb))
            case 'D_S_C':
                single_call('soil_sensor_reading', lambda hw,
                            cb: hw.devices.read_pin(59, True, on_done=cb))
            case 'P4_0' | 'P4_1':
                peripheral4_pin = 10
                single_call('peripheral4', lambda hw,
                            cb: hw.devices.set_pin_value(peripheral4_pin, int(code[0][3]),
                                                         False, False, on_done=cb))
            case 'P5_0' | 'P5_1':
                peripheral5_pin = 12
                single_call('peripheral5', lambda hw,
                            cb: hw.devices.set_pin_value(peripheral5_pin, int(code[0][3]),
                                                         False, False, on_done=cb))
            case 'M_SV':
                if len(code) != 3:
                    self.get_logger().warning('You need to include the servo pin and angle! '
                                              'Command ignored!')
                    line_number += 1
                else:
                    self.get_logger().info(f'Trying to move servo {int(code[1])} to {int(code[2])}')
                    line_number += 1
                    single_call('move_servo', lambda hw,
                                cb: hw.devices.move_servo(pin=int(code[1]), angle=float(code[2]),
                                                          on_done=cb))
            case _:
                print('Invalid input\n')
                line_number += 2

        self.line_number_list.append(line_number)
        self._screen_scrolling()

    def _screen_scrolling(self):
        if len(self.line_number_list) >= 3:
            line_number = self.line_number_list[0]
            print(f'\033[{sum(self.line_number_list)}A', end='')

            for _ in range(line_number):
                print('\033[1M', end='')

            self.line_number_list.pop(0)
            print(f'\033[{sum(self.line_number_list)}B', end='')


def main(args=None):
    """Initialize and run the keyboard teleoperation node."""
    rclpy.init(args=args)
    user_cli_node = UserCLI()

    try:
        while rclpy.ok():
            user_cli_node.check_input()
    except KeyboardInterrupt:
        pass
    finally:
        readline.clear_history()
        user_cli_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
