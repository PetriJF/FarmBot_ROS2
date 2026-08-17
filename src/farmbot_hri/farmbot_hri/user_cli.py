#!/usr/bin/env python3
"""
User CLI node for ROS2 Farmbot.

Publishes user commands to the farmbot controller for execution and
demonstrates command priority handling versus sequencer commands.
"""
import readline

from farmbot_interfaces.action import LoadingParameters

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class UserCLI(Node):
    """
    ROS2 node that publishes the user commands for the Task sequencer.

    Node used for recording user commands and sending them forward to the
    task sequencer for interpretation, the queuing and execution.
    """

    # Node contructor
    def __init__(self):
        """Initialize the UserCLI node and its ROS2 publishers."""
        super().__init__('UserCLI')

        # User command publisher
        self.cmd = String()
        self.input_pub = self.create_publisher(String, 'request_command', 10)

        # Initialisation of the service clients for priority commands (estop, abort, resume)
        self.estop_client = self.create_client(Trigger, 'estop')
        self.abort_client = self.create_client(Trigger, 'abort')
        self.resume_client = self.create_client(Trigger, 'resume')

        self.loading_params_client = ActionClient(self, LoadingParameters, 'loading_params')

        self.line_number_list = []

        # Log the initialization
        self.get_logger().info('User CLI Initialized..')

        self.get_logger().info("""\n
                               This is a user interface for the ROS2 Farmbot
                               Controllers. The commands accepted can be found in the
                               Documentation under High Level Commands.\n
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

    def check_input(self):
        """
        Check the input stream for valid commands and sends them on.

        Commands can be simple keys (e.g. T_1_1) or compound commands (e.g. P_1), where the
        key is followed by more information that is to be used by the farmbot
        """
        # Valid Keys and Commands
        valid_keys = ('E', 'R', '@', 'SW_VER', 'w', 'a', 's', 'd', '1', '2', '3', 'h', 'j', 'k',
                      'l', 'fh', 'o', 'p', 'T_1_1', 'T_1_2', 'T_2_1', 'T_2_2', 'T_3_1', 'T_3_2',
                      'T_4_1', 'T_4_2', 'T_5_1', 'T_5_2', 'T_6_1', 'T_6_2', 'P_3', 'P_4', 'P_5',
                      'P_9', 'I_0', 'I_1', 'I_2', 'I_3', 'I_4', 'D_C', 'D_L_1', 'D_L_0', 'D_W_1',
                      'D_W_0', 'D_V_1', 'D_V_0', 'H_0', 'D_S_C', 'P4_0', 'P4_1', 'C_0', 'P_1',
                      'P_2', 'C_1', 'C_2', 'T_1_0', 'T_2_0', 'T_3_0', 'T_4_0', 'T_5_0', 'T_6_0',
                      'S_1_0', 'S_2_0', 'S_3_0', 'M', 'M_S', 'CONF', 'H_1', 'M_SV')

        # Record the user input
        user_input = input('\nEnter command: ')
        code = user_input.split(' ')

        line_number = 2  # By default, 2 lines are printed for each command (input + \n)

        # Send the user input to the farmbot controller if it is a valid key or command
        if code[0] in valid_keys:
            # Send the command with priority at the UART controller
            if user_input == 'E':
                self._send(self.estop_client)
                self.get_logger().info('ESTOP button pressed')
                line_number += 1
            elif user_input == 'R':
                self._send(self.resume_client)
                self.get_logger().info('RESET button pressed')
                line_number += 1
            elif user_input == '@':
                self._send(self.abort_client)
                self.get_logger().info('Abort movement')
                line_number += 1
            elif code[0] == 'C_1':
                self.send_load_params_goal(code[1])
            else:
                self.cmd.data = user_input
                self.input_pub.publish(self.cmd)
        else:
            print('Invalid input\n')
            line_number += 2

        self.line_number_list.append(line_number)
        self.screen_scrolling()

    def screen_scrolling(self):
        """
        Keep the command help visible by scrolling the terminal display.

        Uses ANSI escape sequences to move the cursor upward, remove the oldest
        command lines from the terminal, and restore the cursor to its original
        position.
        """
        if len(self.line_number_list) >= 3:
            line_number = self.line_number_list[0]
            print(f'\033[{sum(self.line_number_list)}A', end='')

            for _ in range(line_number):
                print('\033[1M', end='')

            self.line_number_list.pop(0)
            print(f'\033[{sum(self.line_number_list)}B', end='')

    def send_load_params_goal(self, config_name: str, params=[], values=[], on_done=None):
        """
        Send a LoadingParameters goal to the FarmBot action server.

        The goal is configured with the specified profile name, parameter names, and corresponding
        values, then sent asynchronously to the loading parameters action server.

        Args:
            config_name (str): Name of the configuration profile to load.
            params (list[int]): List of parameter identifiers to load.
            values (list[int]): List of values corresponding to the specified parameters.
            on_done (callable, optional): Callback function called when the goal has been completed.
        """
        self._server_availability('LoadingParameters', self.loading_params_client)

        goal = LoadingParameters.Goal()
        goal.profile = config_name
        goal.params = params
        goal.values = values

        self.loading_params_client.send_goal_async(
            goal,
            feedback_callback=self.loading_params_feedback_callback
        ).add_done_callback(lambda future: self.goal_response_callback(future, on_done))

    def loading_params_feedback_callback(self, feedback_msg: LoadingParameters.Feedback):
        """
        Handle feedback messages from the LoadingParameters action server.

        Logs the goal completion during the loading of the parameters.
        """
        percentage = feedback_msg.feedback.progress
        self.node.get_logger().info(f'Loading progress: {percentage*100:.2f} %')

    def goal_response_callback(self, future, on_done=None):
        """
        Handle the action server goal response.

        Retrieves the result asynchronously when the goal is accepted; otherwise
        signals completion with None (rejected or send failed) so a chained
        caller (on_done) is never left waiting.
        """
        try:
            goal_handle: ClientGoalHandle = future.result()
        except Exception as error:  # send failed - report, never leave on_done hanging
            self.node.get_logger().error(f'Goal send failed: {error}')
            if on_done is not None:
                on_done(None)
            return

        if goal_handle.accepted:
            self.node.get_logger().info('Goal accepted')
            goal_handle.get_result_async().add_done_callback(
                lambda result_future: self.goal_result_callback(result_future, on_done)
            )
        else:
            self.node.get_logger().warn('Goal rejected')
            # A rejected goal never produces a result so signal it so on_done resolves.
            if on_done is not None:
                on_done(None)

    def goal_result_callback(self, future, on_done=None):
        """
        Handle the final result from the action server.

        Logs the returned status and forwards the result (or None on failure) to
        the completion callback so a chained caller always resolves.
        """
        try:
            result = future.result().result
        except Exception as error:  # result failed - report, never leave on_done hanging
            self.node.get_logger().error(f'Result retrieval failed: {error}')
            if on_done is not None:
                on_done(None)
            return

        cmd_status = result.code

        if cmd_status == result.ESTOPPED:
            self.node.get_logger().info('The current command has been stopped by a estop request')

        elif cmd_status == result.ABORTED:
            self.node.get_logger().info('The Farmbot has been paused.')

        elif cmd_status == result.FIRMWARE_ERROR:
            self.node.get_logger().info('The command has finished with due to a firmware error.')

        elif cmd_status == result.REJECTED:
            self.node.get_logger().info(f'The command has been rejected. {result.message}')

        elif cmd_status == result.OK:
            self.node.get_logger().info('The command was successful and has been completed')

        if on_done is not None:
            on_done(result)


def main(args=None):
    """Initialize and run the user CLI node."""
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
