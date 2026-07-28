"""
FarmBot movement module.

Provides helper methods for gantry movements, axis homing, and calibration
through ROS2 action commands.
"""
from farmbot_interfaces.action import HomeAxes, MoveGantry

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class Movement:
    """Movement module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the movement module and the ROS2 action clients."""
        self.node = node

        self.move_gantry_client = ActionClient(self.node, MoveGantry, 'move_gantry')
        self.home_axes_client = ActionClient(self.node, HomeAxes, 'home_axes')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_server(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Movement module failed: server unavailable')

    # Calibration and Homing Functions
    def go_home(self):
        """
        Call the FarmBot HomeAxes action for homing.

        Send a goal to have the Farmbot go home
        """
        self.send_home_goal(op=HomeAxes.Goal.GO_HOME)

    def find_axis_home(self, x: bool, y: bool, z: bool):
        """
        Call the FarmBot HomeAxes action for homing a specific axis.

        Home the selected axis on the farmbot. For example, for homing x and y you set them to True.

        Args:
            x {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            y {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            z {Bool}: True if the X-Axis should have it's home position found. Defaults to False
        """
        self.send_home_goal(op=HomeAxes.Goal.FIND_HOME, x_axis=x, y_axis=y, z_axis=z)

    def calibrate_axis(self, x: bool, y: bool, z: bool):
        """
        Call the FarmBot HomeAxes action to calibrate a specific axis.

        Calibrate the selected axis on the farmbot. For example, for calibrating
        the length along the x and y axis, you set them to True.

        Args:
            x {Bool}: True if the X-Axis is to be calibrated. Defaults to False
            y {Bool}: True if the Y-Axis is to be calibrated. Defaults to False
            z {Bool}: True if the Z-Axis is to be calibrated. Defaults to False
        """
        self.send_home_goal(op=HomeAxes.Goal.CALIBRATE, x_axis=x, y_axis=y, z_axis=z)

    def set_curr_to_home(self):
        """
        Call the FarmBot HomeAxes action to set the current position as the new home.

        Sends a goal to update the FarmBot home position using the current axis
        coordinates.
        """
        self.send_home_goal(op=HomeAxes.Goal.SET_HOME)

    def send_home_goal(self, op: int, x_axis=False, y_axis=False, z_axis=False):
        """
        Send a HomeAxes goal to the FarmBot action server.

        Creates and sends a HomeAxes goal containing the requested operation
        and selected axes.

        Args:
            op {int}: HomeAxes operation type.
            x_axis {bool}: Whether the X axis is selected.
            y_axis {bool}: Whether the Y axis is selected.
            z_axis {bool}: Whether the Z axis is selected.
        """
        self._server_availability('HomeAxes', self.home_axes_client)

        goal = HomeAxes.Goal()
        goal.op = op
        goal.x = x_axis
        goal.y = y_axis
        goal.z = z_axis

        self.home_axes_client.send_goal_async(
            goal,
            feedback_callback=self.home_goal_feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def home_goal_feedback_callback(self, feedback_msg: HomeAxes.Feedback):
        """
        Handle feedback messages from the HomeAxes action server.

        Logs the current FarmBot position received during the homing process.
        """
        curr_position = feedback_msg.feedback.position

        self.node.get_logger().info(f'Current postion : X{curr_position.x} '
                                    f'Y{curr_position.y} Z{curr_position.z}')

    # Gantry Movement Functions
    def move_gantry_abs(self, x_coord: float, y_coord: float, z_coord: float):
        """
        Move the Gantry at max speed to the desired coordinates.

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
        """
        self.send_move_gantry_goal(x_coord=x_coord, y_coord=y_coord, z_coord=z_coord)

    def move_gantry_s(self, x_coord: float, y_coord: float, z_coord: float, speed: float):
        """
        Move the Gantry to the desired coordinates at the the speed specified.

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
            speed {float}: Desired Speed for all the axis in percent format (0 - lowest speed,
                                                                             100 - highest speed)
        """
        self.send_move_gantry_goal(x_coord=x_coord, y_coord=y_coord, z_coord=z_coord,
                                   x_speed=speed, y_speed=speed, z_speed=speed, interpolated=False)

    def send_move_gantry_goal(self, x_coord: float, y_coord: float, z_coord: float,
                              interpolated=True, x_speed=100.0, y_speed=100.0, z_speed=100.0):
        """
        Send a MoveGantry goal to the FarmBot action server.

        Creates and sends a MoveGantry goal containing the target position,
        movement type, and axis speed percentages.

        Args:
            x_coord {float}: Target X coordinate.
            y_coord {float}: Target Y coordinate.
            z_coord {float}: Target Z coordinate.
            interpolated {bool}: Whether the movement should be interpolated.
            x_speed {float}: X axis speed percentage.
           y_speed {float}: Y axis speed percentage.
            z_speed {float}: Z axis speed percentage.
        """
        self._server_availability('MoveGantry', self.move_gantry_client)

        goal = MoveGantry.Goal()
        goal.target.x = x_coord
        goal.target.y = y_coord
        goal.target.z = z_coord
        goal.speed_percent_x = x_speed
        goal.speed_percent_y = y_speed
        goal.speed_percent_z = z_speed
        goal.interpolated = interpolated

        self.move_gantry_client.send_goal_async(
            goal,
            feedback_callback=self.move_gantry_goal_feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def move_gantry_goal_feedback_callback(self, feedback_msg: MoveGantry.Feedback):
        """
        Handle feedback messages from the MoveGantry action server.

        Logs the current FarmBot position received and the goal completion
        during the movement process.
        """
        curr_position = feedback_msg.feedback.position
        percentage = feedback_msg.feedback.progress
        self.node.get_logger().info(f'Current postion : X{curr_position.x} '
                                    f'Y{curr_position.y} Z{curr_position.z} \n'
                                    f'Goal completion: {percentage*100:.2f} %')

    def goal_response_callback(self, future):
        """
        Handle the action server goal response.

        Processes the goal response and retrieves the result asynchronously if
        the goal has been accepted.
        """
        self.goal_handle: ClientGoalHandle = future.result()

        if self.goal_handle.accepted:
            self.node.get_logger().info('Goal accepted')

            self.goal_handle.get_result_async().add_done_callback(
                self.goal_result_callback
            )

        else:
            self.busy_state = False
            self.node.get_logger().warn('Goal rejected')

    def goal_result_callback(self, future):
        """
        Handle the final result from the action server.

        Processes the action result and logs the command status returned by the
        FarmBot action server.
        """
        result = future.result().result
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
