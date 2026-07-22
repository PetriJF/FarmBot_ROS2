"""
Movement module for the FarmBot controller node.

Provides gantry motion, calibration, and homing command helpers.
"""
from farmbot_interfaces.action import HomeAxes, MoveGantry

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

# from std_msgs.msg import String


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class Movement:
    """Movement module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """
        Initialize the module.

        Args:
            node {Node}: the node the module extends
        """
        self.node = node

        self.X_MAX_SPEED = 400.0
        self.Y_MAX_SPEED = 400.0
        self.Z_MAX_SPEED = 400.0

        self.move_gantry_client = ActionClient(self.node, MoveGantry, 'move_gantry')
        self.home_axes_client = ActionClient(self.node, HomeAxes, 'home_axes')

        # self.gantry_config = String()    # Used for gantry configuration (homing, calibration)
        # self.move_gantry_cmd = String()  # Used for moving the gantry along the 3 axis

        # self.movement_pub = self.node.create_publisher(String, 'farmbot_command', 10)

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_server(1.0):
            raise ServerError(f'{cmd_name} Server not available!')

    # Calibration and Homing Functions
    def go_home(self):
        """Go to the home position on each axis."""
        self.send_home_goal(op=HomeAxes.Goal.GO_HOME)

    def find_axis_home(self, x: bool, y: bool, z: bool):
        """
        Home the selected axis on the farmbot.

        For example, for homing x and y you set them to True.

        Args:
            x {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            y {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            z {Bool}: True if the X-Axis should have it's home position found. Defaults to False
        """
        self.send_home_goal(op=HomeAxes.Goal.FIND_HOME, x_axis=x, y_axis=y, z_axis=z)

    def calibrate_axis(self, x: bool, y: bool, z: bool):
        """
        Calibrate the selected axis on the farmbot.

        For example, for calibrating the length along the x and y axis, you set them to True.

        Args:
            x {Bool}: True if the X-Axis is to be calibrated. Defaults to False
            y {Bool}: True if the Y-Axis is to be calibrated. Defaults to False
            z {Bool}: True if the Z-Axis is to be calibrated. Defaults to False
        """
        self.send_home_goal(op=HomeAxes.Goal.CALIBRATE, x_axis=x, y_axis=y, z_axis=z)

    def set_curr_to_home(self):
        """Set the current position of the extruder as the home position."""
        self.send_home_goal(op=HomeAxes.Goal.SET_HOME)

    def send_home_goal(self, op: int, x_axis=False, y_axis=False, z_axis=False):
        """Send the home axes goal its action server."""
        try:
            self._server_availability('HomeAxes', self.home_axes_client)
        except ServerError:
            raise

        goal = HomeAxes.Goal()
        goal.op = op
        goal.x = x_axis
        goal.y = y_axis
        goal.z = z_axis

        self.home_axes_client.send_goal_async(
            goal,
            feedback_callback=self.home_goal_feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def home_goal_feedback_callback(self, feedback_msg):
        """Handle feedback messages from the FarmbotComms action server."""
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
        """Send the move gantry goal its action server."""
        try:
            self._server_availability('MoveGantry', self.move_gantry_client)
        except ServerError:
            raise

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

    def move_gantry_goal_feedback_callback(self, feedback_msg):
        """Handle feedback messages from the MoveGantry action server."""
        curr_position = feedback_msg.feedback.position
        percentage = feedback_msg.feedback.progress
        self.node.get_logger().info(f'Current postion : X{curr_position.x} '
                                    f'Y{curr_position.y} Z{curr_position.z} \n'
                                    f'Goal completion: {percentage:.2f} %')

    def goal_response_callback(self, future):
        """Handle the action server's goal response."""
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
        """Handle the final result from the action server."""
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

    # def go_home(self):
    #     """Go to the home position on each axis."""
    #     self.manip_mvm_config(all_home=True)

    # def find_all_homes(self):
    #     """Find the home for all the axis on the farmbot."""
    #     self.find_axis_home(x=False, y=False, z=True)
    #     self.find_axis_home(x=False, y=True, z=False)
    #     self.find_axis_home(x=True, y=False, z=False)

    # def find_axis_home(self, x=False, y=False, z=False):
    #     """
    #     Home the selected axis on the farmbot.

    #     For example, for homing x and y you set them to True.

    #     Args:
    #         x {Bool}: True if the X-Axis should have it's home position found. Defaults to False
    #         y {Bool}: True if the X-Axis should have it's home position found. Defaults to False
    #         z {Bool}: True if the X-Axis should have it's home position found. Defaults to False
    #     """
    #     self.manip_mvm_config(x_axis=x, y_axis=y, z_axis=z)

    # def calibrate_all_axis(self):
    #     """Calibrate the lengths for all the axis on the farmbot."""
    #     self.calibrate_axis(x=False, y=False, z=True)
    #     self.calibrate_axis(x=False, y=True, z=False)
    #     self.calibrate_axis(x=True, y=False, z=False)

    # def calibrate_axis(self, x=False, y=False, z=False):
    #     """
    #     Calibrate the selected axis on the farmbot.

    #     For example, for calibrating the length along the x and y axis, you set them to True.

    #     Args:
    #         x {Bool}: True if the X-Axis is to be calibrated. Defaults to False
    #         y {Bool}: True if the Y-Axis is to be calibrated. Defaults to False
    #         z {Bool}: True if the Z-Axis is to be calibrated. Defaults to False
    #     """
    #    self.manip_mvm_config(calibrate=True, x_axis=x, y_axis=y, z_axis=z)

    # def set_curr_to_home(self):
    #     """Set the current position of the extruder as the home position."""
    #     self.set_curr_axis_home(x=True, y=True, z=True)

    # def set_curr_axis_home(self, x=False, y=False, z=False):
    #     """
    #     Set the current position of the extruder as the home position for the selected axis.

    #     Args:
    #         x {Bool}: True if this axis' position should be set as the axis' home.
    # Defaults to False
    #         y {Bool}: True if this axis' position should be set as the axis' home.
    # Defaults to False
    #         z {Bool}: True if this axis' position should be set as the axis' home.
    # Defaults to False
    #     """
    #     self.manip_mvm_config(set_this_home=True, x_axis=x, y_axis=y, z_axis=z)

    # def manip_mvm_config(self, all_home=False, set_this_home=False, calibrate=False, x_axis=False,
    #                      y_axis=False, z_axis=False):
    #     """
    #     Create the command for FarmBot homing and calibration translation.

    #     Note:
    #         The if you want to FIND HOME, you should set all_home and calibrate to False,
    #         followed by the axis you want the find home operation to work on.

    #     Args:
    #         all_home {Bool}: If it is True, the gantry goes to home (Note this is GO home not FIND
    #                         home). Defaults to False
    #         set_this_home {Bool}: Determines if the extruder's position should be set as
    #                               the home. Defaults to False
    #         calibrate {Bool}: Determines if the command will be taken as a homing or a calibration
    #                           command. If it is False, it Homes; if it is True, it calibrates.
    #                           Defaults to False
    #         x_axis {Bool}: Specifies wheather the command manipulates the X-Axis.
    # Defaults to False
    #         y_axis {Bool}: Specifies wheather the command manipulates the Y-Axis.
    # Defaults to False
    #         z_axis {Bool}: Specifies wheather the command manipulates the Z-Axis.
    # Defaults to False
    #     """
    #     self.gantry_config.data = ('home_handler ' + str(all_home) + ' ' + str(set_this_home)
    #                                + ' '
    #                                + str(calibrate) + ' ' + str(x_axis) + ' ' + str(y_axis) + ' '
    #                                + str(z_axis))

    #     self.movement_pub.publish(self.gantry_config)

    # # Gantry Movement Functions

    # def move_gantry_abs(self, x_coord=float, y_coord=float, z_coord=float):
    #     """
    #     Move the Gantry at max speed to the desired coordinates.

    #     Args:
    #         x_coord {float}: Desired X-Coordinate to move to
    #         y_coord {float}: Desired Y-Coordinate to move to.
    #         z_coord {float}: Desired Z-Coordinate to move to.
    #     """
    #     self.move_gantry(x_coord=x_coord, y_coord=y_coord, z_coord=z_coord,
    #                      x_speed=100.0, y_speed=100.0, z_speed=100.0)

    # def move_gantry_s(self, x_coord=float, y_coord=float, z_coord=float, speed=float):
    #     """
    #     Move the Gantry to the desired coordinates at the the speed specified.

    #     Args:
    #         x_coord {float}: Desired X-Coordinate to move to
    #         y_coord {float}: Desired Y-Coordinate to move to.
    #         z_coord {float}: Desired Z-Coordinate to move to.
    #         speed {float}: Desired Speed for all the axis in percent format (0 - lowest speed,
    #                                                                          100 - highest speed)
    #     """
    #     self.move_gantry(x_coord=x_coord, y_coord=y_coord, z_coord=z_coord,
    #                      x_speed=speed, y_speed=speed, z_speed=speed)

    # def move_gantry(self, x_coord=float, y_coord=float, z_coord=float, x_speed=float,
    #                 y_speed=float, z_speed=float):
    #     """
    #     Move the Gantry to the desired coordinates at the specified per axis speed.

    #     Args:
    #         x_coord {float}: Desired X-Coordinate to move to
    #         y_coord {float}: Desired Y-Coordinate to move to.
    #         z_coord {float}: Desired Z-Coordinate to move to.
    #         x_speed {float}: Desired Speed for X-Axis in percent format (0 - lowest speed,
    #                                                                      100 - highest speed)
    #         y_speed {float}: Desired Speed for Y-Axis in percent format (0 - lowest speed,
    #                                                                      100 - highest speed)
    #         z_speed {float}: Desired Speed for Z-Axis in percent format (0 - lowest speed,
    #                                                                      100 - highest speed)
    #     """
    #     self.manip_movement(mode=False, x_coord=x_coord, y_coord=y_coord, z_coord=z_coord,
    #                         x_speed=x_speed / 100.0 * self.X_MAX_SPEED,
    #                         y_speed=y_speed / 100.0 * self.Y_MAX_SPEED,
    #                         z_speed=z_speed / 100.0 * self.Z_MAX_SPEED)

    # def manip_movement(self, mode=False, x_coord=float, y_coord=float, z_coord=float,
    #                    x_speed=float, y_speed=float, z_speed=float):
    #     """
    #     Create the command for FarmBot gantry axis movement translation.

    #     Args:
    #         mode {Bool}: The movement mode. False - standard, True - with interpolation (needs
    #                      constant feed rate). Defaults to False
    #         x_coord {Int}: The desired x-axis coordinate relative to home
    #         y_coord {Int}: The desired y-axis coordinate relative to home
    #         z_coord {Int}: The desired z-axis coordinate relative to home
    #         x_speed {Int}: The speed used to reach the x coordinate
    #         y_speed {Int}: The speed used to reach the y coordinate
    #         z_speed {Int}: The speed used to reach the z coordinate
    #     """
    #     self.move_gantry_cmd.data = ('move_gantry ' + str(mode) + ' ' + str(x_coord) + ' '
    #                                  + str(y_coord) + ' ' + str(z_coord) + ' ' + str(x_speed)
    #                                  + ' ' + str(y_speed) + ' ' + str(z_speed))

    #     self.movement_pub.publish(self.move_gantry_cmd)
