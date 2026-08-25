"""
FarmBot tool sequences module.

Provides helper methods to build tools sequences.
"""
import ast

from farmbot_controllers.command_map import Sequence
from farmbot_controllers.sequences.mount_tool import mount_tool
from farmbot_controllers.sequences.unmount_tool import unmount_tool

from farmbot_utils.exceptions import ServerError

from rclpy.node import Node

from std_srvs.srv import Trigger


class ToolSequences:
    """Tool sequences module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the tool_sequences module."""
        self.node = node
        self.current_map = {}

        # Config service client
        self.get_map_client = self.node.create_client(Trigger, 'get_map')

    def get_map(self, on_done=None):
        """Request the current map from the MapController node."""
        self._server_availability('GetMap', self.get_map_client)
        self.get_map_client.call_async(Trigger.Request()).add_done_callback(
                                       lambda future: self._complete(future, on_done))

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Map_controller failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report, never leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not response.success:
            self.node.get_logger().warn(f'Command failed: {response.message}')
        else:
            self.current_map = ast.literal_eval(response.message)
        if on_done is not None:
            on_done(response)

    def mount_tool_command(self, index: int, on_done=None):
        """
        Form the tool mounting sequence based on the tool details.

        Args:
            index {int}: Index of the tool to mount
        """
        def mount_tool_cmd(response):
            if response is None or not response.success:
                if on_done:
                    on_done(response)
                return
            tool = self.current_map['map_reference']['tools'][f'T{index}']

            # Check the tool information for any possible errors
            try:
                release_x_inc, release_y_inc = self.__get_release_direction(tool['release_dir'])
                self.__check_tool_details(tool, release_x_inc, release_y_inc)
            except Exception as error:
                self.node.get_logger().error('Tool mounting failed %r' % (error, ))
                return

            Sequence(mount_tool(x=tool['position']['x'], y=tool['position']['y'], z=tool['position']['z'],
                                x_inc=release_x_inc, y_inc=release_y_inc,
                                z_inc=self.current_map['map_reference']['safe_z_increment']))

        self.get_map(on_done=mount_tool_cmd)

    def unmount_tool_command(self, index: int, on_done=None):
        """
        Form the tool unmounting sequence based on the tool details.

        Args:
            index {int}: Index of the tool to unmount
        """
        def unmount_tool_cmd(response):
            if response is None or not response.success:
                if on_done:
                    on_done(response)
                return
            tool = self.current_map['map_reference']['tools'][f'T{index}']

            # Check the tool information for any possible errors
            try:
                release_x_inc, release_y_inc = self.__get_release_direction(tool['release_dir'])
                self.__check_tool_details(tool, release_x_inc, release_y_inc)
            except Exception as error:
                self.node.get_logger().error('Tool unmounting failed %r' % (error, ))
                return

            Sequence(unmount_tool(x=tool['position']['x'], y=tool['position']['y'],
                                  z=tool['position']['z'], x_inc=release_x_inc,
                                  y_inc=release_y_inc,
                                  z_inc=self.current_map['map_reference']['safe_z_increment']))

        self.get_map(on_done=unmount_tool_cmd)

    def __get_release_direction(self, direction: int):
        """
        Get tool release coordinate increments based on mounting orientation.

        1: -x, 2: +x, 3: -y, 4: +y
        """
        increments = {
            1: (-100.0, 0.0),
            2: (100.0, 0.0),
            3: (0.0, -100.0),
            4: (0.0, 100.0),
        }
        if direction not in increments:
            raise ValueError('Release direction for the tool unrecognized! '
                             'Check configuration!')

        return increments[direction]

    def __check_tool_details(self, tool_info: dict, x_inc: float, y_inc: float):
        """Check if the tool position is reachable and valid."""
        map_max_x = self.current_map['map_reference']['x_len']
        map_max_y = self.current_map['map_reference']['y_len']
        map_max_z = self.current_map['map_reference']['z_len']
        # Check if the tool position is reachable
        if not self.__outside_bounds(x_min=0.0, x_max=map_max_x, y_min=0.0,
                                     y_max=map_max_y, z_min=(-1)*map_max_z,
                                     z_max=0.0, x=tool_info['position']['x'], y=tool_info['position']['y'],
                                     z=tool_info['position']['z']):
            self.node.get_logger().warn(f'Max pos {map_max_x}  {map_max_y}  '
                                         f'{map_max_z} ')

            raise ValueError(f"Tool home position {tool_info['position']['x']} {tool_info['position']['y']} "
                             f"{tool_info['position']['z']} is outside of the farmbot's reach ({map_max_x},{map_max_y},{map_max_z})!")

        # Check if the release position is valid
        if not self.__outside_bounds(x_min=0.0, x_max=map_max_x, y_min=0.0,
                                     y_max=map_max_y, z_min=(-1)*map_max_z, z_max=0.0,
                                     x=tool_info['position']['x'] + x_inc, y=tool_info['position']['y'] + y_inc,
                                     z=tool_info['position']['z']):
            raise ValueError(f"Tool release position {tool_info['position']['x'] + x_inc} "
                             f"{tool_info['position']['y'] + y_inc} {tool_info['position']['z']} is outside of "
                             f"the farmbot's reach ({map_max_x},{map_max_y},{map_max_z})!")

    def __outside_bounds(self, x_min: float, x_max: float, y_min: float, y_max: float,
                         z_min: float, z_max: float, x: float, y: float, z: float):
        """Check if a position (x, y, z) is within the ranges set for each axis."""
        if (
            (x >= x_min and x <= x_max)
            and (y >= y_min and y <= y_max)
            and (z >= z_min and z <= z_max)
        ):
            return True
        return False
