"""
Map info client utilities for ROS2 Farmbot.

ROS2 Python Module that lets the sequencer query the map handler through its
map_info service.
"""
from farmbot_interfaces.srv import StringRepReq

from rclpy.node import Node


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class MapInfo:
    """Map info module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the map info module and the ROS2 service client."""
        self.node = node

        self.map_info_client = self.node.create_client(StringRepReq, 'map_info')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Map info module failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report to not leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        else:
            self.node.get_logger().info(f'map_info replied: {response.data!r}')
        if on_done is not None:
            on_done(response)

    def list_plants(self, on_done=None):
        """
        Call the map_info service with the PlantList command.

        Args:
            on_done {callable}: Optional completion callback; receives the StringRepReq
                                response (data is a newline-separated 'index x y' list, or
                                None on a communication failure)
        """
        self._server_availability('map_info', self.map_info_client)

        request = StringRepReq.Request()
        request.data = 'PlantList'

        self.map_info_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))
