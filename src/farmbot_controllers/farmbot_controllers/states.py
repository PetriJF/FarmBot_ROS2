"""
ROS2 Farmbot state module.

Provides state command helpers for emergency stop, abort, reset, and other
command-handler interactions via the farmbot_command publisher.
"""
from rclpy.node import Node

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class State:
    """
    ROS2 Farmbot State module.

    Forms the commands that are sent to the command handling package
    """

    def __init__(self, node: Node):
        """
        ROS2 Farmbot State Module Constructor.

        Args:
            node {Node}: The node the module extends
        """
        self.node = node

        self.estop_client = self.node.create_client(Trigger, 'estop')
        self.abort_client = self.node.create_client(Trigger, 'abort')
        self.resume_client = self.node.create_client(Trigger, 'resume')
        self.end_stop_client = self.node.create_client(Trigger, 'end_stop')
        self.sw_version_client = self.node.create_client(Trigger, 'sw_version')
        self.curr_position_client = self.node.create_client(Trigger, 'curr_pos')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('States module failed: server unavailable')
        self.send_request(client)

    def send_request(self, client):
        """Send a Trigger service request asynchronously."""
        request = Trigger.Request()
        future = client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    # Service Client
    def estop(self):
        """Service client for estop."""
        self._server_availability('Estop', self.estop_client)

    def abort_movement(self):
        """Service client for abort."""
        self._server_availability('Abort', self.abort_client)

    def reset_estop(self):
        """Service client for abort."""
        self._server_availability('Reset estop', self.resume_client)

    def request_end_stop(self):
        """Service client to request the end stops."""
        self._server_availability('End stop request', self.end_stop_client)

    def request_sw_version(self):
        """Service client to request the software version."""
        self._server_availability('Software version request', self.sw_version_client)

    def request_curr_pos(self):
        """Service client to request the current position."""
        self._server_availability('Current position request', self.curr_position_client)

    def client_callback(self, future):
        """Service client callback once the request is send."""
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            elif response.message:
                self.node.get_logger().info(response.message)

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))
