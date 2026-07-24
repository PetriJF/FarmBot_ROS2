"""
ROS2 FarmBot state module.

Provides helper methods to interact with FarmBot state commands, including
emergency stop, abort, reset, software version, current position and end stops
requests through ROS2 services.
"""
from rclpy.node import Node

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class State:
    """State module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the state module and the ROS2 service clients."""
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
        self._send_request(client)

    def _send_request(self, client):
        request = Trigger.Request()
        future = client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    # Service Client
    def estop(self):
        """
        Call the FarmBot estop service.

        Send an estop request to the FarmBot in order to immediately halt all robot
        operations.
        """
        self._server_availability('Estop', self.estop_client)

    def abort_movement(self):
        """
        Call the FarmBot abort service.

        Sends a request to abort the current robot movement.
        """
        self._server_availability('Abort', self.abort_client)

    def reset_estop(self):
        """
        Call the FarmBot reset emergency stop service.

        Sends a request to reset the emergency stop state.
        """
        self._server_availability('Reset estop', self.resume_client)

    def request_end_stop(self):
        """
        Call the FarmBot end stop request service.

        Sends a request to retrieve the current end stop states.
        """
        self._server_availability('End stop request', self.end_stop_client)

    def request_sw_version(self):
        """
        Call the FarmBot software version request service.

        Sends a request to retrieve the current software version.
        """
        self._server_availability('Software version request', self.sw_version_client)

    def request_curr_pos(self):
        """
        Call the FarmBot current position request service.

        Sends a request to retrieve the current robot position.
        """
        self._server_availability('Current position request', self.curr_position_client)

    def client_callback(self, future):
        """
        Handle the response of a service client request.

        Processes the service response and logs the command status depending
        on the result of the request.
        """
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
