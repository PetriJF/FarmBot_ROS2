"""
ROS2 FarmBot state module.

Provides helper methods to interact with FarmBot state commands, including
emergency stop, abort, reset, software version, current position and end stops
requests through ROS2 services.
"""
import asyncio

from farmbot_utils.exceptions import ServerError

from rclpy.node import Node

from std_srvs.srv import Trigger


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

    def _send(self, cmd_name: str, client, on_done=None):
        """Check the server, send a Trigger request and complete via _complete."""
        self._server_availability(cmd_name, client)
        client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._complete(future, on_done))

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
        elif response.message:
            self.node.get_logger().info(response.message)
        else:
            self.node.get_logger().info('Command successful')
        if on_done is not None:
            on_done(response)

    # Service Client
    def estop(self, on_done=None):
        """
        Call the FarmBot estop service.

        Send an estop request to the FarmBot in order to immediately halt all robot
        operations.
        """
        self._send('Estop', self.estop_client, on_done)

    def reset_estop(self, on_done=None):
        """
        Call the FarmBot reset emergency stop service.

        Sends a request to reset the emergency stop state.
        """
        self._send('Reset estop', self.resume_client, on_done)

    def abort_movement(self, on_done=None):
        """
        Call the FarmBot abort service.

        Sends a request to abort the current robot movement.
        """
        self._send('Abort', self.abort_client, on_done)

    def timer_pause(self, tick_delay: float, on_done=None):
        timer = self.node.create_timer(
            tick_delay,
            lambda: self._timer_done(timer, on_done)
        )

    def _timer_done(self, timer, on_done=None):
        timer.cancel()

        if on_done:
            on_done()

    def request_end_stop(self, on_done=None):
        """
        Call the FarmBot end stop request service.

        Sends a request to retrieve the current end stop states.
        """
        self._send('End stop request', self.end_stop_client, on_done)

    def request_sw_version(self, on_done=None):
        """
        Call the FarmBot software version request service.

        Sends a request to retrieve the current software version.
        """
        self._send('Software version request', self.sw_version_client, on_done)

    def request_curr_pos(self, on_done=None):
        """
        Call the FarmBot current position request service.

        Sends a request to retrieve the current robot position.
        """
        self._send('Current position request', self.curr_position_client, on_done)
