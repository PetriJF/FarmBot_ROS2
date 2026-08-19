"""
Vision control utilities for ROS2 Farmbot.

ROS2 Python Module that lets the sequencer trigger a plant radius measurement
through the plant_radius vision service.
"""
from farmbot_interfaces.srv import MeasurePlantRadius

from rclpy.node import Node


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class VisionControl:
    """Vision module that extends the farmbot controller node."""

    def __init__(self, node: Node, capture_z: float):
        """Initialise the vision module, the capture height, and the ROS2 service client."""
        self.node = node
        self.capture_z = capture_z

        self.plant_radius_client = self.node.create_client(MeasurePlantRadius, 'plant_radius')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Vision module failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report to not leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        self._log_response(response)
        if on_done is not None:
            on_done(response)

    def _log_response(self, response):
        """Log a service response generically."""
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not getattr(response, 'success', True):
            self.node.get_logger().warn(f'Command failed: {getattr(response, "message", "")}')
        else:
            self.node.get_logger().info('Command successful')

    def measure_plant_radius(self, index: int, on_done=None):
        """
        Call the MeasurePlantRadius service.

        Captures a single frame above the plant and measures its exclusion radius.

        Args:
            index {int}: Plant index in the active map
            on_done {callable}: Optional completion callback
        """
        self._server_availability('MeasurePlantRadius', self.plant_radius_client)

        request = MeasurePlantRadius.Request()
        request.index = index

        self.plant_radius_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))
