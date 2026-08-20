"""
Vision control utilities for ROS2 Farmbot.

ROS2 Python Module that lets the sequencer trigger a plant radius measurement or a bed
stitch capture through the plant_radius and map/stitch vision services.
"""
from farmbot_interfaces.srv import MeasurePlantRadius, StitchImage

from rclpy.node import Node


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class VisionControl:
    """Vision module that extends the farmbot controller node."""

    def __init__(self, node: Node, radius_capture_z: float,
                 stitch_capture_z: float, stitch_step_mm: float,
                 camera_offset_x: float, camera_offset_y: float):
        """Initialise the vision module, the capture heights/step/offset, and the clients."""
        self.node = node
        self.radius_capture_z = radius_capture_z
        self.stitch_capture_z = stitch_capture_z
        self.stitch_step_mm = stitch_step_mm
        self.camera_offset_x = camera_offset_x
        self.camera_offset_y = camera_offset_y

        self.plant_radius_client = self.node.create_client(MeasurePlantRadius, 'plant_radius')
        self.stitch_client = self.node.create_client(StitchImage, 'map/stitch')

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
        """Request a radius update for one plant."""
        self._server_availability('MeasurePlantRadius', self.plant_radius_client)

        request = MeasurePlantRadius.Request()
        request.index = index

        self.plant_radius_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def stitch(self, x: float, y: float, on_done=None):
        """Request one frame be stitched at the given gantry position."""
        self._server_availability('StitchImage', self.stitch_client)

        request = StitchImage.Request()
        request.x = x
        request.y = y

        self.stitch_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))
