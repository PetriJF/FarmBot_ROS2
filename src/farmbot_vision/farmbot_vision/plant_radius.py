"""
Plant radius node for the ROS2 Farmbot vision system.

A simple ROS2 node that captures a single top-down frame through the camera service, measures a
plant's exclusion radius with a green colour mask, and writes the result to the map.
"""
import cv2

from farmbot_interfaces.srv import CaptureImage, MeasurePlantRadius, StringRepReq

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


def to_bgr_image(msg) -> np.ndarray:
    """Decode a bgr8 sensor_msgs/Image into a BGR array, the inverse of to_image_msg."""
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


def green_mask(image, hsv_min, hsv_max) -> np.ndarray:
    """Threshold the image to a binary mask of likely plant (green) pixels."""
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, np.array(hsv_min), np.array(hsv_max))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def measure_plant_radius_px(mask, min_area_px) -> float | None:
    """Return the greatest distance from the frame centre to any qualifying green pixel."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [contour for contour in contours if cv2.contourArea(contour) >= min_area_px]
    if not contours:
        return None

    # Measured from the frame centre, not the blob's own centre, since the plant is assumed
    # centred in frame and this radius is later combined with its known map position.
    points = np.vstack(contours).reshape(-1, 2)
    height, width = mask.shape[:2]
    centre = np.array([width / 2.0, height / 2.0])
    return float(np.linalg.norm(points - centre, axis=1).max())


class PlantRadiusNode(Node):
    """Measure a plant's exclusion radius from a single top-down camera capture."""

    def __init__(self):
        """Initialize the plant radius node and its camera/map service clients."""
        super().__init__('PlantRadius')

        # Configurable parameters
        self.hsv_min = self.declare_parameter('hsv_min', [40, 50, 50]).value
        self.hsv_max = self.declare_parameter('hsv_max', [90, 255, 255]).value
        self.min_contour_area_px = self.declare_parameter('min_contour_area_px', 100).value
        self.mm_per_pixel = self.declare_parameter('mm_per_pixel', 0.5).value
        self.plant_radius_padding_mm = self.declare_parameter(
            'plant_radius_padding_mm', 20.0).value

        # A single callback group lets the awaited camera/capture call complete while this
        # node's own service call is still pending, matching the serial_controller convention.
        callback_group = ReentrantCallbackGroup()

        self.capture_client = self.create_client(
            CaptureImage, 'camera/capture', callback_group=callback_group)
        self.map_client = self.create_client(
            StringRepReq, 'map_info', callback_group=callback_group)

        self.measure_service = self.create_service(
            MeasurePlantRadius, 'plant_radius', self.measure_callback,
            callback_group=callback_group)

        self.get_logger().info('Plant Radius Node initialized...')

    async def measure_callback(
            self, request: MeasurePlantRadius.Request,
            response: MeasurePlantRadius.Response) -> MeasurePlantRadius.Response:
        """Capture a frame above the plant and measure its exclusion radius."""
        if not self.capture_client.service_is_ready():
            response.success = False
            response.message = 'Camera capture service not available'
            return response

        capture = await self.capture_client.call_async(CaptureImage.Request())
        if not capture.success:
            response.success = False
            response.message = f'Capture failed: {capture.message}'
            return response

        image = to_bgr_image(capture.image)
        mask = green_mask(image, self.hsv_min, self.hsv_max)
        radius_px = measure_plant_radius_px(mask, self.min_contour_area_px)

        if radius_px is None:
            response.success = False
            response.no_plant_detected = True
            response.message = 'No plant detected in the image'
            return response

        plant_radius = radius_px * self.mm_per_pixel + self.plant_radius_padding_mm
        response.plant_radius = plant_radius

        if not await self.update_map(request.index, plant_radius):
            response.success = False
            response.message = 'Plant radius measured but map update failed'
            return response

        response.success = True
        response.message = 'Plant radius measured'
        return response

    async def update_map(self, index: int, plant_radius: float) -> bool:
        """Push the measured radius to the map handler for persistence."""
        if not self.map_client.service_is_ready():
            self.get_logger().warn('Map service not available; radius measurement not saved')
            return False

        request = StringRepReq.Request()
        request.data = f'PlantRadius {index} {plant_radius}'
        result = await self.map_client.call_async(request)
        if result.data != 'SUCCESS':
            self.get_logger().warn(f"Map update failed: '{result.data}'")
            return False
        return True


def main(args=None):
    """Initialize ROS2 and run the PlantRadiusNode until shutdown."""
    rclpy.init(args=args)

    node = PlantRadiusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
