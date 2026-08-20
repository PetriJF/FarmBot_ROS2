"""
Camera calibration node for the ROS2 Farmbot vision system.

A simple ROS2 node that photographs the FarmBot dot grid calibration card at three known gantry
offsets and solves the millimetre-to-pixel transform that the stitcher uses to place frames.
"""
import os

import cv2

from farmbot_interfaces.srv import CalibrateCamera, CaptureImage

from farmbot_vision.plant_radius import to_bgr_image

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import yaml

# The official FarmBot calibration card: 7 staggered rows of 5 circles, 30mm apart within a row.
PATTERN_SIZE = (5, 7)
ROW_SEPARATION_MM = 30.0

# Gantry offsets, relative to where the run starts, that each capture is taken from.
CALIBRATION_OFFSETS_MM = ((0.0, 0.0), (0.0, 50.0), (50.0, 50.0))

CALIBRATION_FILE = 'camera_calibration.yaml'


def preprocess(image, basic: bool, invert: bool) -> np.ndarray:
    """Prepare a frame for circle detection; the heavier pass rescues a low contrast card."""
    def odd_fraction(factor: int) -> int:
        """Return a fraction of the image height, forced odd for the OpenCV kernel sizes."""
        size = int(image.shape[0] / factor)
        return size + 1 if size % 2 == 0 else size

    processed = cv2.bitwise_not(image) if invert else image
    processed = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)
    processed = cv2.medianBlur(processed, odd_fraction(100))
    if basic:
        return processed

    processed = cv2.adaptiveThreshold(processed, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                      cv2.THRESH_BINARY, odd_fraction(15), 2)
    kernel = np.ones((odd_fraction(50), odd_fraction(50)), np.uint8)
    processed = cv2.morphologyEx(processed, cv2.MORPH_DILATE, kernel, iterations=1)
    return cv2.medianBlur(processed, odd_fraction(100))


def find_circle_grid(image) -> np.ndarray | None:
    """Locate the calibration card's circle centres, returning them as an (N, 2) array."""
    # findCirclesGrid only ever looks for dark blobs, and the printed card's polarity is not
    # known ahead of time, so both polarities are tried before giving up.
    for invert in (False, True):
        for basic in (True, False):
            found, centres = cv2.findCirclesGrid(
                preprocess(image, basic, invert), PATTERN_SIZE,
                flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
            if found:
                return centres.reshape(-1, 2).astype(float)
    return None


def card_scale(centres) -> float:
    """Estimate millimetres per pixel from the known spacing of the first row's circles."""
    row_length_mm = (PATTERN_SIZE[0] - 1) * ROW_SEPARATION_MM
    row_length_px = np.linalg.norm(centres[PATTERN_SIZE[0] - 1] - centres[0])
    return float(row_length_mm / row_length_px)


def solve_calibration(centres) -> dict:
    """
    Solve the millimetre-to-pixel transform from three captures of a stationary card.

    The card never moves, so a gantry move of `d` shifts every detected circle by `mm_to_px @ -d`.
    Each axis' move therefore hands us one column of the transform directly, which is why no
    feature matching between the frames is needed.

    `centres` holds the (N, 2) circle centres detected at each of CALIBRATION_OFFSETS_MM, in
    order. Returns 'mm_to_px' and a scale estimate from the card geometry.
    """
    captures = [np.asarray(capture, dtype=float) for capture in centres]
    if len(captures) != len(CALIBRATION_OFFSETS_MM) or len({c.shape for c in captures}) != 1:
        raise ValueError('expected three captures holding the same number of circles')

    first, second, third = captures
    y_move = CALIBRATION_OFFSETS_MM[1][1] - CALIBRATION_OFFSETS_MM[0][1]
    x_move = CALIBRATION_OFFSETS_MM[2][0] - CALIBRATION_OFFSETS_MM[1][0]
    y_shift = (second - first).mean(axis=0)
    x_shift = (third - second).mean(axis=0)

    mm_to_px = np.column_stack((-x_shift / x_move, -y_shift / y_move))
    determinant = np.linalg.det(mm_to_px)
    if np.isclose(determinant, 0.0):
        raise ValueError('the two gantry moves produced no distinct shift in the image')

    return {
        'mm_to_px': mm_to_px,
        'mm_per_px': float(np.sqrt(1.0 / abs(determinant))),
        'rotation_deg': float(np.degrees(np.arctan2(mm_to_px[1, 0], mm_to_px[0, 0]))),
        'mm_per_px_card': card_scale(first),
    }


class CameraCalibrationNode(Node):
    """Solve the camera's millimetre-to-pixel transform from three calibration card captures."""

    def __init__(self):
        """Initialize the calibration node, its camera client and the config directory."""
        super().__init__('CameraCalibration')

        # Shared with map_controller so the calibration lands beside the active map.
        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)
        self.config_path = os.path.join(
            self.get_parameter('ws_path').get_parameter_value().string_value,
            self.get_parameter('folder_config_name').get_parameter_value().string_value)

        # A single callback group lets the awaited camera/capture call complete while this
        # node's own service call is still pending, matching the plant_radius convention.
        callback_group = ReentrantCallbackGroup()

        self.capture_client = self.create_client(
            CaptureImage, 'camera/capture', callback_group=callback_group)

        self.calibrate_service = self.create_service(
            CalibrateCamera, 'camera/calibrate', self.calibrate_callback,
            callback_group=callback_group)

        self.captures = {}

        self.get_logger().info('Camera Calibration Node initialized...')

    def check_step(self, step: int) -> str:
        """Return why this step cannot be captured right now, or an empty string if it can."""
        if step < 1 or step > len(CALIBRATION_OFFSETS_MM):
            return f'Step must be 1 to {len(CALIBRATION_OFFSETS_MM)}, got {step}'
        if not self.capture_client.service_is_ready():
            return 'Camera capture service not available'
        if step > 1 and step - 1 not in self.captures:
            return f'Step {step - 1} has not been captured; start again from step 1'
        return ''

    async def calibrate_callback(
            self, request: CalibrateCamera.Request,
            response: CalibrateCamera.Response) -> CalibrateCamera.Response:
        """Photograph the card at one calibration offset, solving once all three are in."""
        rejection = self.check_step(request.step)
        if rejection:
            response.success = False
            response.message = rejection
            return response

        if request.step == 1:
            self.captures.clear()

        capture = await self.capture_client.call_async(CaptureImage.Request())
        if not capture.success:
            response.success = False
            response.message = f'Capture failed: {capture.message}'
            return response

        centres = find_circle_grid(to_bgr_image(capture.image))
        if centres is None:
            response.success = False
            response.message = 'Calibration card not found in the image'
            return response

        self.captures[request.step] = centres
        if request.step < len(CALIBRATION_OFFSETS_MM):
            response.success = True
            response.message = (f'Captured step {request.step}; move to the next offset '
                                'and call again')
            return response

        return self.solve_and_save(response)

    def solve_and_save(self, response: CalibrateCamera.Response) -> CalibrateCamera.Response:
        """Solve the transform from the three captures and persist it beside the active map."""
        try:
            calibration = solve_calibration(
                [self.captures[step] for step in sorted(self.captures)])
        except ValueError as error:
            self.captures.clear()
            response.success = False
            response.message = f'Calibration failed: {error}'
            return response
        self.captures.clear()

        self.warn_on_scale_mismatch(calibration)
        path = self.save_calibration(calibration)
        response.success = True
        response.message = (f'Calibration saved to {path} '
                            f"({calibration['mm_per_px']:.4f} mm/px, "
                            f"{calibration['rotation_deg']:.2f} deg rotation)")
        return response

    def warn_on_scale_mismatch(self, calibration: dict):
        """Cross-check the solved scale against the card's own geometry."""
        solved = calibration['mm_per_px']
        from_card = calibration['mm_per_px_card']
        if abs(solved - from_card) > 0.05 * from_card:
            self.get_logger().warn(
                f'Scale from the gantry moves ({solved:.4f} mm/px) disagrees with the scale '
                f'from the card geometry ({from_card:.4f} mm/px). Check the card is flat and '
                'fully in frame, and that each step moved the gantry exactly 50mm.')

    def save_calibration(self, calibration: dict) -> str:
        """Write the solved transform to the calibration file and return its path."""
        os.makedirs(self.config_path, exist_ok=True)
        path = os.path.join(self.config_path, CALIBRATION_FILE)
        with open(path, 'w') as calibration_file:
            yaml.dump({
                'mm_to_px': [[float(value) for value in row]
                             for row in calibration['mm_to_px']],
            }, calibration_file, default_flow_style=False)
        return path


def main(args=None):
    """Initialize ROS2 and run the CameraCalibrationNode until shutdown."""
    rclpy.init(args=args)

    node = CameraCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
