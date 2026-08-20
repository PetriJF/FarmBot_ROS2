"""
Image stitching node for the ROS2 Farmbot vision system.

A simple ROS2 node that captures a single frame at a known gantry position and blends it onto a
persistent map of the raised bed. The gantry's absolute position and a calibrated mm-to-px
transform place every frame directly, so no feature matching between frames is needed.
"""
import os

import cv2

from farmbot_interfaces.srv import CaptureImage, StitchImage, StringRepReq

from farmbot_vision.camera_calibration import CALIBRATION_FILE
from farmbot_vision.plant_radius import to_bgr_image

import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import yaml

MAP_FILE = 'bed_map.png'


def canvas_size(bed_size_mm, map_mm_per_px) -> tuple:
    """Return the (width, height) in pixels of a canvas covering the whole bed."""
    return (int(np.ceil(bed_size_mm[0] / map_mm_per_px)),
            int(np.ceil(bed_size_mm[1] / map_mm_per_px)))


def map_affine(camera_xy_mm, mm_to_px, frame_shape, map_mm_per_px, map_height_px) -> np.ndarray:
    """
    Build the 2x3 transform taking frame pixels to map pixels.

    Composes the calibrated px-to-mm transform, the camera's known bed position, and the map's
    own scale into one matrix, so a single warp does rotation, scale, any axis flip and the
    translation together.
    """
    px_to_mm = np.linalg.inv(np.asarray(mm_to_px, dtype=float))
    # The map image runs top down while the bed's y axis runs up, hence the flipped y scale.
    to_map = np.diag([1.0 / map_mm_per_px, -1.0 / map_mm_per_px]) @ px_to_mm
    frame_centre = np.array([frame_shape[1] / 2.0, frame_shape[0] / 2.0])
    origin = np.array([camera_xy_mm[0] / map_mm_per_px,
                       map_height_px - camera_xy_mm[1] / map_mm_per_px])
    return np.column_stack((to_map, origin - to_map @ frame_centre))


def feather_alpha(frame_shape, crop_fraction, feather_px) -> np.ndarray:
    """
    Build a frame's blend weights: solid in the middle, ramping to zero at the edges.

    The calibration is affine only, with no lens distortion model, so the frame's outer border is
    where placement is least trustworthy. Cropping it away and fading what remains is what keeps
    neighbouring frames from showing a seam where they meet.
    """
    height, width = frame_shape[:2]
    # At least one pixel is always discarded, so the ramp has a border to measure from.
    margin_x = max(1, int(round(width * (1.0 - crop_fraction) / 2.0)))
    margin_y = max(1, int(round(height * (1.0 - crop_fraction) / 2.0)))

    kept = np.zeros((height, width), dtype=np.uint8)
    kept[margin_y:height - margin_y, margin_x:width - margin_x] = 255
    if feather_px <= 0:
        return kept.astype(np.float32) / 255.0

    # Every kept pixel's distance from the discarded border ramps the weight to zero over
    # exactly the feather width.
    distance = cv2.distanceTransform(kept, cv2.DIST_L2, 3)
    return np.clip(distance / float(feather_px), 0.0, 1.0).astype(np.float32)


def blend_over(canvas, tile, alpha, origin_xy):
    """
    Composite one warped frame onto the canvas in place, clipped to the canvas bounds.

    Alpha-over normalised by the accumulated coverage, so a feathered edge fades into whatever
    was photographed before it rather than fading towards the blank canvas.
    """
    left_offset, top_offset = origin_xy
    height, width = alpha.shape[:2]
    canvas_height, canvas_width = canvas.shape[:2]

    left, top = max(left_offset, 0), max(top_offset, 0)
    right = min(left_offset + width, canvas_width)
    bottom = min(top_offset + height, canvas_height)
    if right <= left or bottom <= top:
        return

    on_tile = (slice(top - top_offset, bottom - top_offset),
               slice(left - left_offset, right - left_offset))
    on_canvas = (slice(top, bottom), slice(left, right))

    source_alpha = alpha[on_tile][..., None]
    destination = canvas[on_canvas].astype(np.float32)
    destination_alpha = destination[..., 3:4] / 255.0

    out_alpha = source_alpha + destination_alpha * (1.0 - source_alpha)
    colour = (tile[on_tile].astype(np.float32) * source_alpha
              + destination[..., :3] * destination_alpha * (1.0 - source_alpha))
    colour = np.divide(colour, out_alpha, out=np.zeros_like(colour), where=out_alpha > 0.0)

    canvas[on_canvas] = np.concatenate(
        (np.clip(colour, 0.0, 255.0), np.clip(out_alpha * 255.0, 0.0, 255.0)),
        axis=2).astype(np.uint8)


def stitch_frame(canvas, frame, affine, crop_fraction, feather_px):
    """Warp one frame into map space and blend it onto the canvas in place."""
    height, width = frame.shape[:2]
    corners = np.array([[0.0, 0.0], [width, 0.0], [width, height], [0.0, height]])
    placed = corners @ affine[:, :2].T + affine[:, 2]

    left, top = np.floor(placed.min(axis=0)).astype(int)
    right, bottom = np.ceil(placed.max(axis=0)).astype(int)
    tile_size = (int(right - left), int(bottom - top))
    if tile_size[0] <= 0 or tile_size[1] <= 0:
        return

    # Warp into a tile the size of the placed frame rather than the whole bed sized canvas.
    shifted = affine.copy()
    shifted[:, 2] -= (left, top)
    tile = cv2.warpAffine(frame, shifted, tile_size, flags=cv2.INTER_LINEAR)
    alpha = cv2.warpAffine(feather_alpha(frame.shape, crop_fraction, feather_px),
                           shifted, tile_size, flags=cv2.INTER_LINEAR)

    blend_over(canvas, tile, alpha, (left, top))


class ImageStitcherNode(Node):
    """Place single camera frames onto a persistent map of the raised bed."""

    def __init__(self):
        """Initialize the stitcher node, its service clients and the tuning parameters."""
        super().__init__('ImageStitcher')

        # Shared with map_controller so the map lands beside the active map.
        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)
        self.config_path = os.path.join(
            self.get_parameter('ws_path').get_parameter_value().string_value,
            self.get_parameter('folder_config_name').get_parameter_value().string_value)
        self.map_path = os.path.join(self.config_path, MAP_FILE)

        # Configurable parameters
        self.map_mm_per_px = self.declare_parameter('map_mm_per_px', 0.5).value
        self.feather_px = self.declare_parameter('feather_px', 40).value
        self.frame_crop_fraction = self.declare_parameter('frame_crop_fraction', 0.8).value
        self.camera_offset_x = self.declare_parameter('camera_offset_x', 18.0).value
        self.camera_offset_y = self.declare_parameter('camera_offset_y', 65.0).value

        # A single callback group lets the awaited camera/capture call complete while this
        # node's own service call is still pending, matching the plant_radius convention.
        callback_group = ReentrantCallbackGroup()

        self.capture_client = self.create_client(
            CaptureImage, 'camera/capture', callback_group=callback_group)
        self.map_client = self.create_client(
            StringRepReq, 'map_info', callback_group=callback_group)

        self.stitch_service = self.create_service(
            StitchImage, 'map/stitch', self.stitch_callback, callback_group=callback_group)

        self.get_logger().info('Image Stitcher Node initialized...')

    async def stitch_callback(self, request: StitchImage.Request,
                              response: StitchImage.Response) -> StitchImage.Response:
        """Capture a frame at the given gantry position and blend it onto the bed map."""
        if not self.capture_client.service_is_ready():
            response.success = False
            response.message = 'Camera capture service not available'
            return response

        try:
            mm_to_px = self.load_calibration()
            bed_size = await self.bed_size_mm()
            canvas = self.load_canvas(bed_size)
        except (OSError, TypeError, ValueError, KeyError) as error:
            response.success = False
            response.message = str(error)
            return response

        capture = await self.capture_client.call_async(CaptureImage.Request())
        if not capture.success:
            response.success = False
            response.message = f'Capture failed: {capture.message}'
            return response

        frame = to_bgr_image(capture.image)
        camera_xy = (request.x + self.camera_offset_x, request.y + self.camera_offset_y)
        affine = map_affine(camera_xy, mm_to_px, frame.shape,
                            self.map_mm_per_px, canvas.shape[0])
        stitch_frame(canvas, frame, affine, self.frame_crop_fraction, self.feather_px)
        if not cv2.imwrite(self.map_path, canvas):
            response.success = False
            response.message = f'Failed to write bed map to {self.map_path}'
            return response

        response.success = True
        response.message = (f'Frame stitched at camera position '
                            f'({camera_xy[0]:.1f}, {camera_xy[1]:.1f})')
        return response

    def load_calibration(self) -> np.ndarray:
        """Read the calibrated mm-to-px transform."""
        path = os.path.join(self.config_path, CALIBRATION_FILE)
        with open(path, 'r') as calibration_file:
            calibration = yaml.safe_load(calibration_file)

        return np.array(calibration['mm_to_px'], dtype=float)

    async def bed_size_mm(self) -> tuple:
        """Ask the map handler for the bed dimensions in millimetres."""
        if not self.map_client.service_is_ready():
            raise ValueError('Map service not available')

        request = StringRepReq.Request()
        request.data = 'MapSize'
        result = await self.map_client.call_async(request)

        x_len, y_len = (float(value) for value in result.data.split(' '))
        if x_len <= 0.0 or y_len <= 0.0:
            raise ValueError(f'Bed dimensions are not set on the map ({x_len} x {y_len})')
        return (x_len, y_len)

    def load_canvas(self, bed_size_mm) -> np.ndarray:
        """Load the persistent bed map, starting a blank one the first time round."""
        width, height = canvas_size(bed_size_mm, self.map_mm_per_px)
        if not os.path.exists(self.map_path):
            os.makedirs(self.config_path, exist_ok=True)
            return np.zeros((height, width, 4), dtype=np.uint8)

        canvas = cv2.imread(self.map_path, cv2.IMREAD_UNCHANGED)
        if canvas is None:
            raise ValueError(f'{self.map_path} could not be read; delete it to start again')
        if canvas.shape[:2] != (height, width):
            raise ValueError(
                f'{self.map_path} is {canvas.shape[1]}x{canvas.shape[0]}px but the bed now '
                f'needs {width}x{height}px; delete it to start a fresh map')
        return canvas


def main(args=None):
    """Initialize ROS2 and run the ImageStitcherNode until shutdown."""
    rclpy.init(args=args)

    node = ImageStitcherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
