"""
Camera node for the standard pinhole camera that is equipped on the farmbots.

A simple ROS2 node that captures frames from the standard FarmBot USB camera and publishes
RGB images on the `camera/image_raw` topic via a timer.
"""

from time import sleep

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from farmbot_interfaces.srv import CaptureImage, EnableCamera

from sensor_msgs.msg import Image

CAMERA = 'USB'  # Set your camera type here


def fourcc_code(fmt):
    """Return the OpenCV FOURCC integer for a 4-character format string (e.g. 'MJPG')."""
    try:
        return cv2.VideoWriter_fourcc(*fmt)
    except AttributeError:
        return cv2.VideoWriter.fourcc(*fmt)


def fourcc_to_str(value):
    """Convert an OpenCV FOURCC integer back into its 4-character string form."""
    value = int(value)
    return ''.join(chr((value >> (8 * i)) & 0xFF) for i in range(4))


class StandardCameraNode(Node):
    """Camera Node reads packets from the standard farmbot camera and publishes the RGB frames."""

    def __init__(self):
        """Initialize the standard camera node and start the capture timer."""
        super().__init__('StandardCamera')

        # Configurable parameters
        self.camera_port = self.declare_parameter('camera_index', 0).value
        self.width = self.declare_parameter('image_width', 640).value
        self.height = self.declare_parameter('image_height', 480).value
        self.fps = self.declare_parameter('frame_rate', 30.0).value
        self.pixel_format = self.declare_parameter('pixel_format', 'MJPG').value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        self.auto_exposure = self.declare_parameter('auto_exposure', False).value
        self.exposure = self.declare_parameter('exposure', 156.0).value

        self.camera_active = False

        self.rgb_publisher = self.create_publisher(
            Image, 'camera/image_raw', qos_profile_sensor_data)
        
        self.capture_publisher = self.create_publisher(
            Image, 'camera/image_raw_capture', qos_profile_system_default)
        
        self.enable_service = self.create_service(
            EnableCamera, 'camera/enable', self.enable_camera_callback)

        self.capture_service = self.create_service(
            CaptureImage, 'camera/capture', self.capture_callback)

        self.camera = None

        if not self.init_camera():
            return

        # A timer drives capture; read() blocks at the camera's cadence, pacing us to the fps.
        period = 1.0 / self.fps if self.fps > 0 else 1.0 / 30.0
        self.capture_timer = self.create_timer(period, self.capture_frame)

        self.get_logger().info('Standard Camera Node initialized...')
    
    def enable_camera_callback(self, request, response):
        """Turn continous streaming on (enable=true) or off (enable=false)."""
        self.camera_active = request.enable

        if self.camera_active:
            self.get_logger().debug('Camera enabled')
            response.message = 'Camera enabled'
        else:
            self.get_logger().debug('Camera disabled')
            response.message = 'Camera disabled'

        response.success = True
        return response

    def capture_callback(self, request, response):
        """Capture and publish a single frame."""
        ret, image = self.camera.read()
        if not ret:
            self.get_logger().warn('Single capture failed.')
            response.success = False
            response.message = 'Failed to capture frame'
            return response

        image_msg = self.to_image_msg(image)
        self.capture_publisher.publish(image_msg)
        self.get_logger().debug('Single frame captured and published.')
        response.success = True
        response.message = 'Frame captured and published'
        response.image = image_msg
        return response

    def init_camera(self):
        """Open and configure the camera. Return True on success."""
        self.discard_frames = 5  # Warm-up frames to flush before publishing

        # Force the V4L2 backend so the property/format calls below are honoured.
        self.camera = cv2.VideoCapture(self.camera_port, cv2.CAP_V4L2)
        if not self.camera.isOpened():
            self.get_logger().error('Error: Could not open video device.')
            return False

        if self.pixel_format:
            self.camera.set(cv2.CAP_PROP_FOURCC, fourcc_code(self.pixel_format))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)

        self.configure_exposure()

        sleep(0.1)

        actual_fmt = fourcc_to_str(self.camera.get(cv2.CAP_PROP_FOURCC))
        actual_w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera negotiated: {actual_fmt} {actual_w}x{actual_h} @ {actual_fps:.0f} fps')
        if self.pixel_format and actual_fmt != self.pixel_format:
            self.get_logger().warn(
                f"Requested '{self.pixel_format}' but driver gave '{actual_fmt}'; "
                'fps may be limited by USB bandwidth.')

        for _ in range(self.discard_frames):
            self.camera.grab()
        return True

    def configure_exposure(self):
        """Apply the exposure mode. Manual exposure is needed to reach the full frame rate."""
        if self.auto_exposure:
            self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 3 = aperture priority (auto)
            self.get_logger().info('Exposure: auto (aperture priority); fps may be limited.')
            return

        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual; must precede EXPOSURE
        if self.exposure >= 0:
            self.camera.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.get_logger().info(
            f'Exposure: manual (value={self.camera.get(cv2.CAP_PROP_EXPOSURE):.0f}).')

    def capture_frame(self):
        """Read one frame and publish it to camera/image_raw (called by the timer)."""
        if not self.camera_active:
            return
        
        ret, image = self.camera.read()
        if not ret:
            self.get_logger().warn('Problem getting image.', throttle_duration_sec=2.0)
            return

        self.rgb_publisher.publish(self.to_image_msg(image))

    def to_image_msg(self, image):
        """Build a bgr8 sensor_msgs/Image from an OpenCV frame without cv_bridge."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = image.shape[1] * image.shape[2]  # width * channels (3 bytes/px for bgr8)
        msg.data = image.tobytes()
        return msg

    def destroy_node(self):
        """Cleanup resources when shutting down the node."""
        if self.camera is not None:
            self.camera.release()
            self.get_logger().info('Camera released.')
        super().destroy_node()


def main(args=None):
    """Initialize ROS2 and run the StandardCameraNode until shutdown."""
    rclpy.init(args=args)

    cam = StandardCameraNode()

    try:
        rclpy.spin(cam)
    except KeyboardInterrupt:
        pass

    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
