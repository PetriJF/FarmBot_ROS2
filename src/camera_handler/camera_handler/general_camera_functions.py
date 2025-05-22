import cv2
from datetime import datetime
import threading

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class GeneralCameraFunctions:
    """
    Utility for grabbing a single RGB frame from the Luxonis camera topic
    and saving it to disk on demand.
    """
    def __init__(self, node: Node):
        self.node_ = node                     # Store parent node for logging & ROS APIs
        self.bridge = CvBridge()
        self.latest_frame = None              # Last frame received (np.ndarray)
        self._lock = threading.Lock()

        # Subscribe to the RGB image stream published by LuxonisCameraNode
        self.sub_ = self.node_.create_subscription(
            Image,
            "rgb",
            self._rgb_callback,
            qos_profile_sensor_data     # Best-effort, low latency
        )
        self.node_.get_logger().info("GeneralCameraFunctions: RGB subscriber ready")

    # ---------- Callbacks ---------- #
    def _rgb_callback(self, msg: Image):
        """
        Runs every time a new RGB frame arrives.
        Convert to OpenCV BGR ndarray and cache it.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self.latest_frame = frame.copy()
        except Exception as e:
            self.node_.get_logger().warn(f"Bridge conversion failed: {e}")

    # ---------- Public API ---------- #
    def save_weed_image(self, num: int, name: str):
        """
        Save the most recent frame to YYYY_MM_DD_capture_###_<name>.png,
        compression level 0 (= lossless).
        """
        with self._lock:
            if self.latest_frame is None:
                self.node_.get_logger().warn("No RGB image received yet.")
                return
            frame_to_save = self.latest_frame.copy()

        date_str = datetime.now().strftime("%Y_%m_%d")
        filename = f"{date_str}_capture_{num:03}_{name}.png"

        cv2.imwrite(filename, frame_to_save, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        self.node_.get_logger().info(f"Saved image {filename}")
