import cv2
from datetime import datetime
from rclpy.node import Node

class PinholeCapture:
    '''
    Class used for taking pictures with a USB camera.
    '''
    def __init__(self, node: Node):
        self.node_ = node
        self.cap_ = cv2.VideoCapture(0)
        if not self.cap_.isOpened():
            self.node_.get_logger().error("Failed to open USB camera.")
    
    def save_image(self, num: int, name: str):
        """
        Capture a frame and save it with today's date in the filename
        in format YYYY_MM_DD_output_XXX.png
        """
        ret, frame = self.cap_.read()
        if ret:
            # Prepare filename with date prefix
            date_str = datetime.now().strftime("%Y_%m_%d")
            filename = f"{date_str}_capture_{num:03}_{name}.png"
            cv2.imwrite(filename, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            self.node_.get_logger().info(f"Saved image {filename}")
        else:
            self.node_.get_logger().error("Error: Failed to capture image.")

    def release(self):
        self.cap_.release()
        self.node_.get_logger().info("Camera released.")
