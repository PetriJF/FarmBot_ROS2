import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.rgb_sub = self.create_subscription(Image, '/rgb_img', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_img', self.depth_callback, 10)
        self.rgb_image = None
        self.depth_image = None

        # Timer to wait a bit for the first images to arrive and then save
        self.timer = self.create_timer(1.0, self.save_and_shutdown)  # Wait 1 second

    def rgb_callback(self, msg):
        if self.rgb_image is None:  # Only save the first image
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        if self.depth_image is None:  # Only save the first image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def save_and_shutdown(self):
        if self.rgb_image is not None and self.depth_image is not None:
            cv2.imwrite("saved_rgb_image.png", self.rgb_image)
            cv2.imwrite("saved_depth_image.png", self.depth_image)
            self.get_logger().info('Images saved successfully.')
        else:
            self.get_logger().info('No images to save.')
        
        # Cancel the timer to prevent repeated saving
        self.timer.cancel()
        
        # Shutdown the ROS 2 node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)  # This will exit once `rclpy.shutdown()` is called

if __name__ == '__main__':
    main()
