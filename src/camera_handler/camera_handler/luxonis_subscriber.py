import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2

class OakCameraSubscriber(Node):
    def __init__(self):
        super().__init__('oak_camera_subscriber')
        self.bridge = CvBridge()
        self.subscription_color = self.create_subscription(
            Image,
            'color/image_raw',  # Update this topic name
            self.listener_callback_color,
            10)
        self.subscription_left = self.create_subscription(
            Image,
            'left/image_raw',  # Update this topic name
            self.listener_callback_left,
            10)
        self.subscription_right = self.create_subscription(
            Image,
            'right/image_raw',  # Update this topic name
            self.listener_callback_right,
            10)
        self.subscription_imu = self.create_subscription(
            Imu,
            'imu/data',  # Update this topic name
            self.listener_callback_imu,
            10)

    def listener_callback_color(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process color image here
        cv2.imshow("Color Image", cv_image)
        cv2.waitKey(1)

    def listener_callback_left(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process left image here

    def listener_callback_right(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process right image here

    def listener_callback_imu(self, msg):
        # Process IMU data here
        pass

def main(args=None):
    rclpy.init(args=args)
    oak_camera_subscriber = OakCameraSubscriber()
    rclpy.spin(oak_camera_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    oak_camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
