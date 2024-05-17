import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
from time import sleep

CAMERA = 'USB'  # Set your camera type here

class StandardCameraNode(Node):
    '''
    Camera Node that reads packets from the standard farmbot camera 
    and publishes the RGB frames on the topic
    '''
    def __init__(self):
        super().__init__('StandardCamera')
        self.bridge_ = CvBridge()# Bridge to convert between ROS and OpenCV images

        # Initialize publishers for RGB and depth images
        self.rgb_publisher_ = self.create_publisher(Image, 'rgb_img', 10)

        # First run flag
        self.first_run_flag_ = True

        # Camera Frequency
        capture_freq = 1.0 / 24.0 # 24 frames a second
        self.camera_timer_ = self.create_timer(capture_freq, self.capture_image)

        self.get_logger().info('Standard Camera Node initialized...')

    def capture_image(self):
        '''
        Take a photo using the farmbot snake camera and publish it to /rgb_img
        '''
        WIDTH = 640
        HEIGHT = 480

        if 'NONE' in CAMERA:
            self.get_logger().error('No camera selected. Choose a camera on the device page.')
            return
        elif 'RPI' in CAMERA:
            self.get_logger().error('Raspberry Pi Camera not supported yet.')
            return
        else:  # With USB camera:
            camera_port = 0
            image_width = WIDTH
            image_height = HEIGHT
            discard_frames = 20

            camera = cv2.VideoCapture(camera_port)
            sleep(0.1)
            try:
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
            except AttributeError:
                camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, image_width)
                camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, image_height)

            if self.first_run_flag_:
                for _ in range(discard_frames):
                    camera.grab()
                self.first_run_flag_ = False

            ret, image = camera.read()
            camera.release()


            if not ret:
                self.get_logger().error('Problem getting image.')
                return

            image_msg = self.bridge_.cv2_to_imgmsg(image, "bgr8")
            self.rgb_publisher_.publish(image_msg)

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    cam = StandardCameraNode()

    try:
        rclpy.spin(cam)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()