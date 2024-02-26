import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import yaml
import os
import numpy as np
import depthai as dai

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()  # Bridge to convert between ROS and OpenCV images
        self.load_config()  # Load configuration from YAML file
        self.setup_camera()  # Initialize and configure the DepthAI camera
        
        # Initialize publishers for RGB and depth images
        self.rgb_publisher = self.create_publisher(Image, 'rgb_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        
        # Set a timer to process and publish images at a specified rate
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.run)

    def load_config(self):
        # Load configuration from YAML file
        directory = os.path.join(get_package_share_directory('camera_handler'), 'config')
        config_file = 'camera_config.yaml'
        self.config_data = self.load_from_yaml(directory, config_file)
        # Extract relevant configuration values
        self.HOST = self.config_data['server_constants']['HOST']
        self.PORT = self.config_data['server_constants']['PORT']
        self.DEPTH_MIN_VAL = self.config_data['camera_constants']['DEPTH_MIN_VAL']
        self.DEPTH_MAX_VAL = self.config_data['camera_constants']['DEPTH_MAX_VAL']
        self.FOCAL_LENGTH_PX = self.config_data['camera_constants']['FOCAL_LENGTH_PX']
        self.BASELINE = self.config_data['camera_constants']['BASELINE']

    def setup_camera(self):
        # Configure the DepthAI pipeline with stereo and RGB camera nodes
        self.pipeline = dai.Pipeline()
        stereo, monoLeft, monoRight, cam_rgb = self.configure_depthai_components()
        
        # Link camera outputs to XLinkOut nodes for transmission to host
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        cam_rgb.preview.link(self.xout_rgb.input)
        stereo.disparity.link(self.xout_stereo.input)
        
        self.device = dai.Device(self.pipeline)  # Connect to the DepthAI device

    def configure_depthai_components(self):
        # Create and configure DepthAI components (stereo depth, mono cameras, RGB camera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        
        # Mono camera settings
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        # RGB camera settings
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(40)
        
        # Create XLinkOut nodes for RGB and stereo depth data
        self.xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        self.xout_rgb.setStreamName("rgb")
        self.xout_stereo = self.pipeline.create(dai.node.XLinkOut)
        self.xout_stereo.setStreamName("stereo")
        
        # Configure stereo depth settings based on YAML configuration
        self.configure_stereo_depth(stereo)
        
        return stereo, monoLeft, monoRight, cam_rgb

    def configure_stereo_depth(self, stereo):
        # Apply stereo depth settings from configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(self.config_data['camera_constants']['lr_check'])
        stereo.setExtendedDisparity(self.config_data['camera_constants']['extended_disparity'])
        stereo.setSubpixel(self.config_data['camera_constants']['subpixel'])
        
        # Additional disparity post-processing
        config = stereo.initialConfig.get()
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        stereo.initialConfig.set(config)

    def publish_images(self, rgb_frame, depth_frame):
        # Publish RGB and depth frames if available
        if rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
            self.rgb_publisher.publish(rgb_msg)

        if depth_frame is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono8')
            self.depth_publisher.publish(depth_msg)

    def run(self):
        # Retrieve and publish RGB and depth frames from the camera
        latestPacket = {"rgb": None, "stereo": None}
        queueEvents = self.device.getQueueEvents(("rgb", "stereo"))
        for queueName in queueEvents:
            packets = self.device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["rgb"] is not None:
            self.publish_images(latestPacket["rgb"].getCvFrame(), None)
        if latestPacket["stereo"] is not None:
            depth_frame = self.process_depth_frame(latestPacket["stereo"].getFrame())
            self.publish_images(None, depth_frame)

    def process_depth_frame(self, disparity_frame):
        # Process disparity frame to generate a depth frame for publishing
        disparity_frame[disparity_frame == 0] = 0.01  # Handle zero disparity
        depth_frame = (self.FOCAL_LENGTH_PX * self.BASELINE) / disparity_frame
        depth_frame[(depth_frame >= self.DEPTH_MAX_VAL) | (depth_frame <= self.DEPTH_MIN_VAL)] = 0
        scaled_frame = np.clip(((depth_frame - 0) / (self.DEPTH_MAX_VAL - 0) * 255.0), 0, 255).astype(np.uint8)
        return scaled_frame

    def load_from_yaml(self, path, fileName):
        # Load configuration data from a YAML file
        fullPath = os.path.join(path, fileName)
        if not os.path.exists(fullPath):
            self.get_logger().warn(f"File path is invalid: {fullPath}")
            return None
        
        with open(fullPath, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.get_logger().warn(f"Error reading YAML file: {e}")
                return None


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
