import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import yaml
import os
import numpy as np
import depthai as dai

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.load_config()
        self.setup_camera()
        
        # Publishers
        self.rgb_publisher = self.create_publisher(Image, 'rgb_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)

        
        # Timer to periodically process and publish images
        timer_period = 0.05  # seconds (adjust based on desired framerate)
        self.timer = self.create_timer(timer_period, self.run)

    def load_config(self):
        print('test2')
        self.directory = os.path.join(
            get_package_share_directory('camera_handler'),
            'config'
        )
        self.config_file = 'camera_config.yaml'
        self.config_data = self.load_from_yaml(self.directory, self.config_file)
        self.HOST = self.config_data['server_constants']['HOST']
        self.PORT= self.config_data['server_constants']['PORT']
        self.DEPTH_MIN_VAL = self.config_data['camera_constants']['DEPTH_MIN_VAL']
        self.DEPTH_MAX_VAL = self.config_data['camera_constants']['DEPTH_MAX_VAL']
        self.FOCAL_LENGTH_PX = self.config_data['camera_constants']['FOCAL_LENGTH_PX']
        self.BASELINE = self.config_data['camera_constants']['BASELINE']
 
    def setup_camera(self):
        # Create a pipeline
        self.pipeline = dai.Pipeline()
        # Define and configure the stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        self.xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        self.xout_stereo = self.pipeline.create(dai.node.XLinkOut)

        # Set properties for mono cameras
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        
        # Define and configure the RGB camera node
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(40)

        self.xout_rgb.setStreamName("rgb")
        self.xout_stereo.setStreamName("stereo")

        # Set stereo depth properties from your configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(self.config_data['camera_constants']['lr_check'])
        stereo.setExtendedDisparity(self.config_data['camera_constants']['extended_disparity'])
        stereo.setSubpixel(self.config_data['camera_constants']['subpixel'])

        # Link mono cameras to stereo depth node
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        cam_rgb.preview.link(self.xout_rgb.input)
        stereo.disparity.link(self.xout_stereo.input)
        # Additional RGB camera configurations from your YAML data
        # cam_rgb.setFps(self.config_data['camera_constants']['FPS'])

        # Create XLinkOut nodes for sending data to the host


        # Disparity config
        config = stereo.initialConfig.get()
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        config.postProcessing.spatialFilter.holeFillingRadius = 3
        config.postProcessing.spatialFilter.numIterations = 1
        config.postProcessing.decimationFilter.decimationFactor = 1
        stereo.initialConfig.set(config)
        
        self.device = dai.Device(self.pipeline)

        
    def publish_images(self, rgb_frame, depth_frame):
        # Convert and publish RGB frame
        if rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
            self.rgb_publisher.publish(rgb_msg)

        if depth_frame is not None:
        # Convert and publish Depth frame (adjust conversion as needed)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono8')
            self.depth_publisher.publish(depth_msg)

    def run(self):
    # Connect to device and start the pipeline
        # Output queues will be used to get the rgb frames and depth maps from the outputs defined above
        latestPacket = {"rgb": None, "stereo": None}
        queueEvents = self.device.getQueueEvents(("rgb", "stereo"))
        for queueName in queueEvents:
            packets = self.device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]
        if latestPacket["rgb"] is not None:
            rgb_frame = latestPacket["rgb"].getCvFrame()
            self.publish_images(rgb_frame, None)
        if latestPacket["stereo"] is not None:
            disparity_frame = latestPacket["stereo"].getFrame()
            disparity_frame[disparity_frame == 0] = 0.01  # Adjust this value as needed
            depth_frame = (self.FOCAL_LENGTH_PX * self.BASELINE) / disparity_frame
            depth_frame[(depth_frame >= self.DEPTH_MAX_VAL) | (depth_frame <= self.DEPTH_MIN_VAL)] = 0
            scaled_frame = ((depth_frame - 0) / (self.DEPTH_MAX_VAL - 0) * 255.0).astype(np.uint8)
            np.set_printoptions(threshold=np.inf)
            print("SCALED max: ",scaled_frame.max(), " avg: ", scaled_frame.mean())
            print("DISPARITY max: ",disparity_frame.max(), " avg: ", disparity_frame.mean())
            print("DEPTH max: ",depth_frame.max(), " avg: ", depth_frame.mean())
            #cv2.imshow("depth", scaled_frame)
            self.publish_images(None, scaled_frame)  # Publish depth frame 
        # Use rclpy.spin_once(self) if you need to handle ROS 2 callbacks
        
    def load_from_yaml(self, path = '', fileName = ''):
        if path == '':
            self.get_logger().warn("Path not set for retrieving the parameter config file")
            return
        if fileName == '':
            self.get_logger().warn("Parameter Config File name not set")
            return
        if not os.path.exists(path):
            self.get_logger().warn("File path is invalid: ")
            self.get_logger().warn(path)
            return
        
        with open(os.path.join(path, fileName), 'r') as yaml_file:
            loaded_data = yaml.safe_load(yaml_file)
            if isinstance(loaded_data, dict):
                return loaded_data
            else:
                self.get_logger().warn("Invalid YAML file format..")


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
