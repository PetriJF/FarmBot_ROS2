import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import yaml
import os
import cv2
import numpy as np
import depthai as dai
import threading

class LuxonisCameraNode(Node):
    '''
    Camera module that reads the Luxonis camera packets and publishes the RGB and 
    depth frames on their respective topics
    '''
    def __init__(self):
        '''
        Module constructor extendint the node instance
        '''
        super().__init__('LuxonisCamera')

        self.bridge = CvBridge()# Bridge to convert between ROS and OpenCV images
        self.load_config()      # Load configuration from YAML file
        self.setup_camera()     # Initialize and configure the DepthAI camera
        
        # Initialize publishers for RGB and depth images
        self.rgb_publisher = self.create_publisher(Image, 'rgb_img', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_img', 10)
        
        self.rgb_image_ = None
        self.depth_image_ = None

        # Start the packet processing loop in a separate thread
        self.should_continue = True  # Flag to control the execution of the thread
        self.processing_thread = threading.Thread(target=self.packet_processing_loop)
        self.processing_thread.daemon = True  # Ensures that the thread will close when the main program exits
        self.processing_thread.start()

        self.get_logger().info('Luxonis Camera Node initialized...')
    
    def setup_camera(self):
        '''
        Sets the camera links and pipeline
        '''
        # Configure the DepthAI pipeline with stereo and RGB camera nodes
        self.pipeline = dai.Pipeline()
        stereo, mono_left, mono_right, cam_rgb = self.configure_depthai_components()
        
        # Link camera outputs to XLinkOut nodes for transmission to host
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        cam_rgb.preview.link(self.xout_rgb.input)
        stereo.disparity.link(self.xout_stereo.input)
        
        self.device = dai.Device(self.pipeline)  # Connect to the DepthAI device

    def load_config(self):
        '''
        Loads the camera configuration from the calibration file
        '''
        # Load configuration from YAML file
        self.config_directory_ = os.path.join(get_package_share_directory('camera_handler'), 'config')
        config_file = 'luxonis_camera_config.yaml'
        self.config_data = self.load_from_yaml(self.config_directory_, config_file)
        # Extract relevant configuration values
        self.HOST = self.config_data['server_constants']['HOST']
        self.PORT = self.config_data['server_constants']['PORT']
        self.DEPTH_MIN_VAL = self.config_data['camera_constants']['DEPTH_MIN_VAL']
        self.DEPTH_MAX_VAL = self.config_data['camera_constants']['DEPTH_MAX_VAL']
        self.FOCAL_LENGTH_PX = self.config_data['camera_constants']['FOCAL_LENGTH_PX']
        self.BASELINE = self.config_data['camera_constants']['BASELINE']

        self.WIDTH_PIXEL_COUNT = int(self.config_data['camera_calibration']['WIDTH_PIXEL_COUNT'])
        self.HEIGHT_PIXEL_COUNT = int(self.config_data['camera_calibration']['HEIGHT_PIXEL_COUNT'])

        self.get_logger().info('Updated camera settings based on calibration')

    def packet_processing_loop(self):
        '''
        Loop used for processing packets from the camera
        '''
        while self.should_continue:
            # Fetch packets from the device's output queues
            latestPacket = {"rgb": None, "stereo": None}
            queueEvents = self.device.getQueueEvents(("rgb", "stereo"))
            for queueName in queueEvents:
                packets = self.device.getOutputQueue(queueName).tryGetAll()
                if len(packets) > 0:
                    latestPacket[queueName] = packets[-1]

            # Process the latest available packets
            if latestPacket["rgb"] is not None:
                self.rgb_image_ = latestPacket["rgb"].getCvFrame()
                self.publish_images(rgb_frame=self.rgb_image_)
            if latestPacket["stereo"] is not None:
                self.depth_image_ = self.process_depth_frame(latestPacket["stereo"].getFrame())
                self.publish_images(depth_frame=self.depth_image_)
                
    def stop_processing(self):  # Gracefully stop the processing thread
        self.should_continue = False  # Signal the thread to exit
        if self.processing_thread.is_alive():
            self.processing_thread.join()  # Wait for the thread to finish

    def __del__(self):  # Consider removing if you opt for explicit shutdown calls
        self.stop_processing()  # Ensure resources are cleaned up

    def configure_depthai_components(self):
        '''
        Configuration for the DepthAI components
        '''
        # Create and configure DepthAI components (stereo depth, mono cameras, RGB camera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        
        # Mono camera settings
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        # RGB camera settings
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.WIDTH_PIXEL_COUNT, self.HEIGHT_PIXEL_COUNT)
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
        
        return stereo, mono_left, mono_right, cam_rgb

    def configure_stereo_depth(self, stereo):
        '''
        Apply stereo depth settings from configuration amd disparity post-processing
        '''
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

    def publish_images(self, rgb_frame = None, depth_frame = None):
        '''
        Publish RGB and depth frames if available
        '''
        if rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
            self.rgb_publisher.publish(rgb_msg)

        if depth_frame is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono8')
            self.depth_publisher.publish(depth_msg)

    # def run(self):
    #     # Retrieve and publish RGB and depth frames from the camera
    #     latestPacket = {"rgb": None, "stereo": None}
    #     queueEvents = self.device.getQueueEvents(("rgb", "stereo"))
    #     for queueName in queueEvents:
    #         packets = self.device.getOutputQueue(queueName).tryGetAll()
    #         if len(packets) > 0:
    #             latestPacket[queueName] = packets[-1]

    #     if latestPacket["rgb"] is not None:
    #         self.rgb_image_ = latestPacket["rgb"].getCvFrame()
    #         #self.rgb_image_ = cv2.rotate(self.rgb_image_, cv2.ROTATE_180)
    #         #self.rgb_image_ = cv2.flip(self.rgb_image_, 1)
    #         self.publish_images(rgb_frame=self.rgb_image_)
    #         self.save_images(rgb=True)
    #     if latestPacket["stereo"] is not None:
    #         self.depth_image_ = self.process_depth_frame(latestPacket["stereo"].getFrame())
    #         #self.depth_image_ = cv2.rotate(self.depth_image_, cv2.ROTATE_180)
    #         #self.depth_image_ = cv2.flip(self.depth_image_, 1)
    #         self.publish_images(depth_frame=self.depth_image_ )
    #         self.save_images(depth=True)
        

    def process_depth_frame(self, disparity_frame):
        '''
        Process disparity frame to generate a depth frame for publishing
        '''
        disparity_frame[disparity_frame == 0] = 0.01  # Handle zero disparity
        depth_frame = (self.FOCAL_LENGTH_PX * self.BASELINE) / disparity_frame
        depth_frame[(depth_frame >= self.DEPTH_MAX_VAL) | (depth_frame <= self.DEPTH_MIN_VAL)] = 0
        scaled_frame = np.clip(((depth_frame - 0) / (self.DEPTH_MAX_VAL - 0) * 255.0), 0, 255).astype(np.uint8)
        return scaled_frame

    def load_from_yaml(self, path, file_name):
        '''
        Loads the specified yaml file from the specified path and returns a 
        the dictionary withign the file
        '''
        full_path = os.path.join(path, file_name)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"File path is invalid: {full_path}")
            return None
        
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.get_logger().warn(f"Error reading YAML file: {e}")
                return None
            
    # def save_images(self):
    #     if self.rgb_image_ is not None and self.depth_image_ is not None:
    #         cv2.imwrite(os.path.join(self.config_directory_,"saved_rgb_image.png"), self.rgb_image_)
    #         self.get_logger().info('RGB image saved successfully.')
    #         cv2.imwrite(os.path.join(self.config_directory_,"saved_depth_image.png"), self.depth_image_)
    #         self.get_logger().info('Depth image saved successfully.')
    #     else:
    #         self.get_logger().info('No images to save.')
            
    # def save_images(self, rgb = False, depth = False):
    #     if self.rgb_image_ is not None and rgb == True: 
    #         cv2.imwrite(os.path.join(self.config_directory_,"saved_rgb_image.png"), self.rgb_image_)
    #         self.get_logger().info('RGB image saved successfully.')
    #     elif self.depth_image_ is not None and depth == True:
    #         cv2.imwrite(os.path.join(self.config_directory_,"saved_depth_image.png"), self.depth_image_)
    #         self.get_logger().info('Depth image saved successfully.')
    #     else:
    #         self.get_logger().info('No images to save.')
    

def main(args=None):
    rclpy.init(args=args)
    camera_node = LuxonisCameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:        
        camera_node.stop_processing()  # Ensure the processing thread is stopped gracefully
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
