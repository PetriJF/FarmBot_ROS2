from rclpy.node import Node
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Panorama:
    def __init__(self, node: Node):
        self.node_ = node
        
        self.map_x = -1.0
        self.map_y = -1.0

        self.bridge = CvBridge()
        self.rgb_image_ = None
        self.depth_image_ = None
        self.rgb_sub = self.node_.create_subscription(Image, '/rgb_img', self.rgb_callback, 10)
        self.depth_sub = self.node_.create_subscription(Image, '/depth_img', self.depth_callback, 10)

    def rgb_callback(self, msg):
        self.rgb_image_ = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_image_ = self.bridge.imgmsg_to_cv2(msg, "mono8")
        
    def initialize_map_if_needed(self, map_path):
        if not os.path.exists(map_path):
            # Create a blank canvas with 3 channels for RGB
            blank_canvas = np.zeros((self.map_size_x_px, self.map_size_y_px, 3), dtype=np.uint8)
            cv2.imwrite(map_path, blank_canvas)

    def load_from_yaml(self, path, file_name):
        # Load configuration data from a YAML file
        full_path = os.path.join(path, file_name)
        if not os.path.exists(full_path):
            self.node_.get_logger().warn(f"File path is invalid: {full_path}")
            return None
        
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.node_.get_logger().warn(f"Error reading YAML file: {e}")
                return None
    

    def stitch_image_onto_map(self, x: float, y: float):
        self.config_directory_ = os.path.join(get_package_share_directory('camera_handler'), 'config')
        calib_file = os.path.join(self.config_directory_,'camera_calibration.yaml')
        
        self.config_data_ = self.load_from_yaml(self.config_directory_, calib_file)
        
        if self.map_x == -1.0 or self.map_y == -1.0:
            map_directory_ = os.path.join(get_package_share_directory('map_handler'), 'config')
            map_file = 'active_map.yaml'
            map_instance = self.load_from_yaml(map_directory_, map_file)
            
            self.map_x = map_instance['map_reference']['x_len']
            self.map_y = map_instance['map_reference']['y_len']

            self.node_.get_logger().info('Loading map dimensions from active map file')
        
        self.map_size_x_px = int(self.map_x/self.config_data_['coord_scale'])
        self.map_size_y_px = int(self.map_y/self.config_data_['coord_scale'])
        
        x = int(x/self.config_data_['coord_scale'])
        y = self.map_size_y_px - int(y/self.config_data_['coord_scale'])
        
        # Load the new image
        new_image = self.rgb_image_
        rotation_angle = self.config_data_['total_rotation_angle']
        new_image = self.rotate_image(new_image, rotation_angle)
        map_path = os.path.join(self.config_directory_,'rgb_map.png')
        # Load or initialize the map image
        self.initialize_map_if_needed(map_path)
        # Loading or initializing the map_image as RGB
        map_image = cv2.imread(map_path, cv2.IMREAD_COLOR)  # Ensures map_image is read as RGB
        if new_image.shape[2] != 3 or map_image.shape[2] != 3:
            raise ValueError("Both images must be RGB.")

        # Calculate placement and cropping
        new_img_height, new_img_width = new_image.shape[:2]
        map_height, map_width = map_image.shape[:2]

        # Calculate the region of interest (ROI) in the map
        start_x = max(x, 0)
        start_y = max(y - new_img_height, 0)
        end_x = min(x + new_img_width, map_width)
        end_y = min(y, map_height)

        # Calculate corresponding region in new_image
        new_start_x = start_x - x if x < 0 else 0
        new_start_y = 0 if y - new_img_height < 0 else new_img_height - (y - start_y)
        new_end_x = new_img_width - (x + new_img_width - map_width) if (x + new_img_width) > map_width else new_img_width
        new_end_y = y if y < new_img_height else new_img_height

        # Copy the overlapping area from new_image to map_image
        if new_end_x > new_start_x and new_end_y > new_start_y:
            map_image[start_y:end_y, start_x:end_x] = new_image[new_start_y:new_end_y, new_start_x:new_end_x]

        # Save the updated map image
        cv2.imwrite(map_path, map_image)
        
    def rotate_image(self, image, angle):
        # Get the dimensions of the image
        height, width = image.shape[:2]
        
        # Calculate the rotation center
        center = (width // 2, height // 2)
        
        # Perform the rotation
        rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))

        return rotated_image
