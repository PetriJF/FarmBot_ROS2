from rclpy.node import Node
from farmbot_controllers.movement import Movement
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory

#TODO get these values from map config file
MAP_X = 2000
MAP_Y = 1500

class Panorama:
    def __init__(self, node: Node, mvm: Movement):
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        self.node_ = node
        self.mvm_ = mvm
        
    
    def update_position(self, x: float, y: float, z: float):
        self.x_ = x
        self.y_ = y
        self.z_ = z
        
    def initialize_map_if_needed(self, map_path):
        if not os.path.exists(map_path):
            # Create a blank canvas with 3 channels for RGB
            blank_canvas = np.zeros((self.map_size_x_px, self.map_size_y_px, 3), dtype=np.uint8)
            cv2.imwrite(map_path, blank_canvas)

    def load_from_yaml(self, path, fileName):
        # Load configuration data from a YAML file
        fullPath = os.path.join(path, fileName)
        if not os.path.exists(fullPath):
            self.node_.get_logger().warn(f"File path is invalid: {fullPath}")
            return None
        
        with open(fullPath, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.node_.get_logger().warn(f"Error reading YAML file: {e}")
                return None

    def stitch_image_onto_map(self):
        self.config_directory_ = os.path.join(get_package_share_directory('camera_handler'), 'config')
        calib_file = os.path.join(self.config_directory_,'camera_calibration.yaml')
        self.config_data_ = self.load_from_yaml(self.config_directory_, calib_file)
        self.map_size_x_px = int(MAP_X/self.config_data_['coord_scale'])
        self.map_size_y_px = int(MAP_Y/self.config_data_['coord_scale'])
        
        self.x_ = int(self.x_/self.config_data_['coord_scale'])
        self.y_ = self.map_size_y_px - int(self.y_/self.config_data_['coord_scale'])
        
        # Load the new image
        new_image_path = os.path.join(self.config_directory_,'saved_rgb_image.png')
        for _ in range(3):
            try:
                new_image = cv2.imread(new_image_path, cv2.IMREAD_UNCHANGED)
                if new_image is not None:
                    break  # Image successfully read, exit the loop
            except Exception as e:
                print(f"Error reading image: {e}")
        else:
            # If the loop completes without successfully reading the image, raise an error
            raise RuntimeError("Failed to read the image after multiple attempts")
        
        rotation_angle = self.config_data_['total_rotation_angle']
        new_image = self.rotate_image(new_image, rotation_angle)
        map_path = os.path.join(self.config_directory_,'rgb_map.png')
        # Load or initialize the map image
        self.initialize_map_if_needed(map_path)
        # Loading or initializing the map_image as RGB
        map_image = cv2.imread(map_path, cv2.IMREAD_COLOR)  # Ensures map_image is read as RGB
        if new_image.shape[2] != 3 or map_image.shape[2] != 3:
            raise ValueError("Both images must be RGB.")

        # The rest of your function remains the same

        
        # Calculate placement and cropping
        new_img_height, new_img_width = new_image.shape[:2]
        map_height, map_width = map_image.shape[:2]

        # Calculate the region of interest (ROI) in the map
        start_x = max(self.x_, 0)
        start_y = max(self.y_ - new_img_height, 0)
        end_x = min(self.x_ + new_img_width, map_width)
        end_y = min(self.y_, map_height)

        # Calculate corresponding region in new_image
        new_start_x = start_x - self.x_ if self.x_ < 0 else 0
        new_start_y = 0 if self.y_ - new_img_height < 0 else new_img_height - (self.y_ - start_y)
        new_end_x = new_img_width - (self.x_ + new_img_width - map_width) if (self.x_ + new_img_width) > map_width else new_img_width
        new_end_y = self.y_ if self.y_ < new_img_height else new_img_height

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
