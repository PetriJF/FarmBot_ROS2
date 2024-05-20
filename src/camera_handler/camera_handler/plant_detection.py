from rclpy.node import Node
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Panorama:
    '''
    Class used for taking pictures from the farmbot and stitching them into
    a panorama representing the map information
    '''
    def __init__(self, node: Node):
        '''
        Panorama module constructor extending a node's functionality
        '''
        self.node_ = node
        
        self.map_x = -1.0
        self.map_y = -1.0
        self.config_directory_ = os.path.join(get_package_share_directory('camera_handler'), 'config')
        self.calib_file_ = 'camera_calibration.yaml'
        
        self.bridge = CvBridge()
        self.rgb_image_ = None
        self.depth_image_ = None
        self.rgb_sub = self.node_.create_subscription(Image, '/rgb_img', self.__rgb_callback, 10)
        self.depth_sub = self.node_.create_subscription(Image, '/depth_img', self.__depth_callback, 10)

    def __rgb_callback(self, msg):
        '''
        Subscriber callback for the RGB camera feed
        '''
        self.rgb_image_ = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def __depth_callback(self, msg):
        '''
        Subscriber callback for the Depth camera feed
        '''
        self.depth_image_ = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def load_from_yaml(self, path: str, file_name: str):
        '''
        Loads the specified yaml file from the specified path and returns a 
        the dictionary withign the file
        '''
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
    
    def segment_plants(image, hsv_min=[40, 50, 50], hsv_max=[90, 255, 255]):
        # Read the image
        if image is None:
            print("Could not open or find the image.")
            return
        
        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the green color
        lower_green = np.array(hsv_min)
        upper_green = np.array(hsv_max)
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # Perform morphological operations to remove noise and fill gaps
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours of the plants
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create a copy of the original image to draw contours
        circles = []
        # Draw circles around each detected plant
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter out small contours
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circles.append((center, radius))
        return circles
    
    def save_image_for_mosaic(self, num: int):
        mosaic_directory = os.path.join(self.config_directory_, 'mosaic')
        filename = f"{mosaic_directory}/image_{num}.jpg"
        os.makedirs(mosaic_directory, exist_ok=True)
        cv2.imwrite(filename, self.rgb_image_)
        self.node_.get_logger().info(f'saved mosaic image {num:03}')
        
        
    def stitch_image_onto_map(self, x: float, y: float):
        '''
        Takes a picture from the Luxonis camera and stitches it to the 
        panorama map at the specified x and y coordinates
        '''
        # Loading the camera calibration information
        
        self.config_data = self.load_from_yaml(self.config_directory_, self.calib_file_)
        # load image rgb
        rgb_image_raw = self.rgb_image_
        rotation_angle = self.config_data['total_rotation_angle']
        rgb_image_rotated = self.rotate_image(rgb_image_raw, rotation_angle)
        # draw circles around green things
        circles = segment_plants(rgb_image_raw)

        # convert circle center coordinates to map coordinates = robot coords + converted image coords to map coords
        
        # mark circles corresponding to saved plant coordinates blue
        # mark other circles red
        # write coordinates and radi of red circles to a yaml file for weeds
        # If the map dimensions were not set at the start of the run, load them from the active map
        if self.map_x == -1.0 or self.map_y == -1.0:
            # Load map instance
            map_directory_ = os.path.join(get_package_share_directory('map_handler'), 'config')
            map_file = 'active_map.yaml'
            map_instance = self.load_from_yaml(map_directory_, map_file)
            if map_instance:
                # Get map dimensions
                self.map_x = map_instance['map_reference']['x_len']
                self.map_y = map_instance['map_reference']['y_len']

                self.node_.get_logger().info('Loading map dimensions from active map file')
        
        # Set the map size relative to pixels
        self.map_size_x_px = int(self.map_x / self.config_data['coord_scale'])
        self.map_size_y_px = int(self.map_y / self.config_data['coord_scale'])
        
        x = int(x / self.config_data['coord_scale'])
        y = self.map_size_y_px - int(y/self.config_data['coord_scale'])
        
        # Load the new image
        
        map_path = os.path.join(self.config_directory_,'rgb_map.png')
        # Load or initialize the map (panorama) image 
        self.initialize_map_if_needed(map_path)
        # Loading or initializing the map_image as RGB
        map_image = cv2.imread(map_path, cv2.IMREAD_COLOR)  # Ensures map_image is read as RGB
        if new_image.shape[2] != 3 or map_image.shape[2] != 3:
            raise ValueError("Both images must be RGB.")

        # Calculate placement and cropping
        new_img_height, new_img_width = new_image.shape[:2]
        map_height, map_width = map_image.shape[:2]

        # Calculate the region of interest (ROI) in the map
        start_x = max(map_width - (x - new_img_width), 0)
        start_y = max(map_height - (y + new_img_height), 0)
        end_x = min(map_width - x , map_width)
        end_y = min(map_height - y, map_height)

        # Calculate corresponding region in new_image
        #new_start_x = start_x - x if x < 0 else 0
        new_start_x = start_x - (x-map_width) if x > map_width else 0
        new_start_y = 0 if y - new_img_height < 0 else new_img_height - (y - start_y)
        new_end_x = new_img_width - (x + new_img_width - map_width) if (x + new_img_width) > map_width else new_img_width
        new_end_y = y if y < new_img_height else new_img_height

        # Copy the overlapping area from new_image to map_image
        if new_end_x > new_start_x and new_end_y > new_start_y:
            map_image[start_y:end_y, start_x:end_x] = new_image[new_start_y:new_end_y, new_start_x:new_end_x]

        # Save the updated map image
        cv2.imwrite(map_path, map_image)
    
    
        
    def rotate_image(self, image, angle):
        '''
        Rotates the image around the center at the set angle
        '''
        
        # Get the dimensions of the image
        height, width = image.shape[:2]
        
        # Calculate the rotation center
        center = (width // 2, height // 2)
        
        # Perform the rotation
        rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))

        return rotated_image