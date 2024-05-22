import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node

class PlantDetection:
    '''
    Class used for taking pictures from the farmbot and processing them for plant detection
    '''
    def __init__(self, node: Node):
        '''
        PlantDetection module constructor extending a node's functionality
        '''
        self.node = node
        
        self.map_width = -1.0
        self.map_height = -1.0
        self.config_directory = os.path.join(get_package_share_directory('camera_handler'), 'config')
        self.map_directory = os.path.join(get_package_share_directory('map_handler'), 'config')
        self.map_file = 'active_map.yaml'
        self.calib_file = 'camera_calibration.yaml'
        self.camera_config_file = 'standard_camera_config.yaml'
        
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.rgb_sub = self.node.create_subscription(Image, '/rgb_img', self.rgb_callback, 10)
        self.depth_sub = self.node.create_subscription(Image, '/depth_img', self.depth_callback, 10)

    def rgb_callback(self, msg):
        '''
        Subscriber callback for the RGB camera feed
        '''
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        '''
        Subscriber callback for the Depth camera feed
        '''
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

    def load_yaml(self, path: str, file_name: str):
        '''
        Loads the specified yaml file from the specified path and returns a dictionary
        '''
        full_path = os.path.join(path, file_name)
        if not os.path.exists(full_path):
            self.node.get_logger().warn(f"File path is invalid: {full_path}")
            return None
        
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.node.get_logger().warn(f"Error reading YAML file: {e}")
                return None
    
    def segment_plants(self, image, hsv_min=[40, 50, 50], hsv_max=[90, 255, 255]):
        '''
        Segments plants in the image based on HSV color space and returns circles around detected plants
        '''
        if image is None:
            print("Could not open or find the image.")
            return
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array(hsv_min)
        upper_green = np.array(hsv_max)
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter out small contours
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x) + self.camera_config_data['camera_calibration']['CAMERA_X'] - self.camera_config_data['camera_calibration']['CENTER_PIXEL_X'], 
                          int(y) + self.camera_config_data['camera_calibration']['CAMERA_Y'] - self.camera_config_data['camera_calibration']['CENTER_PIXEL_Y'])
                radius = int(radius)
                circles.append((center, radius))
        return np.array(circles)
    
    def identify_known_plants(self, circles, plant_positions):
        '''
        Identifies which circles correspond to known plant positions
        '''
        results = []
        for circle_x, circle_y, circle_radius in circles:
            radius_squared = circle_radius ** 2
            is_known_plant = False
            for plant_x, plant_y in plant_positions:
                distance_squared = (plant_x - circle_x) ** 2 + (plant_y - circle_y) ** 2
                if distance_squared <= radius_squared:
                    is_known_plant = True
                    break
            results.append((circle_x, circle_y, circle_radius, is_known_plant))
        return np.array(results)
    
    def filter_nearby_circles(self, known_circles):
        '''
        Adds a boolean column indicating if each circle is within 100 units of the perimeter of any known plant circle
        '''
        known_plant_circles = np.array([circle for circle in known_circles if circle[3]])
        all_circles = np.array(known_circles)
        within_100_units = np.zeros(len(all_circles), dtype=bool)
        
        for i, circle in enumerate(all_circles):
            if circle[3]:
                within_100_units[i] = True

        for i, oc in enumerate(all_circles):
            if within_100_units[i]:
                continue
            (ox, oy), oradius = oc[:2], oc[2]
            for kc in known_plant_circles:
                (kx, ky), kradius = kc[:2], kc[2]
                distance = np.sqrt((ox - kx)**2 + (oy - ky)**2)
                if distance <= kradius + oradius + 100:
                    within_100_units[i] = True
                    break

        all_circles_within_100 = np.hstack((all_circles, within_100_units.reshape(-1, 1)))
        return all_circles_within_100
    
    def scale_circles(self, circles, scale):
        '''
        Scales all values in circles by the given scale factor
        '''
        circles = np.array(circles)
        values = circles[:, :3]  # Numeric values
        boolean_column = circles[:, 3]  # Boolean values
        scaled_values = values * scale
        scaled_circles = np.hstack((scaled_values, boolean_column.reshape(-1, 1)))
        return scaled_circles
    
    def append_to_yaml(self, path, data):
        '''
        Appends data to a YAML file without overwriting existing data
        '''
        if not os.path.exists(path):
            with open(path, 'w') as file:
                yaml.dump(data, file)
        else:
            with open(path, 'r') as file:
                existing_data = yaml.safe_load(file) or []
            with open(path, 'w') as file:
                yaml.dump(existing_data + data, file)
    
    def detect_weeds(self, x: float, y: float):
        '''
        Processes the images to detect weeds and known plants and updates the map with detected circles
        '''
        self.calib_data = self.load_yaml(self.config_directory, self.calib_file)
        self.camera_config_data = self.load_yaml(self.config_directory, self.camera_config_file)
        active_map = self.load_yaml(self.map_directory, self.map_file)
        
        plant_positions = [(plant_data['position']['x'], plant_data['position']['y']) 
                           for plant_data in active_map['plant_details']['plants'].values() if plant_data]
        plant_positions = np.array(plant_positions)

        rgb_image_raw = self.rgb_image
        rotation_angle = self.calib_data['total_rotation_angle']
        rgb_image_rotated = self.rotate_image(rgb_image_raw, rotation_angle)

        circles = self.segment_plants(rgb_image_rotated)

        robot_coords_circles = np.array([((int(center[0] * self.calib_data['coord_scale']) + x,
                                          int(center[1] * self.calib_data['coord_scale']) + y),
                                         int(radius * self.calib_data['coord_scale']) + 10) for center, radius in circles])
        
        known_circles = self.identify_known_plants(robot_coords_circles, plant_positions)
        final_circles = self.filter_nearby_circles(known_circles)

        # Visualize circles on camera image
        cam_img_circles = rgb_image_rotated.copy()
        for circle in final_circles:
            (cx, cy), radius, is_known_plant, is_within_100 = circle[:2], circle[2], circle[3], circle[4]
            color = (255, 0, 0) if is_known_plant else (0, 0, 255)
            if is_within_100 and not is_known_plant:
                color = (0, 255, 255)
            cv2.circle(cam_img_circles, (int(cx), int(cy)), int(radius), color, 2)

        cam_img_circles_path = os.path.join(self.config_directory, 'cam_img_circles.jpg')
        cv2.imwrite(cam_img_circles_path, cam_img_circles)

        map_circles = self.scale_circles(final_circles, self.calib_data['coord_scale'])

        map_path = os.path.join(self.config_directory, 'rgb_map.png')
        self.initialize_map_if_needed(map_path)
        map_image = cv2.imread(map_path, cv2.IMREAD_COLOR)
        if map_image.shape[2] != 3:
            raise ValueError("Both images must be RGB.")

        for circle in map_circles:
            (cx, cy), radius, is_known_plant, is_within_100 = circle[:2], circle[2], circle[3], circle[4]
            color = (255, 0, 0) if is_known_plant else (0, 0, 255)
            if is_within_100 and not is_known_plant:
                color = (0, 255, 255)
            cv2.circle(map_image, (int(cx), int(cy)), int(radius), color, 2)

        cv2.imwrite(map_path, map_image)

        known_plant_circles = [circle.tolist() for circle in final_circles if circle[3]]
        other_circles = [circle.tolist() for circle in final_circles if not circle[3]]

        known_plants_yaml = os.path.join(self.config_directory, 'known_plants.yaml')
        other_plants_yaml = os.path.join(self.config_directory, 'other_plants.yaml')

        self.append_to_yaml(known_plants_yaml, known_plant_circles)
        self.append_to_yaml(other_plants_yaml, other_circles)
    
    def rotate_image(self, image, angle):
        '''
        Rotates the image around the center at the set angle
        '''
        height, width = image.shape[:2]
        center = (width // 2, height // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
        return rotated_image

    def initialize_map_if_needed(self, map_path):
        '''
        Initialize the map image if it does not exist
        '''
        if not os.path.exists(map_path):
            self.map_width = 1000  # Set the map size (example values)
            self.map_height = 1000
            map_image = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
            cv2.imwrite(map_path, map_image)
