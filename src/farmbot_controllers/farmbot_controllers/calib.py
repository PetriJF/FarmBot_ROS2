from rclpy.node import Node
from farmbot_controllers.movement import Movement
from ament_index_python.packages import get_package_share_directory
import os
import math
import cv2
import numpy as np
import time
import yaml


"""Calibrate camera using a grid of circles calibration card."""

ROW_COLORS = [(0, 0, 255), (0, 127, 255), (0, 186, 186), (0, 255, 0),
              (187, 187, 0), (255, 0, 0), (255, 0, 255)]
AXIS_INDEX = {'init': 0, 'x': 2, 'y': 1}
AXIS_COLORS = [
    (255, 255, 255),  # init/center: white
    (0, 0, 255),  # y-axis: red
    (255, 0, 0),  # x-axis: blue
]
RELATIVE_MOVEMENTS = [
    {'x': 0, 'y': 0, 'z': 0},
    {'x': 0, 'y': 50, 'z': 0},
    {'x': 50, 'y': 0, 'z': 0},
]

class CalibrateCamera:
    def __init__(self, node: Node, mvm: Movement):
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        self.node_ = node
        self.mvm_ = mvm
        """Set initial attributes.

        Arguments:
            calibration_data: P2C().calibration_params JSON
        """
        self.calibration_data = {}
        self.config_directory_ = os.path.join(get_package_share_directory('camera_handler'), 'config')
        self.rgb_dir_ = os.path.join(self.config_directory_,"saved_rgb_image.png")
        self.depth_dir_ = os.path.join(self.config_directory_,"saved_depth_image.png")
        self.pattern = {
            'size': (5, 7),
            'type': cv2.CALIB_CB_ASYMMETRIC_GRID,
            'row_circle_separation': 30,
        }
        self.dot_images = {
            AXIS_INDEX['init']: {},
            AXIS_INDEX['x']: {},
            AXIS_INDEX['y']: {},
        }
        self.output_img = None
        self.center = None
        self.axis_points = None
        self.rotation_angles = []
        self.success_flag = True
        self.relative_starting_position = None
    
    def update_position(self, x: float, y: float, z: float):
        self.x_ = x
        self.y_ = y
        self.z_ = z
        
    def calibrate_camera(self):
        calibration_results = {}
        success = self.move_and_capture()
        if success:
            self.calibrate()
            self.save_image()
    
    """Determine camera calibration data using a circle grid calibration card.

    # Card details

              5 cols
     _______________________
    |                       |
    |    o   o   o   o   o  |
    |  o   o   o   o   o    |
    |    o   o   o   o   o  |
    |  o   o   o   o   o    | 7 rows
    |    o   o   o   o   o  |
    |  o   o   o   o   o    |
    |    o   o   o   o   o  |
    |_______________________|

    30mm center to center circles along horizontal and vertical lines
    15mm center to center circles in each row

    # Process

    Will take a photo, then move +50mm on the y-axis, take another photo,
    and finally move +50mm on the x-axis and take a third photo.

    Pattern must be visible in all three images, but can be in any orientation.

    positions moved to:
    | 1 > 2
    | ^
    | 0
    '-----

    where calibration card ends up in frame:
    |     0
    |     v
    | 2 < 1
    '-----
    """
    def count_circles(self):
        """Total number of circles in pattern."""
        return self.pattern['size'][0] * self.pattern['size'][1]

    def row_length(self):
        """Length of circle row in millimeters."""
        return (self.pattern['size'][0] - 1) * self.pattern['row_circle_separation']

    def _move(self, amount):
        target_x = self.x_ + amount['x']
        target_y = self.y_ + amount['y']
        target_z = self.z_ + amount['z']
        self.mvm_.move_gantry_abs(x_coord = target_x, y_coord = target_y, z_coord = target_z)
        self.x_ = self.x_ + amount['x']
        self.y_ = self.y_ + amount['y']
        self.z_ = self.z_ + amount['z']

    def move_and_capture(self):
        """Move the bot along x and y axes, take photos, and detect circles."""
        for i, movement in enumerate(RELATIVE_MOVEMENTS):
            if i > 0:
                if self.relative_starting_position is None:
                    self.relative_starting_position = {'x': 0, 'y': 0, 'z': 0}
                self.node_.get_logger().info('Moving to next camera calibration photo location.')
                self._move( movement)
                time.sleep(15)
                for axis in movement:
                    self.relative_starting_position[axis] -= movement[axis]
            self.node_.get_logger().info('Taking camera calibration photo. ({}/3)'.format(i + 1))
            coordinates = {'x':self.x_,'y':self.y_,'z':self.z_}
            self.node_.get_logger().info(f"Trying to load image from: {self.rgb_dir_}")
            for _ in range(5):
                try:
                    img_rgb = cv2.imread(self.rgb_dir_, 1)
                    if img_rgb is not None:
                        break  # Image successfully read, exit the loop
                except Exception as e:
                    print(f"Error reading image: {e}")
            else:
                # If the loop completes without successfully reading the image, raise an error
                raise RuntimeError("Failed to read the image after multiple attempts")
            
            ret, centers = self.find_pattern(img_rgb)
            if not self.success_flag:
                self.save_image(img_rgb, str(i + 1))
                return self.success_flag
            self.dot_images[i]['circles'] = centers
            self.dot_images[i]['found'] = ret
            self.dot_images[i]['image'] = img_rgb
            self.dot_images[i]['coordinates'] = coordinates
            
        #self.return_to_start()
        return self.success_flag

    def return_to_start(self):
        """Move back to starting position."""
        if self.relative_starting_position is not None:
            self.node_.get_logger().info('Returning to starting location...')
            self._move(self.relative_starting_position)

    def get_initial_img_info(self):
        """Get initial image details."""
        self.output_img = self.dot_images[AXIS_INDEX['init']]['image'].copy()
        rows, cols, _ = self.output_img.shape
        self.center = (int(cols / 2), int(rows / 2))
        self.axis_points = [[self.center] * self.count_circles(), [], []]

    def preprocess(self, img, basic):
        'Pre-process image in preparation for pattern detection.'
        def _divide_size_by(factor):
            height = img.shape[0]
            num = int(height / factor)
            return num + 1 if num % 2 == 0 else num
        block_size = {
            'blur': _divide_size_by(100),
            'threshold': _divide_size_by(15),
            'morph': _divide_size_by(50),
        }

        img = img.copy()
        img = cv2.bitwise_not(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img, block_size['blur'])

        if basic:
            return img

        method = cv2.ADAPTIVE_THRESH_MEAN_C
        thesh_type = cv2.THRESH_BINARY
        blocksize = block_size['threshold']
        img = cv2.adaptiveThreshold(img, 255, method, thesh_type, blocksize, 2)

        kernel = np.ones((block_size['morph'], block_size['morph']), np.uint8)
        img = cv2.morphologyEx(img, cv2.MORPH_DILATE, kernel, iterations=1)

        img = cv2.medianBlur(img, block_size['blur'])
        return img

    def detect_circles(self, img, downsample=False):
        'Detect pattern in image.'
        pattern_size = self.pattern['size']
        flags = self.pattern['type']
        large = img.shape[0] > 1200
        if large and downsample:
            img = cv2.pyrDown(img)
        try:
            ret, centers = cv2.findCirclesGrid(img, pattern_size, flags=flags)
        except Exception as exception:
            self.node_.get_logger().error(exception)
            ret, centers = False, None
        if large and downsample and ret:
            centers *= 2
        return ret, centers

    def find_pattern(self, img, move_back=False, save_output=None):
        """Find calibration pattern circles in single image."""
        if img is None:
            self.node_.get_logger().error('ERROR: Calibration failed. Image missing.')
            self.success_flag = False
        original = img.copy()
        # first pass with basic pre-processing
        img = self.preprocess(original, True)
        ret, centers = self.detect_circles(img)
        if not ret:
            # second pass with heavier pre-processing
            img = self.preprocess(original, False)
            ret, centers = self.detect_circles(img, downsample=True)
        if save_output is not None:
            cv2.drawChessboardCorners(img, self.pattern['size'], centers, ret)
            save_output(ret, img, original)
        if not ret and save_output is None:
            if move_back:
                self.return_to_start()
            self.node_.get_logger().error('ERROR: Calibration failed, calibration object not detected in image. Check recent photos.')
            self.success_flag = False
        return ret, centers

    def find_pattern_in_all(self):
        """Find calibration pattern circles in all images."""
        for i, dot_image in enumerate(self.dot_images.values()):
            if dot_image.get('circles') is None:
                ret, centers = self.find_pattern(dot_image['image'])
                self.dot_images[i]['circles'] = centers
                self.dot_images[i]['found'] = ret

    def combine_data(self):
        """Combine detected circle data from the three images."""
        for i in range(self.count_circles()):
            prev_axis_index = 0
            for k in range(1, 3):
                from_dot = self.dot_images[prev_axis_index]['circles'][i][0]
                to_dot = self.dot_images[k]['circles'][i][0]
                prev_axis_index += 1
                if k == 1:  # draw initial detected dots on initial image
                    center = (int(from_dot[0]), int(from_dot[1]))
                    cv2.circle(self.output_img, center, 10,
                               ROW_COLORS[i // 5], -1)
                # translate axis compass at each circle to image center
                translated = self.translate_dot(from_dot, to_dot)
                cv2.line(self.output_img, self.center,
                         translated, AXIS_COLORS[k], 3)
                self.axis_points[k].append(translated)
                if k == AXIS_INDEX['x']:
                    self.rotation_angles.append(self.rotation_calc(translated))

    def translate_dot(self, from_dot, to_dot):
        """Translate axis compass to image center."""
        center_x, center_y = self.center
        from_dot_x, from_dot_y = from_dot
        to_dot_x, to_dot_y = to_dot
        translate_x, translate_y = from_dot_x - center_x, from_dot_y - center_y
        return (int(to_dot_x - translate_x),
                int(to_dot_y - translate_y))

    def rotation_calc(self, translated_dot):
        """Calculate rotation angle using x-axis translations."""
        center_x, center_y = self.center
        x_translation, y_translation = translated_dot
        delta_x = x_translation - center_x
        delta_y = y_translation - center_y
        if delta_x == 0:
            delta_x += 0.001
        return math.degrees(math.atan(delta_y / float(delta_x)))

    def generate_rotation_matrix(self, rotation):
        """For rotating images and points."""
        return cv2.getRotationMatrix2D(tuple(self.center), rotation, 1)

    def rotate_points(self, rotation_matrix):
        """Rotate an array of points using a rotation matrix."""
        axis_points = np.array(self.axis_points, dtype='float32')
        return cv2.transform(axis_points, rotation_matrix)

    def rotate_image(self, rotation_matrix):
        """Rotate an image using a rotation matrix."""
        rows, cols, _ = self.output_img.shape
        size = (cols, rows)
        self.output_img = cv2.warpAffine(
            self.output_img, rotation_matrix, size)

    def calculate_origin(self, rotated_axis_points):
        """Determine image origin location from dot axis compasses."""
        origin = []
        for i in range(2):
            diffs = rotated_axis_points[i] - rotated_axis_points[i + 1]
            avg_diffs = np.mean(diffs[:, 1 - i])
            if abs(avg_diffs) < 10:
                self.node_.get_logger().info('Warning: small deltas.')
            origin.append(1 if avg_diffs > 0 else 0)
        both = sum(origin) % 2 == 0
        origin = [1 - o for o in origin] if both else origin
        return origin

    def calculate_scale(self):
        """Use pattern dimensions to calculate image pixel scale."""
        # first circle in first row
        x_1, y_1 = self.dot_images[AXIS_INDEX['init']]['circles'][0][0]
        # last circle in first row
        x_2, y_2 = self.dot_images[AXIS_INDEX['init']]['circles'][4][0]
        pixel_separation = math.sqrt((x_2 - x_1) ** 2 + (y_2 - y_1) ** 2)
        return self.row_length() / pixel_separation

    def draw_origin(self):
        """Draw axis compass at image origin."""
        origin = self.calibration_data['image_bot_origin_location']
        origin_x = origin[0] * self.center[0] * 2
        origin_y = origin[1] * self.center[1] * 2
        cv2.line(self.output_img, (origin_x, origin_y + 100),
                 (origin_x, origin_y - 100), AXIS_COLORS[AXIS_INDEX['y']], 10)
        cv2.line(self.output_img, (origin_x - 100, origin_y),
                 (origin_x + 100, origin_y), AXIS_COLORS[AXIS_INDEX['x']], 10)

    def calculate_parameters(self):
        """Calculate camera calibration data."""
        rotation = np.mean(self.rotation_angles)
        rotation_matrix = self.generate_rotation_matrix(rotation)
        self.rotate_image(rotation_matrix)
        rotated_axis_points = self.rotate_points(rotation_matrix)
        origin = self.calculate_origin(rotated_axis_points)
        scale = self.calculate_scale()
        z_coordinate = self.dot_images[AXIS_INDEX['init']]['coordinates']['z']
        # save parameters
        self.calibration_data['center_pixel_location'] = list(self.center)
        self.calibration_data['image_bot_origin_location'] = origin
        self.calibration_data['total_rotation_angle'] = round(rotation, 2)
        self.calibration_data['camera_z'] = z_coordinate
        self.calibration_data['coord_scale'] = round(scale, 4)
        calibration_data_yaml = {
            'center_pixel_location': list(self.center),
            'image_bot_origin_location': list (origin),
            'total_rotation_angle': float(rotation),
            'camera_z': float(z_coordinate),
            'coord_scale': float(scale)
        }
        config_file = os.path.join(self.config_directory_,'camera_calibration.yaml')

        # Write the dictionary to a YAML file
        with open(config_file, 'w') as file:
            try:
                yaml.dump(calibration_data_yaml, file, default_flow_style=False)
            except yaml.YAMLError as e:
                self.node_.get_logger().warn(f"Error reading YAML file: {e}")
                return None
            
        self.node_.get_logger().info(f'Calibration data saved to {config_file}')

    def save_image(self, img=None, name='output'):
        """Save output image."""
        if img is None:
            img = self.output_img
        title = 'pattern_calibration'
        filename = '{}_{}_{}.jpg'.format(title, int(time.time()), name)
        filename = os.path.join(self.config_directory_,filename)
        cv2.imwrite(filename, img)
        cv2.imwrite('/tmp/images/{}'.format(filename), img)

    def calibrate(self):
        """Calibrate camera. Requires three translated images of dot grid."""
        self.get_initial_img_info()
        self.find_pattern_in_all()
        if not self.success_flag:
            return self.success_flag
        self.combine_data()
        self.calculate_parameters()
        self.draw_origin()
        return self.success_flag

