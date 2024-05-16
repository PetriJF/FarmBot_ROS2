#!/usr/bin/env python3

# ROS2 Imports
import rclpy
from rclpy.node import Node
from farmbot_interfaces.srv import StringRepReq
from camera_handler.luxonis_publisher import CameraNode
from camera_handler.panorama import Panorama
from camera_handler.calib import CalibrateCamera
from ament_index_python.packages import get_package_share_directory
import os
import math

class LuxonisCameraController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("LuxonisCameraController")
        # Loading the panorama and calibration modules
        self.panorama_ = Panorama(self)
        self.calib_ = CalibrateCamera(self)

        # Sequencing Service Server
        self.panorama_sequencing_server_ = self.create_service(StringRepReq, 'panorama_sequence', self.luxonis_panorama_sequence_server)
        
        # Sequencing Service Server
        self.panorama_server_ = self.create_service(StringRepReq, 'form_panorama', self.stitch_image_server)
        self.take_picture_ = CameraNode(self)

        # Camera Calibration Server
        self.calibration_server_ = self.create_service(StringRepReq, 'calibrate_luxonis', self.luxonis_calibration)

        # Log the initialization
        self.get_logger().info("Luxonis Camera Handler Initialized..")

    def stitch_image_server(self, request, response):
        '''
        Service server that handles the luxonis camera node.
        Primary task is taking a picture of the map and stitching it accordingly
        '''
        # Assuming request success
        response.data = 'SUCCESS'
        # Getting the command information
        msg = request.data.split(' ')

        # Command information not set
        if not request.data:
            self.get_logger().warn('You need to parse panorama command! Request ignored')
            response.data = 'FAILED'
            return response

        # Requesting the camera to laod the calibration configuration
        if request.data == 'CALIB':
            self.take_picture_.load_config()
            return response
        elif request.data.split(' ')[0] == 'MAP':
            info = request.data.split(' ')
            self.panorama_.map_x = float(info[1])
            self.panorama_.map_y = float(info[2])
            self.get_logger().info(f'Updated camera map dimensions to {self.panorama_.map_x} and {self.panorama_.map_y}')
        elif request.data.split(' ')[0] == 'MOSAIC':
            # Save image for mosaic
            # Sequencing constructed successfully and server returns it
            self.panorama_.save_image_for_mosaic(num = int(msg[1]))
            self.get_logger().info('Picture saved for mosaic successfully')
        elif len(msg) == 3: # Ensuring the command information is complete
            if float(msg[2]) != 0.0:    # Ensuring the z-axis is homed
                self.get_logger().warn('Z Axis is not in home position for the panorama stitching! Panorama command cancelled')
                response.data = 'FAILED'
                return response
            # Stitch image to panorama
            # Sequencing constructed successfully and server returns it
            self.panorama_.stitch_image_onto_map(x = float(msg[0]), y = float(msg[1]))
            self.get_logger().info('Picture stitched to the panorama successfully')
        else:
            self.get_logger.warn('Command not recognized! Request ignored')
            response.data = 'FAILED'
            return response

        return response
    
    def luxonis_calibration(self, request, response):
        '''
        Service server that handles the luxonis camera calibration.
        It forms a command sequence when the 'GET' request is received
        and then handles subsequent commands that come from the sequence
        manager.
        '''
        # Retrieve command information
        info = request.data.split(' ')

        if request.data == 'GET':   # Get calibration sequence
            response.data = self.calib_.get_sequence()
            self.get_logger().info('Camera Calibration command sequence formed')
        elif len(info) != 5:        # Check if calibration command information is complete
            self.get_logger().warn('Parsed calibration command incomplete. Command ignored!')
            response.data = 'FAILED'
        else:                       # Take picture and use it for calibration
            response.data = self.calib_.calibrate_camera(run = int(info[1]), x = float(info[2]), y = float(info[3]), z = float(info[4]))
        
        # Load the new config into the camera node
        if response.data == 'COMPLETE':
            self.take_picture_.load_config()

        # Return outcome
        return response

    def luxonis_panorama_sequence_server(self, request, response):
        '''
        Service server that creates a sequence for making taking 
        pictures at multiple positions through the farmbot working
        area and stitching them into a large panorama.
        '''
        seq = ''
        
        # In case settings are needed
        
        x_inc, y_inc = self.panorama_.get_panorama_increments()
        self.get_logger().info(f'Panorama increments {x_inc}, {y_inc}')
        
        if self.panorama_.map_x != -1.0 and self.panorama_.map_y != -1.0:
            self.get_logger().info(f'Panorama max_x: {self.panorama_.map_x}, map_y: {self.panorama_.map_y}')
            x_pos_count = math.ceil(self.panorama_.map_x / x_inc)
            y_pos_count = math.ceil(self.panorama_.map_y / y_inc)

            x_inc = self.panorama_.map_x / x_pos_count
            y_inc = self.panorama_.map_y / y_pos_count
            num = int(1)
            for y_pos in range(y_pos_count + 1):
                # Determine the range for x_pos based on the current row (y_pos)
                if y_pos % 2 == 0:
                    # Even row: left to right
                    x_range = range(x_pos_count + 1)
                else:
                    # Odd row: right to left
                    x_range = range(x_pos_count, -1, -1)
                
                for x_pos in x_range:
                    x = int(x_pos * x_inc)
                    y = int(y_pos * y_inc)
                    seq += f"CC_P\n{x} {y} 0.0\n"
                    # Conditionally add either a mosaic or panoramic view command based on request.data
                    if request.data == 'MOSAIC':
                        seq += f"VC_P\nMOSAIC {num:03}\n"  # Using zero-padded numbering
                    else:
                        seq += 'VC_P\nPAN\n'
                    num += 1
                    
            self.get_logger().info('Panorama sequence formed successfully')
        else:
            self.get_logger().warn('Could not form panorama sequence')


        response.data = seq

        return response

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    luxonis_camera = LuxonisCameraController()
    
    try:
        rclpy.spin(luxonis_camera)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()