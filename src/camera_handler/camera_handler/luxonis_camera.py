#!/usr/bin/env python3

# ROS2 Imports
import rclpy
from rclpy.node import Node
from farmbot_interfaces.srv import StringRepReq
from camera_handler.luxonis_publisher import CameraNode
from camera_handler.panorama import Panorama
from camera_handler.calib import CalibrateCamera

class LuxonisCameraController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("LuxonisCameraController")
        
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
        Take a picture of the map and stitch it accordingly
        '''
        # Assuming request success
        response.data = 'SUCCESS'
        msg = request.data.split(' ')

        if not request.data:
            self.get_logger().warn('You need to parse panorama command! Request ignored')
            response.data = 'FAILED'
            return response

        # if the calib handle is parsed, then reload the calibration
        if request.data == 'Calib':
            self.take_picture_.load_config()
            return response
        elif len(msg) == 3:
            if float(msg[2]) != 0.0:
                self.get_logger().warn('Z Axis is not in home position for the panorama stitching! Panorama command cancelled')
                response.data = 'FAILED'
                return response
            self.panorama_.stitch_image_onto_map(x = float(msg[0]), y = float(msg[1]))
        else:
            self.get_logger.warn('Command not recognized! Request ignored')
            response.data = 'FAILED'
            return response


        # Sequencing constructed successfully and server returns it

        self.get_logger().info('Picture stitched to the panorama successfully')
        return response
    
    ## TODO both of this
    def luxonis_calibration(self, request, response):
        
        # Add here the coordinate commands and the camera picture commands
        info = request.data.split(' ')
        if request.data == 'GET': # USE THE .data TO READ FROM YOUR REQUEST STRING
            response.data = self.calib_.get_sequence()
            self.get_logger().info('Camera Calibration command sequence formed')
        elif len(info) != 5:
            self.get_logger().warn('Parsed calibration command incomplete. Command ignored!')
            response.data = 'FAILED'
        else:
            response.data = self.calib_.calibrate_camera(run = int(info[1]), x = float(info[2]), y = float(info[3]), z = float(info[4]))
          
        return response

    def luxonis_panorama_sequence_server(self, request, response):
        ## Add here the coordinate commands and the camera picture commands
        if request.data: # USE THE .data TO READ FROM YOUR REQUEST STRING
            response.data = "WRITE THE COMMANDS HERE"
        # Sequencing constructed successfully and server returns it

        self.get_logger().info("Panorama sequence formed successfully")
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