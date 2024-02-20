#!/usr/bin/env python3

# ROS2 Imports
import rclpy
from rclpy.node import Node
from farmbot_interfaces.srv import StringRepReq

class LuxonisCameraController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("LuxonisCameraController")

        # Sequencing Service Server
        self.panorama_sequencing_server_ = self.create_service(StringRepReq, 'panorama_sequence', self.luxonis_panorama_sequence_server)
        
        # Sequencing Service Server
        self.panorama_server_ = self.create_service(StringRepReq, 'form_panorama', self.luxonis_panorama_server)

        # Camera Calibration Server
        self.calibration_server_ = self.create_service(StringRepReq, 'calibrate_luxonis', self.luxonis_calibration)

        # Log the initialization
        self.get_logger().info("Luxonis Camera Handler Initialized..")

    def luxonis_panorama_server(self, request, response):
        ## Add here the coordinate commands and the camera picture commands
        if request.data: # USE THE .data TO READ FROM YOUR REQUEST STRING
            response.data = "WRITE THE COMMANDS HERE"
        # Sequencing constructed successfully and server returns it
        response.success = True
        self.get_logger().info("Picture stitched to the panorama successfully")
        return response
    
    def luxonis_calibration(self, request, response):
        ## Add here the coordinate commands and the camera picture commands
        if request.data: # USE THE .data TO READ FROM YOUR REQUEST STRING
            response.data = "WRITE THE COMMANDS HERE"
        # Sequencing constructed successfully and server returns it
        response.success = True
        self.get_logger().info("Luxonis camera calibrated successfully")
        return response

    def luxonis_panorama_sequence_server(self, request, response):
        ## Add here the coordinate commands and the camera picture commands
        if request.data: # USE THE .data TO READ FROM YOUR REQUEST STRING
            response.data = "WRITE THE COMMANDS HERE"
        # Sequencing constructed successfully and server returns it
        response.success = True
        self.get_logger().info("Panorama sequence formed successfully")
        return response



# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    panorama = LuxonisCameraController()
    
    try:
        rclpy.spin(panorama)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()