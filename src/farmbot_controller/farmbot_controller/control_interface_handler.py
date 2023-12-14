#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ControlInterfaceHandler(Node):
    def __init__(self):
        super().__init__("ControlInterfaceHandler")
        self.get_logger().info("Control Interface Handler Initialized..")
        

def main(args = None):
    rclpy.init(args = args)

    intCtrlNode = ControlInterfaceHandler()

    rclpy.shutdown()


if __name__ == '__main__':
    main()