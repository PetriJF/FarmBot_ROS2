#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class InterfaceController(Node):
    def __init__(self):
        super().__init__("InterfaceController")
        self.get_logger().info("Interface Controller Initialized..")
        

def main(args = None):
    rclpy.init(args = args)

    intCtrlNode = InterfaceController()

    rclpy.shutdown()


if __name__ == '__main__':
    main()