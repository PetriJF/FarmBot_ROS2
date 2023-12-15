#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray


class DeviceController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("DeviceController")
        
        self.uart_cmd_ = String()

        self.waterCmdSub_ = self.create_subscription(Int64MultiArray, 'water_command', self.waterCommandHandler, 10)
        self.uartTxPub_ = self.create_publisher(String, 'uart_transmit', 10)

        # Log the initialization
        self.get_logger().info("Device Controller Initialized..")

    def waterCommandHandler(self, cmd = Int64MultiArray):
        valid_commands = (1, 2)
        if cmd.data[0] in valid_commands:
            if cmd.data[1] <= 0:
                self.get_logger().warning("The time constraint/volume constraint was not set!")
            else:
                self.uart_cmd_.data = "F0" + str(cmd.data[0]) + " " + str(cmd.data[1])
                
                self.uartTxPub_.publish(self.uart_cmd_)
                self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error("Wrong watering command type! First element should be 1 (timed pulses msec) or 2 (volume pulses)!")
    

def main(args = None):
    rclpy.init(args = args)

    deviceControlNode = DeviceController()
    
    try:
        rclpy.spin(deviceControlNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()