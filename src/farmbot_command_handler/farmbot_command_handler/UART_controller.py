#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class UARTController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("UARTController")

        self.uart_cmd_ = String()

        serialPort = '/dev/ttyACM0'
        serialSpeed = 115200
        checkUartFreq = 10

        # Node subscripters and publishers
        self.uartTxSub_ = self.create_subscription(String, 'uart_transmit', self.uartTransmitCallback, 10)
        
        # Initialize Serial Communication
        self.ser_ = serial.Serial(serialPort, serialSpeed, timeout=1)
        self.ser_.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.timer = self.create_timer(1.0 / checkUartFreq, self.uartReceive)

        # Log the initialization
        self.get_logger().info("State Controller Initialized..")

    def uartTransmitCallback(self, cmd = String):
        if cmd.data[-1] != '\n':
            cmd.data += "\n"
        
        self.ser_.write(cmd.data.encode('utf-8'))

    def uartReceive(self):
        line = self.ser_.readline().decode('utf-8').rstrip()
        
        if line:
            self.get_logger().info(f"Received message: {line}")

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message):
        self.get_logger().info(f"Handling message: {message}")

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    uartControlNode = UARTController()
    
    try:
        rclpy.spin(uartControlNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()