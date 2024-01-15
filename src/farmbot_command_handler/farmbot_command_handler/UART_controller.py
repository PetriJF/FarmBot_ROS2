#!/usr/bin/env python3

# ROS2 Imports
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.clock import ROSClock

# Standard Imports
import os
import time

class UARTController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("UARTController")

        # Get the path to the logging directory and 
        #UART_LOG_PATH = os.path.join(self.get_namespace(), 'log', 'UART_Logs')
        #os.makedirs(UART_LOG_PATH, exist_ok=True)

        # Create a log file based on the node initialization date and time
        #timestamp = ROSClock().now().to_msg()
        #LOG_FILE_NAME = time.strftime("uart_log_%d_%m_%Y_%H_%M_%S.txt", time.localtime(timestamp.sec))
        #LOG_FILE_PATH = os.path.join(UART_LOG_PATH, LOG_FILE_NAME)

        # Open the log file in append mode
        #self.uart_log_file_ = open(LOG_FILE_PATH, 'a')
        
        self.uart_cmd_ = String()

        serialPort = '/dev/ttyACM0'
        serialSpeed = 115200
        checkUartFreq = 100

        # Node subscripters and publishers
        self.uartTxSub_ = self.create_subscription(String, 'uart_transmit', self.uartTransmitCallback, 10)
        
        # Initialize Serial Communication
        self.ser_ = serial.Serial(serialPort, serialSpeed, timeout=1)
        self.ser_.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.timer = self.create_timer(1.0 / checkUartFreq, self.uartReceive)

        # Log the initialization
        self.get_logger().info("State Controller Initialized..")

    def uartTransmitCallback(self, message = String):
        if message.data[-1] != '\n':
            message.data += "\n"
        
        self.log_uart(transmit = True, cmd = message)
        self.ser_.write(message.data.encode('utf-8'))

    def uartReceive(self):
        line = self.ser_.readline().decode('utf-8').rstrip()
        
        if line:
            self.get_logger().info(f"Received message: {line}")

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message):
        self.log_uart(receive = True, cmd = message)


    def log_uart(self, transmit = False, receive = False, cmd = String):
        pass
        #if transmit and receive:
        #    self.get_logger().error("A command can't be logged as both transmitted and received at the same time")
        #elif not transmit and not receive:
        #    self.get_logger().error("You need to choose whether the command was transmitted or received")
        #else:
        #    timestamp = ROSClock().now().to_msg()
        #    message_type = "Sent" if transmit else "Received"
        #    self.uart_log_file_.write(f"{timestamp.sec:02d}_{timestamp.nanosec:09d} - \
        #                                   {message_type}: {cmd}\n")

    def destroy_node(self):
        # Close the UART and log file when the node is destroyed
        self.ser_.close()
        self.uart_log_file_.close()


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