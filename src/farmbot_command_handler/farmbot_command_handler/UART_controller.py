#!/usr/bin/env python3

# ROS2 Imports
import rclpy
import serial
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import String, Bool
from farmbot_interfaces.action import GetUARTResponse
from rclpy.clock import ROSClock
from farmbot_interfaces.srv import StringRepReq

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
        txFreq = 10

        # UART receive publisher
        self.uartRxPub_ = self.create_publisher(String, 'uart_receive', 10)

        # Farmbot state publisher
        self.farmbot_busy_ = Bool()
        self.farmbot_state_pub_ = self.create_publisher(Bool, 'busy_state', 10)

        # Node subscripters and publishers
        self.txBlocker_ = False
        self.txQueue_ = []
        self.uartTxSub_ = self.create_subscription(String, 'uart_transmit', self.uartTransmitCallback, 10)
        
        # Initialize Serial Communication
        self.ser_ = serial.Serial(serialPort, serialSpeed, timeout=1)
        self.ser_.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rxTimer_ = self.create_timer(1.0 / checkUartFreq, self.uartReceive)
        self.txTimer_ = self.create_timer(1.0 / txFreq, self.uartTransmit)

        self.previous_cmd_ = ''

        # Request Response Action Server
        self.uart_received_cmd_ = ''
        self.req_resp_server_ = ActionServer(
            self, GetUARTResponse, 'uart_response',
            execute_callback = self.request_response_server
        )

        # Log the initialization
        self.get_logger().info("State Controller Initialized..")

    ## NOT IN USE WIP
    def request_response_server(self, goal_handle: ServerGoalHandle):
        # Get the request from the goal
        code = goal_handle.request.code
        timeout = rclpy.duration.Duration(seconds = goal_handle.request.timeout_sec)


        start_time = self.get_clock().now()
        # Execute the request
        while (self.get_clock().now() - start_time) < timeout or goal_handle.request.timeout_sec == -1:
            if code == self.uart_received_cmd_.split(' ')[0]:
                goal_handle.succeed()
                result = GetUARTResponse.Result()
                result.msg = self.uart_received_cmd_
                result.success = True
                result.cmd_type = goal_handle.request.cmd_type
                return result
            
        goal_handle.abort()
        return GetUARTResponse.Result()
        


    def uartTransmit(self):
        if not self.txBlocker_ and self.txQueue_:
            self.txBlocker_ = True
            self.farmbot_busy_.data = self.txBlocker_ 
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
            message = self.txQueue_.pop(0)
            
            if message[-1] != '\n':
                message += "\n"
        
            self.previous_cmd_ = message.split(' ')[0]
            self.log_uart(transmit = True, cmd = message)
            self.get_logger().info(f"Sent message: {message}")
            self.ser_.write(message.encode('utf-8'))

    def uartTransmitCallback(self, message = String):
        if message.data in ['E', 'F09', '@']:
            self.get_logger().info(f"Sent message: {message.data}")
            
            if message.data[-1] != '\n':
                message.data += "\n"

            self.ser_.write(message.data.encode('utf-8'))
            self.txQueue_.clear()
            self.txBlocker_ = False
            self.farmbot_busy_.data = self.txBlocker_ 
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
        else:
            self.txQueue_.append(message.data)

    def uartReceive(self):
        line = self.ser_.readline().decode('utf-8').rstrip()
        
        if line:
            self.get_logger().info(f"Received message: {line}")

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message):
        self.uart_received_cmd_ = message
        movement_cmds = ['G00', 'G01', 'G28', 'F11', 'F12', 'F13',
                         'F14', 'F15', 'F16']
        self.uart_cmd_.data = message
        
        reportCode = (message).split(' ')[0]
        if (self.previous_cmd_ in movement_cmds and reportCode in ['R02', 'R03']
                or self.previous_cmd_ not in movement_cmds and reportCode in ['R08']):
            self.txBlocker_ = False
            self.farmbot_busy_.data = self.txBlocker_ 
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
        
        self.uartRxPub_.publish(self.uart_cmd_)
        
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