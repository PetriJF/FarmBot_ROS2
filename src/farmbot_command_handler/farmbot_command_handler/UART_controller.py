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

class UARTController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("UARTController")

        self.uart_cmd_ = String()

        serial_port = '/dev/ttyACM0'
        serial_speed = 115200
        check_uart_freq = 100
        tx_freq = 10

        # UART receive publisher
        self.uart_rx_pub_ = self.create_publisher(String, 'uart_receive', 10)

        # Farmbot state publisher
        self.farmbot_busy_ = Bool()
        self.farmbot_busy_.data = False
        self.farmbot_state_pub_ = self.create_publisher(Bool, 'busy_state', 10)

        # Node subscripters and publishers
        self.tx_queue_ = []
        self.uart_tx_sub_ = self.create_subscription(String, 'uart_transmit', self.uart_transmit_callback, 10)
        
        # Initialize Serial Communication
        self.ser_ = serial.Serial(serial_port, serial_speed, timeout=1)
        self.ser_.reset_input_buffer()
        # Create a timer to periodically check for incoming serial messages
        self.rx_timer_ = self.create_timer(1.0 / check_uart_freq, self.uart_receive)
        self.tx_timer_ = self.create_timer(1.0 / tx_freq, self.uart_transmit)

        # Used for setting the busy status on the ROS2 arch. while a command is running
        self.previous_cmd_ = ''

        # Request Response Action Server WIP
        self.uart_received_cmd_ = ''
        self.req_resp_server_ = ActionServer(
            self, GetUARTResponse, 'uart_response',
            execute_callback = self.request_response_server
        )

        # Log the initialization
        self.get_logger().info("UART Controller Initialized..")

    def uart_transmit(self):
        '''
        Takes commands from the queue (tx_queue_) and sends them
        to the farmbot through UART.
        The command is popped from the queue only if the blocker
        is not active (i.e. a command is not in the process of
        running or a response is not expected).
        '''
        
        # If the blocker is not up, and there are commands in the queue
        if not self.farmbot_busy_.data and self.tx_queue_:
            # Set the blocker flag
            self.farmbot_busy_.data = True
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
            # Clear the command from the queue
            message = self.tx_queue_.pop(0)
            
            # Ensure the command has an endline character at the end
            if message[-1] != '\n':
                message += "\n"

            # Record the transmitted command
            self.previous_cmd_ = message.split(' ')[0] if ' ' in message else message.split('\n')[0]
            self.get_logger().info(f"Sent message: {message}")
            # Send through UART the command
            self.ser_.write(message.encode('utf-8'))

    def uart_transmit_callback(self, message: String):
        '''
        Callback handling the commands that are queued to be sent
        to the farmbot through UART.
        Two cases:
            a) the command has priority (e.g. electronic-stop):
                The command bypasses the queue and the queue is reset
            b) standard command:
                The command is added at the end of the queue
        '''
        # Priority commands
        if message.data in ['E', 'F09', '@']:
            self.get_logger().info(f"Sent message: {message.data}")
            # Ensure the endline char at the end of the command
            if message.data[-1] != '\n':
                message.data += "\n"

            # Send command and reset everything
            self.ser_.write(message.data.encode('utf-8'))
            self.tx_queue_.clear()
            self.farmbot_busy_.data = False
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
        # Standard commands
        else:
            # Add the command to the queue
            self.tx_queue_.append(message.data)

    def uart_receive(self):
        '''
        Timer callback that reads from UART and handles the response
        codes and commands.
        '''
        # Read from serial
        line = self.ser_.readline().decode('utf-8').rstrip()
        
        # If a command is read, handle it
        if line:
            self.get_logger().info(f"Received message: {line}")

            # Call the callback function
            self.handle_message(line)

    def handle_message(self, message: str):
        '''
        Handles the command lines that are received through serial
        
        Args:
            message {str}: the command string
        '''
        # Record the message
        self.uart_received_cmd_ = message
        self.uart_cmd_.data = message
        
        # Blocking command codes
        blocking_cmds = ['G00', 'G01', 'G28', 'F11', 'F12', 'F13',
                         'F14', 'F15', 'F16', 'F20', 'F44']
        blocking_responses = ['R02', 'R03']
        # Commands that block until a response is reached
        request_cmds = ['F42', 'F21']
        response_cmds = ['R41', 'R21']

        # Extract the command code
        rep_code = (message).split(' ')[0]
        
        # If a running command has finished OR the response for a request was retrieved
        # OR the sent command was acknowledged by the farmbot
        if ((self.previous_cmd_ in blocking_cmds and rep_code in blocking_responses)
                or (self.previous_cmd_ in request_cmds and rep_code in response_cmds)
                or (self.previous_cmd_ not in blocking_cmds 
                    and self.previous_cmd_ not in request_cmds
                    and rep_code in ['R08'])):
            # Lower the blocking flag
            self.farmbot_busy_.data = False
            self.farmbot_state_pub_.publish(self.farmbot_busy_)
        
        # Send the reporting message for further processing by other nodes
        self.uart_rx_pub_.publish(self.uart_cmd_)

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

    
    def destroy_node(self):
        # Close the UART when the node is destroyed
        self.ser_.close()

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    uart_node = UARTController()
    
    try:
        rclpy.spin(uart_node)
    except KeyboardInterrupt:
        uart_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()