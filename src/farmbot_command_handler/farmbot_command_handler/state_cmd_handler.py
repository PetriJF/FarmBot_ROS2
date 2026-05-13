#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from farmbot_interfaces.msg import ParameterCommand, StateCommand, StatusCommand

class StateCmdHandler(Node):
    '''
    Node handling commands for manipulating the farmbot's parameters, state and status
    
    Input Topics:
        - /parameter_command -> for manipulating the running parameters of the farmbot
        - /state_command -> for setting the different state commands (reporting end stop, electronic stop, etc)
        - /status_command -> for reading and writing status commands from the farmbot
    Output Topics:
        - /uart_transmit -> Transmits the commands in F-Code (Farmduino's version of GCode)
    '''
    # Node contructor
    def __init__(self):
        super().__init__('StateCmdHandler')

        # String standard interface used for forming the uart message
        self.uart_cmd_ = String()

        # Node subscripters and publishers
        self.param_sub_ = self.create_subscription(String, 'parameter_command', self.param_cmd_callback, 10)
        self.state_sub_ = self.create_subscription(String, 'state_command', self.state_cmd_callback, 10)
        self.status_sub_ = self.create_subscription(StatusCommand, 'status_command', self.status_cmd_callback, 10)
        self.uart_tx_pub_ = self.create_publisher(String, 'uart_transmit', 10)
        
        # Log the initialization
        self.get_logger().info('State Command Handler Initialized..')
    
    def param_cmd_callback(self, cmd: String):
        '''
        Parameter Command Handler. Used to manipulate the Farmduino
        parameters by manipulating code friendly commands into the
        GCode (FCode) commands utilized in the UART communication
        scheme

        Args:
            cmd{String}: Interface containing all the parameter commands
        '''
        command : list = cmd.data.split(' ')
        
        if command[0] == 'True':        # List all parameters 
            self.uart_cmd_.data = 'F20'
        else:
            if command[2] == 'True':    # Read a parameter
                self.uart_cmd_.data = 'F21 P' + command[4]
            elif command[1] == 'True': # Write to a parameter
                self.uart_cmd_.data = 'F22 P' + command[4] + ' V' + command[5]
            elif command[3] == 'True': # Update a parameter (in the calibration state)
                self.uart_cmd_.data = 'F23 P' + command[4] + ' V' + command[5]

        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    def state_cmd_callback(self, cmd: String):
        '''
        State Command Handler. Used as the main pipeline for specific commands.
        E - End Stop, @ - Abort Movement, F09 - Report Endstop, F81 - Report
        Current Position and F83 - Report Software Version

        Args:
            cmd{String} - The state command.
        '''
        command : list = cmd.data.split(' ')
        state_counter = command.count('True')
        
        # Check that only one start command is in effect
        if state_counter != 1:
            self.get_logger().error('Make sure to include only 1 state handler command! Your input has ' + str(state_counter) + ' commands')
        else:
            if command[0] == 'True':               # Electronic stop
                self.uart_cmd_.data = 'E'
            elif command[1] == 'True':    # Abort current movement command
                self.uart_cmd_.data = '@'
            elif command[2] == 'True':       # Reset the Electronic stop
                self.uart_cmd_.data = 'F09'
            elif command[3] == 'True':      # Report End Stop
                self.uart_cmd_.data = 'F81'
            elif command[4] == 'True':      # Report Current Position
                self.uart_cmd_.data = 'F82'
            elif command[5] == 'True':        # Report Software Version
                self.uart_cmd_.data = 'F83'

            self.uart_tx_pub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)

    def status_cmd_callback(self, cmd: StatusCommand):
        '''
        Function that allows the read and write of a status command for the
        Farmbot. Check documentation for more information on status commands
        '''
        if cmd.mode:    # Write mode
            self.uart_cmd_.data = 'F32 P' + str(cmd.p) + ' V' + str(cmd.v)
        else:           # Read mode
            self.uart_cmd_.data = 'F31 P' + str(cmd.p)
        
        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    state_node = StateCmdHandler()
    
    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()