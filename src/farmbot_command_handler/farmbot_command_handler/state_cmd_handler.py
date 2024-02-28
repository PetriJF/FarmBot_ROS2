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
        self.param_sub_ = self.create_subscription(ParameterCommand, 'parameter_command', self.param_cmd_callback, 10)
        self.state_sub_ = self.create_subscription(StateCommand, 'state_command', self.state_cmd_callback, 10)
        self.status_sub_ = self.create_subscription(StatusCommand, 'status_command', self.status_cmd_callback, 10)
        self.uart_tx_pub_ = self.create_publisher(String, 'uart_transmit', 10)
        
        # Log the initialization
        self.get_logger().info('State Command Handler Initialized..')
    
    def param_cmd_callback(self, cmd: ParameterCommand):
        '''
        Parameter Command Handler. Used to manipulate the Farmduino
        parameters by manipulating code friendly commands into the
        GCode (FCode) commands utilized in the UART communication
        scheme

        Args:
            cmd{ParameterCommand}: Interface containing all the parameter commands
        '''
        if cmd.list:        # List all parameters 
            self.uart_cmd_.data = 'F20'
        else:
            if cmd.read:    # Read a parameter
                self.uart_cmd_.data = 'F21 P' + str(cmd.param)
            elif cmd.write: # Write to a parameter
                self.uart_cmd_.data = 'F22 P' + str(cmd.param) + ' V' + str(cmd.value)
            elif cmd.update:# Update a parameter (in the calibration state)
                self.uart_cmd_.data = 'F23 P' + str(cmd.param) + ' V' + str(cmd.value)

        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    def state_cmd_callback(self, cmd: StateCommand):
        '''
        State Command Handler. Used as the main pipeline for specific commands.
        E - End Stop, @ - Abort Movement, F09 - Report Endstop, F81 - Report
        Current Position and F83 - Report Software Version

        Args:
            cmd{StateCommand} - The state command.
        '''
        
        state_sum = sum([cmd.estop, cmd.abort_movement, cmd.reset_estop,\
                   cmd.rep_end_stop, cmd.rep_curr_pos, cmd.rep_sw_ver])
        # Check that only one start command is in effect
        if state_sum != 1:
            self.get_logger().error('Make sure to include only 1 state handler command! Your input has ' + str(state_sum) + ' commands')
        else:
            if cmd.estop:               # Electronic stop
                self.uart_cmd_.data = 'E'
            elif cmd.abort_movement:    # Abort current movement command
                self.uart_cmd_.data = '@'
            elif cmd.reset_estop:       # Reset the Electronic stop
                self.uart_cmd_.data = 'F09'
            elif cmd.rep_end_stop:      # Report End Stop
                self.uart_cmd_.data = 'F81'
            elif cmd.rep_curr_pos:      # Report Current Position
                self.uart_cmd_.data = 'F82'
            elif cmd.rep_sw_ver:        # Report Software Version
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