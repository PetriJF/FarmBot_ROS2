#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardTeleOp(Node):
    '''
    Node used for recording keyboard commands and sending them forward to the
    farmbot controller for interpretation and execution
    '''
    
    # Node contructor
    def __init__(self):
        super().__init__('KeyboardController')

        # Keyboard publisher
        self.cmd_ = String()
        self.input_pub_ = self.create_publisher(String, 'keyboard_topic', 10)

        # Log the initialization
        self.get_logger().info('Keyboard Controller Initialized..')

        self.get_logger().info('''\n
                               This is a keyboard based controller for the ROS2 Farmbot 
                               Controllers. The commands accepted can be found in the 
                               Documentation under High Level Commands.\n
                               NOTE: The commands here DO NOT automatically enter the
                               sequencer as they hold execution priority! An exception
                               is shown for sequencing commands such as 'P_4' for watering.''')

    def check_input(self):
        '''
        Checks the input stream for valid commands and sends them on.
        Commands can be simple keys (e.g. T_1_1) or compound commands (e.g. P_1), where the
        key is followed by more information that is to be used by the farmbot
        '''
        # Valid Keys and Commands
        valid_keys = ('e', 'E', 'w', 'a', 's', 'd', '1', '2', '3', 'h', 'j', 'k', 'l', 'fh',
                      'o', 'p',
                      'T_1_1', 'T_1_2', 'T_2_1', 'T_2_2', 'T_3_1', 'T_3_2',
                      'T_4_1', 'T_4_2', 'T_5_1', 'T_5_2', 'T_6_1', 'T_6_2',
                      'P_3', 'P_4', 'P_5', 'P_9', 'I_0', 'I_1', 'I_2', 'I_3', 'I_4', 'D_C', 'D_L_1', 'D_L_0',
                      'D_W_1', 'D_W_0', 'D_V_1', 'D_V_0',
                      'H_0', 'H_1', 'D_S_C', 'P4_0', 'P4_1')
        compound_cmds = ('C_0', 'P_1', 'P_2', 'C_1', 'C_2', 'T_1_0', 'T_2_0',
                         'T_3_0', 'T_4_0', 'T_5_0', 'T_6_0', 'S_1_0', 'S_2_0',
                         'S_3_0', 'M', 'M_S', 'CONF', 'H_2', 'M_S')
        # Record the user input
        user_input = input('\nEnter command: ')
        
        # Send the user input to the farmbot controller if it is a valid key or command
        if user_input in valid_keys or user_input.split(' ')[0] in compound_cmds:
            self.cmd_.data = user_input
            self.input_pub_.publish(self.cmd_)
        else:
            print('Invalid input\n')

def main(args = None):
    rclpy.init(args = args)

    keyboard_node = KeyboardTeleOp()
    
    try:
        while rclpy.ok():
            keyboard_node.check_input()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()