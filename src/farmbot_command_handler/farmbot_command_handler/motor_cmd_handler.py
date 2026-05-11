#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, ServoCommand, HomeCommand
from std_msgs.msg import String

class MotorCmdHandler(Node):
    '''
    Node handling motor and actuator commands.
    
    Input Topics:
        - /move_gantry -> for moving the gantry
        - /home_handler -> for homing or calibrating the axis for the farmbot
        - /move_servo -> for moving a servo attached to the farmbduino
    Output Topics:
        - /uart_transmit -> Transmits the commands in F-Code (Farmduino's version of GCode)
    '''
    # Node contructor
    def __init__(self):
        super().__init__('MotorCmdHandler')

        self.uart_cmd_ = String()

        # Node subscripters and publishers
        self.gantry_sub_ = self.create_subscription(String, 'move_gantry', self.gantry_cmd_callback, 10)
        self.home_sub_ = self.create_subscription(String, 'home_handler', self.home_cmd_callback, 10)
        self.servo_sub_ = self.create_subscription(String, 'move_servo', self.servo_cmd_callback, 10)
        self.uart_tx_pub_ = self.create_publisher(String, 'uart_transmit', 10)
        
        # Log the initialization
        self.get_logger().info('Motor Command Handler Initialized..')

    def gantry_cmd_callback(self, cmd: String):
        '''
        Handling gantry commands. Note that homing and calibration must be done through the
        home_handler topic
        '''
        # Getting the movement mode
        command: list = cmd.data.split(' ')

        mode = command[0]
        # Getting the target location and the speed
        x, y, z = command[1], command[2], command[3]
        max_x, max_y, max_z = command[4], command[5], command[6]

        # Form the GCode command for the "move at location" action
        self.uart_cmd_.data = ('G01 ' if mode == 'True' else 'G00 ') + 'X' + x + ' Y' + y + ' Z' + z \
                            + ('' if mode == 'True' else (' A' + max_x + ' B' + max_y + ' C' + max_z))

        self.uart_tx_pub_.publish(self.uart_cmd_)

        self.get_logger().info(self.uart_cmd_.data)

    def home_cmd_callback(self, cmd: String):
        '''
        Handling homing and calibration commands
        '''
        command: list = cmd.data.split(' ')

        if command[0] == 'True':            # Home all axis
            self.uart_cmd_.data = 'G28'
        elif command[1] == 'True': # Set the current position as home
            self.uart_cmd_.data = 'F84 ' + 'X' + ("1 " if command[3]=='True' else '0 ')\
                                         + 'Y' + ("1 " if command[4]=='True' else '0 ')\
                                         + 'Z' + ("1" if command[5]=='True' else '0')
        else:
            if command[3] == 'False' and  command[4] == 'False' and command[5] == 'False':
                self.get_logger().error('No axis selected for homing/calibration!')
            else:
                if command[3] == 'True' :              # Find home or calibrate x axis
                    self.uart_cmd_.data = 'F11' if command[2]=='False' else 'F14'
                if command[4] == 'True':            # Find home or calibrate y axis
                    self.uart_cmd_.data = 'F12' if command[2]=='False' else 'F15'
                if command[5] == 'True':            # Find home or calibrate z axis
                    self.uart_cmd_.data = 'F13' if command[2]=='False' else 'F16'

        
        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    def servo_cmd_callback(self, cmd: String):
        '''
        Handles servo command interpretation. Note that servos can be attached only on pins
        4, 5, 6 and 11 on the Farmduino
        '''
        command: list = cmd.data.split(' ')

        # The valid servo pins
        valid_servo_pins = (4, 5, 6, 11)
        
        # If the servo pin is within the valid pins:
        if (int(command[0]) in valid_servo_pins):
            # Request servo attached to PIN to be rotated to ANGLE
            self.uart_cmd_.data = 'F61 P' + command[0] + ' V' + command[1]

            self.uart_tx_pub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error(command[0] + ' is not within the valid servo pins(4, 5, 6, 11)!')

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    motor_cmd_node = MotorCmdHandler()
    
    try:
        rclpy.spin(motor_cmd_node)
    except KeyboardInterrupt:
        motor_cmd_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()