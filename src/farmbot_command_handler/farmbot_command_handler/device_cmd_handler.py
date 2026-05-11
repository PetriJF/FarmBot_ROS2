#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DeviceCmdHandler(Node):
    '''
    Node handling device and peripheral commands.
    
    Input Topics:
        - /water_command -> for water commands containing a flow meter or a pulse meter
        - /i2c_command -> for commands around I2C devices connected to the farmduino
        - /pin_command -> for commands on the farmduino pins
    Output Topics:
        - /uart_transmit -> Transmits the commands in F-Code (Farmduino's version of GCode)
    '''
    # Node contructor
    def __init__(self):
        super().__init__('DeviceCmdHandler')
        
        self.uart_cmd_ = String()

        self.water_cmd_sub_ = self.create_subscription(String, 'water_command', self.water_cmd_callback, 10)
        self.i2c_cmd_sub_ = self.create_subscription(String, 'i2c_command', self.i2c_cmd_callback, 10)
        self.pin_cmd_sub_ = self.create_subscription(String, 'pin_command', self.pin_cmd_callback, 10)
        self.uart_tx_pub_ = self.create_publisher(String, 'uart_transmit', 10)

        # Log the initialization
        self.get_logger().info('Device Command Handler Initialized..')

    def water_cmd_callback(self, cmd: String):
        '''
        Watering style command. Used to set the watering to be either time based (1) or
        measured using a flow meter (2)
        '''
        command : list = cmd.data.split(' ')
        print(command)
        # Valid watering commands
        valid_commands = (1, 2)

        if int(command[0]) in valid_commands:
            if int(command[1]) <= 0:
                self.get_logger().warning('The time constraint/volume constraint was not set!')
            else:
                self.uart_cmd_.data = 'F0' + command[0] + (' T' if int(command[0]) == 1 else ' N') + command[1]

                self.uart_tx_pub_.publish(self.uart_cmd_)
                self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error('Wrong watering command type! First element should be 1 (timed pulses msec) or 2 (volume pulses)!')

    def i2c_cmd_callback(self, cmd: String):
        '''
        I2C Command Handler. Used to read or write to the I2C devices connected
        to the farmduino

        Args:
            cmd{String}: contains the read or write command information
        '''
        command : list = cmd.data.split(' ')

        if command[0] == 'True':    # I2C SET
            self.uart_cmd_.data = 'F51 E' + str(command[1]) + ' P' + str(command[2]) + ' V' + str(command[3])
        else:           # I2C READ
            self.uart_cmd_.data = 'F52 E' + str(command[1]) + ' P' + str(command[2])
        
        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    def pin_cmd_callback(self, cmd: String):
        '''
        Pin Command Handler. Handles reading (F42) and setting (F41, F43, F44)
        pins. Note that this covers both analog and digital pins, and the pin
        mode needs to be set accordingly!

        Args{String}: contains the pin command information
        '''
        command : list = cmd.data.split(' ')

        # Check if in set mode, but no set command selected
        if command[0] == 'True' and command[1] == 'False' and command[2] == 'False' and command[3] == 'False':
            self.get_logger().error('Pin is in SET mode, but no SET CASE was selected. Use set_io, set_value or set_value2 to indicate\
                                        what set mode you want!')
        else:
            # SET mode for the pin
            if command[0] == 'True':
                if command[1] == 'True':      # NOTE: pin_mode should be 0 for input and 1 for output
                    self.uart_cmd_.data = 'F43 P' + command[4] + ' M' + command[8]
                elif command[2] == 'True': # NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = 'F41 P' + command[4] + ' V' + command[5] + ' M' + command[8]
                elif command[3] == 'True':# NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = 'F44 P' + command[4] + ' V' + command[5] + ' W' + command[6] + ' T' + command[7] + ' M' + command[8]
            # READ mode for the pin
            else:
                self.uart_cmd_.data = 'F42 P' + command[5] + ' M' + command[8]

            self.uart_tx_pub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)

def main(args = None):
    rclpy.init(args = args)

    device_node = DeviceCmdHandler()
    
    try:
        rclpy.spin(device_node)
    except KeyboardInterrupt:
        device_node.destroy_node()

    device_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()