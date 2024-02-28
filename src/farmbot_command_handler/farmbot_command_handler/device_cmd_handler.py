#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray
from farmbot_interfaces.msg import I2CCommand, PinCommand


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

        self.water_cmd_sub_ = self.create_subscription(Int64MultiArray, 'water_command', self.water_cmd_callback, 10)
        self.i2c_cmd_sub_ = self.create_subscription(I2CCommand, 'i2c_command', self.i2c_cmd_callback, 10)
        self.pin_cmd_sub_ = self.create_subscription(PinCommand, 'pin_command', self.pin_cmd_callback, 10)
        self.uart_tx_pub_ = self.create_publisher(String, 'uart_transmit', 10)

        # Log the initialization
        self.get_logger().info('Device Command Handler Initialized..')

    def water_cmd_callback(self, cmd: Int64MultiArray):
        '''
        Watering style command. Used to set the watering to be either time based (1) or
        measured using a flow meter (2)
        '''
        # Valid watering commands
        valid_commands = (1, 2)

        if cmd.data[0] in valid_commands:
            if cmd.data[1] <= 0:
                self.get_logger().warning('The time constraint/volume constraint was not set!')
            else:
                self.uart_cmd_.data = 'F0' + str(cmd.data[0]) + (' T' if cmd.data[0] == 1 else ' N') + str(cmd.data[1])
                
                self.uart_tx_pub_.publish(self.uart_cmd_)
                self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error('Wrong watering command type! First element should be 1 (timed pulses msec) or 2 (volume pulses)!')

    def i2c_cmd_callback(self, cmd: I2CCommand):
        '''
        I2C Command Handler. Used to read or write to the I2C devices connected
        to the farmduino

        Args:
            cmd{I2CCommand}: contains the read or write command information
        '''
        if cmd.mode:    # I2C SET
            self.uart_cmd_.data = 'F51 E' + str(cmd.e) + ' P' + str(cmd.p) + ' V' + str(cmd.v)
        else:           # I2C READ
            self.uart_cmd_.data = 'F52 E' + str(cmd.e) + ' P' + str(cmd.p)
        
        self.uart_tx_pub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    def pin_cmd_callback(self, cmd: PinCommand):
        '''
        Pin Command Handler. Handles reading (F42) and setting (F41, F43, F44)
        pins. Note that this covers both analog and digital pins, and the pin
        mode needs to be set accordingly!

        Args{PinCommand}: contains the pin command information
        '''
        # Check if in set mode, but no set command selected
        if cmd.mode and not cmd.set_io and not cmd.set_value and not cmd.set_value2:
            self.get_logger().error('Pin is in SET mode, but no SET CASE was selected. Use set_io, set_value or set_value2 to indicate\
                                        what set mode you want!')
        else:
            # SET mode for the pin
            if cmd.mode:
                if cmd.set_io:      # NOTE: pin_mode should be 0 for input and 1 for output
                    self.uart_cmd_.data = 'F43 P' + str(cmd.pin) + ' M' + str(int(cmd.pin_mode))
                elif cmd.set_value: # NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = 'F41 P' + str(cmd.pin) + ' V' + str(cmd.value) + ' M' + str(int(cmd.pin_mode))
                elif cmd.set_value2:# NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = 'F44 P' + str(cmd.pin) + ' V' + str(cmd.value) + ' W' + str(cmd.value2) + ' T' + str(cmd.delay) + ' M' + str(int(cmd.pin_mode))
            # READ mode for the pin
            else:
                self.uart_cmd_.data = 'F42 P' + str(cmd.pin) + ' M' + str(int(cmd.pin_mode))

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