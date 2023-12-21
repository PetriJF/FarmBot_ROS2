#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray
from farmbot_interfaces.msg import I2CCommand, PinCommand


class DeviceController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("DeviceController")
        
        self.uart_cmd_ = String()

        self.waterCmdSub_ = self.create_subscription(Int64MultiArray, 'water_command', self.waterCommandHandler, 10)
        self.i2cCmdSub_ = self.create_subscription(I2CCommand, 'i2c_command', self.i2cCommandHandler, 10)
        self.pinCmdSub_ = self.create_subscription(PinCommand, 'pin_command', self.pinCommandHandler, 10)
        self.uartTxPub_ = self.create_publisher(String, 'uart_transmit', 10)

        # Log the initialization
        self.get_logger().info("Device Controller Initialized..")

    # Function handling the watering commands
    def waterCommandHandler(self, cmd = Int64MultiArray):
        # Valid watering commands
        valid_commands = (1, 2)

        if cmd.data[0] in valid_commands:
            if cmd.data[1] <= 0:
                self.get_logger().warning("The time constraint/volume constraint was not set!")
            else:
                self.uart_cmd_.data = "F0" + str(cmd.data[0]) + (" T" if cmd.data[0] == 1 else " N") + str(cmd.data[1])
                
                self.uartTxPub_.publish(self.uart_cmd_)
                self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error("Wrong watering command type! First element should be 1 (timed pulses msec) or 2 (volume pulses)!")

    # Function handling the set or read on the I2C Bus
    def i2cCommandHandler(self, cmd = I2CCommand):
        if cmd.mode:    # I2C SET
            self.uart_cmd_.data = "F51 E" + str(cmd.e) + " P" + str(cmd.p) + " V" + str(cmd.v)
        else:           # I2C READ
            self.uart_cmd_.data = "F52 E" + str(cmd.e) + " P" + str(cmd.p)
        
        self.uartTxPub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    # Function handling the pin set and read commands
    def pinCommandHandler(self, cmd = PinCommand):
        # Check if in set mode, but no set command selected
        if cmd.mode and not cmd.set_io and not cmd.set_value and not cmd.set_value2:
            self.get_logger().error("Pin is in SET mode, but no SET CASE was selected. Use set_io, set_value or set_value2 to indicate\
                                        what set mode you want!")
        else:
            # SET mode for the pin
            if cmd.mode:
                if cmd.set_io:      # NOTE: pin_mode should be 0 for input and 1 for output
                    self.uart_cmd_.data = "F43 P" + str(cmd.pin) + " M" + str(int(cmd.pin_mode))
                elif cmd.set_value: # NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = "F41 P" + str(cmd.pin) + " V" + str(cmd.value) + " M" + str(int(cmd.pin_mode))
                elif cmd.set_value2:# NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd_.data = "F44 P" + str(cmd.pin) + " V" + str(cmd.value) + " W" + str(cmd.value2) + " T" + str(cmd.delay) + " M" + str(int(cmd.pin_mode))
            # READ mode for the pin
            else:
                self.uart_cmd_.data = "F42 P" + str(cmd.pin) + " M" + str(int(cmd.pin_mode))

            self.uartTxPub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)

def main(args = None):
    rclpy.init(args = args)

    deviceControlNode = DeviceController()
    
    try:
        rclpy.spin(deviceControlNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()