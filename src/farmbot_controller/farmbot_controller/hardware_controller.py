#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, ServoCommand
from std_msgs.msg import String


class HardwareController(Node):
    # Node contructor
    def __init__(self):
        super().__init__("HardwareController")
        
        # The encoder readings for the position of the end effector relative to the gantry map
        self.x_, self.y_, self.z_ = self.getPosition()

        self.uart_cmd_ = String()

        # Node subscripters and publishers
        self.gantrySub_ = self.create_subscription(GantryCommand, 'move_gantry', self.gantryMovementCallback, 10)
        self.servoSub_ = self.create_subscription(ServoCommand, 'move_servo', self.servoMovementCallback, 10)
        self.uartTxPub_ = self.create_publisher(String, 'uart_transmit', 10)
        
        # Log the initialization
        self.get_logger().info("Hardware Controller Initialized..")


    def gantryMovementCallback(self, moveCommand = GantryCommand):
        # Getting the movement mode
        mode = moveCommand.mode
        # Getting the target location and the speed
        x, y, z = moveCommand.x, moveCommand.y, moveCommand.z
        max_x, max_y, max_z = moveCommand.a, moveCommand.b, moveCommand.c

        # Form the GCode command for the "move at location" action
        self.uart_cmd_.data = ("G01 " if mode else "G00 ") + str(x) + " " + str(y) + " " + str(z) + ("" if mode else (" " + str(max_x) + " " + str(max_y) + " " + str(max_z)))

        self.uartTxPub_.publish(self.uart_cmd_)

        self.get_logger().info(self.uart_cmd_.data)

    def servoMovementCallback(self, servoCommand = ServoCommand):
        valid_servo_pins = (4, 5, 6, 11)

        if (servoCommand.pin in valid_servo_pins):
            self.uart_cmd_.data = "F61 " + str(servoCommand.pin) + " " + str(servoCommand.ang)

            self.uartTxPub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error(str(servoCommand.pin) + " is not within the valid servo pins(4, 5, 6, 11)!")

    # TODO: Make it return the encoder based position relative to the Home location
    # TODO: Make it record the position of the last position before a shutdown and return it on a boot up.
    def getPosition(self):
        return 0.0, 0.0, 0.0

def main(args = None):
    rclpy.init(args = args)

    gantryControlNode = HardwareController()
    
    try:
        rclpy.spin(gantryControlNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()