#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, ServoCommand, HomeCommand
from std_msgs.msg import String

class MotorCmdHandler(Node):
    # Node contructor
    def __init__(self):
        super().__init__("MotorCmdHandler")

        self.uart_cmd_ = String()

        # Node subscripters and publishers
        self.gantrySub_ = self.create_subscription(GantryCommand, 'move_gantry', self.gantryMovementCallback, 10)
        self.homeSub_ = self.create_subscription(HomeCommand, 'home_handler', self.homeHandlerCallback, 10)
        self.servoSub_ = self.create_subscription(ServoCommand, 'move_servo', self.servoMovementCallback, 10)
        self.uartTxPub_ = self.create_publisher(String, 'uart_transmit', 10)
        
        # Log the initialization
        self.get_logger().info("Motor Command Handler Initialized..")

    # Function handling the axis actuation on the gantry
    def gantryMovementCallback(self, moveCommand = GantryCommand):
        # Getting the movement mode
        mode = moveCommand.mode
        # Getting the target location and the speed
        x, y, z = moveCommand.x, moveCommand.y, moveCommand.z
        max_x, max_y, max_z = moveCommand.a, moveCommand.b, moveCommand.c

        # Form the GCode command for the "move at location" action
        self.uart_cmd_.data = ("G01 " if mode else "G00 ") + "X" + str(x) + " Y" + str(y) + " Z" + str(z) \
                            + ("" if mode else (" A" + str(max_x) + " B" + str(max_y) + " C" + str(max_z)))

        self.uartTxPub_.publish(self.uart_cmd_)

        self.get_logger().info(self.uart_cmd_.data)

    # Function Handling the homing and calibration commands
    # TODO: Improve function so in the case False, True, True, True all
    #       the home commands are sent
    def homeHandlerCallback(self, homingCommand = HomeCommand):
        if homingCommand.go_home:            # Home all axis
            self.uart_cmd_.data = "G28"
        elif homingCommand.current_pos_home: # Set the current position as home
            self.uart_cmd_.data = "F84 " + "X" + ("1 " if homingCommand.x else "0 ")\
                                         + "Y" + ("1 " if homingCommand.y else "0 ")\
                                         + "Z" + ("1" if homingCommand.z else "0")
        else:
            if not homingCommand.x and not homingCommand.y and not homingCommand.z:
                self.get_logger().error("No axis selected for homing/calibration!")
            else:
                if homingCommand.x:              # Find home or calibrate x axis
                    self.uart_cmd_.data = "F11" if not homingCommand.calib else "F14"
                if homingCommand.y:            # Find home or calibrate y axis
                    self.uart_cmd_.data = "F12" if not homingCommand.calib else "F15"
                if homingCommand.z:            # Find home or calibrate z axis
                    self.uart_cmd_.data = "F13" if not homingCommand.calib else "F16"

        
        self.uartTxPub_.publish(self.uart_cmd_)
        self.get_logger().info(self.uart_cmd_.data)

    # Servo controller function. Note that servos can be attached only on pins
    # 4, 5, 6 and 11 on the Farmduino
    def servoMovementCallback(self, servoCommand = ServoCommand):
        # The valid servo pins
        valid_servo_pins = (4, 5, 6, 11)
        
        # If the servo pin is within the valid pins:
        if (servoCommand.pin in valid_servo_pins):
            # Request servo attached to PIN to be rotated to ANGLE
            self.uart_cmd_.data = "F61 P" + str(servoCommand.pin) + " V" + str(servoCommand.ang)

            self.uartTxPub_.publish(self.uart_cmd_)
            self.get_logger().info(self.uart_cmd_.data)
        else:
            self.get_logger().error(str(servoCommand.pin) + " is not within the valid servo pins(4, 5, 6, 11)!")

# Main Function called on the initialization of the ROS2 Node
def main(args = None):
    rclpy.init(args = args)

    motorCmdHandlerNode = MotorCmdHandler()
    
    try:
        rclpy.spin(motorCmdHandlerNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()