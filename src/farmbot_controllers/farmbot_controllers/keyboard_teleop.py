#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardTeleOp(Node):
    # Node contructor
    def __init__(self):
        super().__init__("KeyboardController")

        # Temporary keyboard publisher
        self.cmd_ = String()
        self.inputPub_ = self.create_publisher(String, 'keyboard_topic', 10)

        # Log the initialization
        self.get_logger().info("Keyboard Controller Initialized..")

    def checkKeyPress(self):
        validKeys = ('e', 'E', 'w', 'a', 's', 'd', '1', '2', '3', 'i', 'h', 'j', 'k', 'l', 'fh',
                     'c', 'v', 'b', 'n', 'o', 'p', 'T10', 'T11', 'T12', 'T20', 'T21', 'T22',
                     'T30', 'T31', 'T32')
        userInput = input("\nEnter command: ")

        if userInput in validKeys:
            self.cmd_.data = userInput
            self.inputPub_.publish(self.cmd_)
        else:
            print("Invalid input")

def main(args = None):
    rclpy.init(args = args)

    keyboardTeleOpNode = KeyboardTeleOp()
    
    try:
        #rclpy.spin(keyboardTeleOpNode)
        while rclpy.ok():
            keyboardTeleOpNode.checkKeyPress()
    except KeyboardInterrupt:
        pass
    finally:
        keyboardTeleOpNode.destroy_node
        rclpy.shutdown()


if __name__ == '__main__':
    main()