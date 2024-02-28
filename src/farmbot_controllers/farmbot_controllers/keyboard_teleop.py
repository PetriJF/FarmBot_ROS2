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
                     'c', 'v', 'b', 'n', 'o', 'p', 'T_1_0', 'T_1_1', 'T_1_2', 'T_2_0', 'T_2_1', 'T_2_2',
                     'T_3_0', 'T_3_1', 'T_3_2', 
                     'P_3', 'P_4', 'S_1_0', 'S_2_0', 'S_3_0', 'I_0', 'I_1', 'D_C', 'D_L_1', 'D_L_0',
                     'D_W_1', 'D_W_0', 'D_V_1', 'D_V_0')
        compoundCmds = ('P_1')
        userInput = input("\nEnter command: ")

        if userInput in validKeys or userInput.split(' ')[0] in compoundCmds:
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