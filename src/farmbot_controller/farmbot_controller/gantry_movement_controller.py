#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand


class GantryController(Node):
    def __init__(self):
        super().__init__("GantryController")
        self.get_logger().info("Gantry Controller Initialized..")

        self.gantrySub_ = self.create_subscription(GantryCommand, 'move_gantry', self.gantryMovementCallback, 10)

    def gantryMovementCallback(self, moveCommand = GantryCommand):
        x, y, z = moveCommand.x, moveCommand.y, moveCommand.z
        max_x, max_y, max_z = moveCommand.max_speed_x, moveCommand.max_speed_y, moveCommand.max_speed_z

        self.get_logger().info(str(x) + "---" + str(y) + "---" + str(z) + "---" + str(max_x) + "---" + str(max_y) + "---" + str(max_z))



def main(args = None):
    rclpy.init(args = args)

    gantryControlNode = GantryController()
    
    try:
        rclpy.spin(gantryControlNode)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()