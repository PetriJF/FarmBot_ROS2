"""
Autonomous command sender module.

Publishes scheduled autonomous commands from AutonomousCommand.yaml to /input_topic
at configured times.
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import yaml


class AutonomousCmds(Node):
    """
    Publish scheduled autonomous commands.

    Reads commands from AutonomousCommand.yaml and publishes them to /input_topic
    at the configured time.
    """

    def __init__(self):
        """Initialize the autonomous command sender node."""
        super().__init__('command_sender')
        self.publisher = self.create_publisher(String, '/input_topic', 10)
        timer_period = 60  # seconds
        self.timer = self.create_timer(timer_period, self.send_command)

        self.directory_ = os.path.join(
            get_package_share_directory('farmbot_hri'),
            'config'
        )

        self.auto_command = yaml.safe_load(open(os.path.join(self.directory_,
                                                             'AutonomousCommand.yaml'), 'r'))

    def send_command(self):
        """Publish scheduled autonomous commands when their configured time arrives."""
        now = datetime.now().time()
        current_time = now.strftime('%H:%M')

        for task in self.auto_command:
            if current_time == self.auto_command[task]['time']:
                command = self.auto_command[task]['command']
                self.get_logger().info('Publishing: "%s"' % command)
                msg = String()
                msg.data = command
                self.publisher.publish(msg)


def main(args=None):
    """Initialize and run the autonomous command sender node."""
    rclpy.init(args=args)
    command_sender = AutonomousCmds()
    rclpy.spin(command_sender)
    command_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
