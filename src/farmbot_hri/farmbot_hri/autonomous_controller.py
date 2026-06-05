import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class AutonomousCmds(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.publisher = self.create_publisher(String, '/input_topic', 10)
        timer_period = 60  # seconds
        self.timer = self.create_timer(timer_period, self.send_command)

        self.directory_ = os.path.join(
            get_package_share_directory('farmbot_hri'),
            'config'
        )

        self.auto_command = yaml.safe_load(open(os.path.join(self.directory_, 'AutonomousCommand.yaml'), 'r'))

    def send_command(self):
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
    rclpy.init(args=args)
    command_sender = AutonomousCmds()
    rclpy.spin(command_sender)
    command_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()