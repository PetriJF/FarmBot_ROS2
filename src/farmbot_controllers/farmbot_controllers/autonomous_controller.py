import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime

class AutonomousCmds(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.publisher_ = self.create_publisher(String, '/input_topic', 10)
        timer_period = 60  # seconds
        self.timer = self.create_timer(timer_period, self.send_command)

    def send_command(self):
        now = datetime.now().time()
        current_time = now.strftime('%H:%M')
        command = ''
        if current_time == '11:31':
            command = 'C_0'
        elif current_time == '12:00':
            command = 'CONF'
        elif current_time == '13:01':
            command = 'P_4'
        
        if command:
            self.get_logger().info('Publishing: "%s"' % command)
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    command_sender = AutonomousCmds()
    rclpy.spin(command_sender)
    command_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()