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
        if current_time == '11:07':
            command = 'P_4'
        elif current_time == '12:00':
            command = 'H_0'
        # elif current_time == '14:57':
        #     for i in range(0, 16):
        #         for j in range(0, 4):
        #             x = 700 + i*250
        #             y = 850 + j*250
        #             command = f'P_1 {x} {y} -290.0 50.0 100.0 1 0 Basil Planning'
        #             self.get_logger().info('Publishing: "%s"' % command)
        #             msg = String()
        #             msg.data = command
        #             self.publisher_.publish(msg)
        #             time.sleep(0.5)

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