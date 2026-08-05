#!/usr/bin/env python3
"""
Keyboard teleoperation node for ROS2 Farmbot.

Publishes keyboard commands to the farmbot controller for execution and
demonstrates command priority handling versus sequencer commands.
"""
from farmbot_controllers.farmbot_controllers.sequences.calibration import calibrate_axes
from farmbot_controllers.farmbot_controllers.sequences.find_home import find_home
from farmbot_controllers.farmbot_controllers.sequences.single_call import single_call

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger


class KeyboardTeleOp(Node):
    """
    ROS2 node that publishes keyboard commands for the Farmbot controller.

    Node used for recording keyboard commands and sending them forward to the
    farmbot controller for interpretation and execution.
    """

    # Node contructor
    def __init__(self):
        """Initialize the KeyboardTeleOp node and its ROS2 publishers."""
        super().__init__('KeyboardController')

        # Keyboard publisher
        self.cmd = String()
        self.input_pub = self.create_publisher(String, 'input_topic', 10)
        self.priority_pub = self.create_publisher(String, 'farmbot_command', 10)

        # Initialisation of the service clients for priority commands (estop, abort, resume)
        self.estop_client = self.node.create_client(Trigger, 'estop')
        self.abort_client = self.node.create_client(Trigger, 'abort')
        self.resume_client = self.node.create_client(Trigger, 'resume')

        # Farmbot Position Subscriber
        self.fb_position_sub = self.create_subscription(PointStamped, 'farmbot_position',
                                                        self.fb_position_callback, 10)
        # Memory
        self.cur_increment = 10.0
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        # Log the initialization
        self.get_logger().info('Keyboard Controller Initialized..')

        self.get_logger().info("""\n
                               This is a keyboard based controller for the ROS2 Farmbot
                               Controllers. The commands accepted can be found in the
                               Documentation under High Level Commands.\n
                               NOTE: The commands here DO NOT automatically enter the
                               sequencer as they hold execution priority! An exception
                               is shown for sequencing commands such as 'P_4' for watering.""")

    def _send(self, client, on_done=None):
        """Check the server, send a Trigger request and complete via _complete."""
        if client.service_is_ready():
            client.call_async(Trigger.Request()).add_done_callback(
                              lambda future: self._complete(future, on_done))

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report, never leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not response.success:
            self.node.get_logger().warn(f'Command failed: {response.message}')
        elif response.message:
            self.node.get_logger().info(response.message)
        else:
            self.node.get_logger().info('Command successful')
        if on_done is not None:
            on_done(response)

    def fb_position_callback(self, position: PointStamped):
        """
        Update the current FarmBot position from an incoming position message.

        The current x, y, and z coordinates are extracted from the received
        PointStamped message and stored as the robot's current position.

        Args:
            position {PointStamped}: message containing the current position.
        """
        self.cur_x = position.point.x
        self.cur_y = position.point.y
        self.cur_z = position.point.z

    def check_input(self):
        """
        Check the input stream for valid commands and sends them on.

        Commands can be simple keys (e.g. T_1_1) or compound commands (e.g. P_1), where the
        key is followed by more information that is to be used by the farmbot
        """
        # Record the user input
        user_input = input('\nEnter command: ')
        code = user_input.split(' ')

        match code[0]:
            case 'E':
                self._send(self.estop_client)
                self.get_logger().info('ESTOP button pressed')
            case 'R':
                self._send(self.resume_client)
                self.get_logger().info('RESET button pressed')
            case '@':
                self._send(self.abort_client)
                self.get_logger().info('Abort movement')
            case 'SW_VER':
                single_call('software version', lambda hw,
                            cb: hw.states.request_sw_version(on_done=cb))
            case 'M':
                if len(code) != 4:
                    self.get_logger().warning('You need to include all 3 coordinates! '
                                              'Command ignored!')
                else:
                    x = float(code[1])
                    y = float(code[2])
                    z = float(code[3])
                    single_call('move', lambda hw,
                                cb: hw.movement.move_gantry_abs(x, y, z, on_done=cb))
            case 'M_S':
                if len(code) != 5:
                    self.get_logger().warning('You need to include all 3 coordinates and a '
                                              'speed percentage! Command ignored!')
                else:
                    x = float(code[1])
                    y = float(code[2])
                    z = float(code[3])
                    speed = float(code[4])
                    single_call('move_speed', lambda hw,
                                cb: hw.movement.move_gantry_s(x, y, z, speed, on_done=cb))
            case 'w' | 's':
                x = self.cur_x + (self.cur_increment * (-1 if code[0] == 's' else 1))
                single_call('x_axis_increment', lambda hw,
                            cb: hw.movement.move_gantry_abs(x, self.cur_y, self.cur_z, on_done=cb))
            case 'a' | 'd':
                y = self.cur_y + (self.cur_increment * (-1 if code[0] == 'a' else 1))
                single_call('y_axis_increment', lambda hw,
                            cb: hw.movement.move_gantry_abs(self.cur_x, y, self.cur_z, on_done=cb))
            case '1':
                self.cur_increment = 10.0
            case '2':
                self.cur_increment = 100.0
            case '3':
                self.cur_increment = 500.0
            case 'H_0':
                single_call('go_home', lambda hw, cb: hw.movement.go_home(on_done=cb))
            case 'H_1':
                find_home()
            case 'H_2':
                if len(code) != 2:
                    self.get_logger().warning('You need to specify the axes on which you want to '
                                              'determine the origin position! '
                                              'Command ignored!')
                find_home(x=True if code[1] == 'X' else False,
                          y=True if code[1] == 'Y' else False,
                          z=True if code[1] == 'Z' else False)

            # TODO: case 'CONF':

            case 'C_0':  # C_0 for calib. all axis, C_0 X for calib. x axis and so on
                if len(code) == 1:
                    calibrate_axes()
                else:
                    calibrate_axes(x=True if code[1] == 'X' else False,
                                   y=True if code[1] == 'Y' else False,
                                   z=True if code[1] == 'Z' else False)

            # TODO: case 'C_1':
            # TODO: case 'C_2':
            # TODO: case 'T_1_0' | 'T_2_0' | 'T_3_0':
            # TODO: case 'T_1_1' | 'T_1_2' | 'T_2_1' | 'T_2_2' | 'T_3_1' | 'T_3_2':
            # TODO: case 'P_1':
            # TODO: case 'P_2':
            # TODO: case 'P_3' | 'P_4' | 'P_5' | 'P_9':
            # TODO: case 'S_1_0' | 'S_2_0' | 'S_3_0':
            # TODO: case 'I_0':
            # TODO: case 'I_1':
            # TODO: case 'I_2':
            # TODO: case 'I_3':
            # TODO: case 'I_4':

            case 'D_L_1' | 'D_L_0':
                light_pin = 7
                single_call('led_strip', lambda hw,
                            cb: hw.devices.set_pin_value(light_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_V_0' | 'D_V_1':
                vacuum_pin = 9
                single_call('vacuum_pump', lambda hw,
                            cb: hw.devices.set_pin_value(vacuum_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_W_0' | 'D_W_1':
                water_pin = 8
                single_call('water_pump', lambda hw,
                            cb: hw.devices.set_pin_value(water_pin, int(code[0][4]),
                                                         False, False, on_done=cb))
            case 'D_WP':
                water_pin = 8
                single_call('water_pulses', lambda hw,
                            cb: hw.devices.set_pin_value(water_pin, value=1, pin_mode=False,
                                                         pulse=True, value2=0, delay_ms=500,
                                                         on_done=cb))
            case 'D_C':
                single_call('check_tool_mounting', lambda hw,
                            cb: hw.devices.read_pin(63, False, on_done=cb))
            case 'D_S_C':
                single_call('soil_sensor_reading', lambda hw,
                            cb: hw.devices.read_pin(59, True, on_done=cb))
            case 'P4_0' | 'P4_1':
                peripheral4_pin = 10
                single_call('peripheral4', lambda hw,
                            cb: hw.devices.set_pin_value(peripheral4_pin, int(code[0][3]),
                                                         False, False, on_done=cb))
            case 'P5_0' | 'P5_1':
                peripheral5_pin = 12
                single_call('peripheral5', lambda hw,
                            cb: hw.devices.set_pin_value(peripheral5_pin, int(code[0][3]),
                                                         False, False, on_done=cb))
            case 'M_SV':
                if len(code) != 3:
                    self.get_logger().warning('You need to include the servo pin and angle! '
                                              'Command ignored!')
                else:
                    self.get_logger().info(f'Trying to move servo {int(code[1])} to {int(code[2])}')
                    single_call('move_servo', lambda hw,
                                cb: hw.devices.move_servo(pin=int(code[1]), angle=float(code[2]),
                                                          on_done=cb))
            case _:
                print('Invalid input\n')


def main(args=None):
    """Initialize and run the keyboard teleoperation node."""
    rclpy.init(args=args)

    keyboard_node = KeyboardTeleOp()

    try:
        while rclpy.ok():
            keyboard_node.check_input()
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
