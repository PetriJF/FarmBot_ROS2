#!/usr/bin/env python3
"""
Farmbot controller ROS2 node.

Defines the FarmbotControl node and initializes movement, state,
device, tool sequencer, and parameter modules.
"""
# Modules
from farmbot_controllers.devices import DeviceControl
from farmbot_controllers.movement import Movement
from farmbot_controllers.parameters import Parameters
from farmbot_controllers.states import State

import rclpy
from rclpy.node import Node

from farmbot_interfaces.srv import ParameterConfig

from std_msgs.msg import String


class TestClient(Node):
    """ROS2 node for FarmBot control, coordinating movement, devices, and tools."""

    # Node contructor
    def __init__(self):
        """Initialize the FarmbotControl node and its modules."""
        super().__init__('TestClient')

        # Initializing movemement module
        self.mvm = Movement(self)
        # Initializing the state module
        self.state = State(self)
        # Initializing the devices and peripherals modules
        self.devices = DeviceControl(self)
        # Initializing the parameter manipulator
        self.params = Parameters(self)

        # Memory
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        # Temporary Keyboard subscriber
        self.cur_increment = 10.0
        self.input_sub = self.create_subscription(String,
                                                  'input_topic', self.cmd_interp_callback, 10)

        # Log the initialization
        self.get_logger().info('Test Farmbot Controller Initialized..')

    def cmd_interp_callback(self, cmd: String):
        """Parse the incoming command string and dispatch the corresponding FarmBot action."""
        code = cmd.data.split(' ')
        match code[0]:
            # STATE
            case 'E':
                self.state.estop()
            case 'R':
                self.state.reset_estop()
            case '@':
                self.state.abort_movement()
            case 'SW':
                self.state.request_sw_version()
            case 'CURR_POS':
                self.state.request_curr_pos()
            case 'END_STOP':
                self.state.request_end_stop()

            # MOVEMENT
            case 'M':
                if len(code) != 4:
                    self.get_logger().warning('You need to include all 3 coordinates! '
                                              'Command ignored!')
                else:
                    self.mvm.move_gantry_abs(x_coord=float(code[1]),
                                             y_coord=float(code[2]),
                                             z_coord=float(code[3]))
            case 'M_S':
                if len(code) != 5:
                    self.get_logger().warning('You need to include all 3 coordinates and a speed '
                                              'percentage! Command ignored!')
                else:
                    self.mvm.move_gantry_s(x_coord=float(code[1]), y_coord=float(code[2]),
                                           z_coord=float(code[3]), speed=float(code[4]))
            case 'HOME':
                self.mvm.set_curr_to_home(x=True if code[1] == 'X' else False,
                                          y=True if code[1] == 'Y' else False,
                                          z=True if code[1] == 'Z' else False)
            case 'H_0':
                self.mvm.go_home()
            case 'FIND_AXIS':
                self.mvm.find_axis_home(x=True if code[1] == 'X' else False,
                                        y=True if code[1] == 'Y' else False,
                                        z=True if code[1] == 'Z' else False)
            case 'CALIBRATE':
                self.mvm.calibrate_axis(x=True if code[1] == 'X' else False,
                                        y=True if code[1] == 'Y' else False,
                                        z=True if code[1] == 'Z' else False)
            # Parameter configuration commands
            case 'CONF':
                if len(code) == 1:
                    self.param_config_client(cmd='SAVE')
                    self.param_config_client(cmd='MAP')
                else:
                    if code[1] == 'S':
                        self.param_config_client(cmd='SAVE')
                    elif code[1] == 'M':
                        self.param_config_client(cmd='MAP')

            # DEVICE
            case 'I2C_READ':
                self.devices.i2c_read(1, 2)
            case 'I2C_SET':
                self.devices.i2c_set(1, 2, 3)
            case 'W':
                self.devices.water_command(1, 20.0)
            case 'D_C':
                self.devices.read_pin(63, False)
            case 'D_set':
                self.devices.set_pin_value(63, int(float(code[1])), False, False)
            case 'D_configure':
                self.devices.set_pin_io(63, False)
            case 'M_SV':
                self.get_logger().info(f'Trying to move servo {int(code[1])} to {int(code[2])}')
                self.devices.move_servo(pin=int(code[1]), angle=float(code[2]))

            # PARAMETER
            case 'READ_PARAM':
                self.params.read_param(int(float(code[1])))
            case 'LIST_ALL':
                self.params.list_all_params()
            case 'C_2':
                if len(code) == 1:
                    self.get_logger().warning('You have not selected the axis encoder you want to '
                                              'flip. Command ignored')
                else:
                    if code[1] in ['X', 'Y', 'Z']:
                        param = 130 + ((1 if code[1] == 'X' else 0) +
                                       (2 if code[1] == 'Y' else 0) +
                                       (3 if code[1] == 'Z' else 0))
                        self.params.write_param(param, int(float(code[1])), False)
                    else:
                        self.get_logger().warning('C_2: Invalid option selected. Choose: X, Y, Z')
            case 'UPDATE_PARAM':
                self.params.writeParam(code[1], code[2], True)

    def param_config_client(self, cmd: String):
        """
        Parameter Configuration Client.

        Requests a response from the Parameter Manager Server
        """
        client = self.create_client(ParameterConfig, 'manage_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Parameter Config Server...')

        request = ParameterConfig.Request()
        request.data = cmd

def main(args=None):
    """Initialize ROS2 and run the Test FarmbotControl node until shutdown."""
    rclpy.init(args=args)

    main_ctrl_node = TestClient()

    try:
        rclpy.spin(main_ctrl_node)
    except (KeyboardInterrupt, Exception) as e:
        main_ctrl_node.get_logger().fatal(f'{e}')
        main_ctrl_node.destroy_node()

    main_ctrl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
