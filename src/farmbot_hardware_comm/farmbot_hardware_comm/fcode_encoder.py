"""
Module that converts high-level commands into FCode commands that Farmbot can interpret.

It includes the Device Command Handler, Motor Command Handler, and State Command Handler modules.
"""
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_interfaces.action import HomeAxes, MoveGantry
from farmbot_interfaces.srv import (ConfigurePin, MoveServo, ReadI2C, ReadParameter, ReadPin,
                                    SetI2C, Watering, WriteParameter, WritePin)

import yaml


class EncodeError(Exception):
    """Raised when a request cannot be encoded into a valid FCode command."""

    pass


class Encoder:
    """Encode incoming FarmBot service/action requests into FCode commands."""

    def __init__(self, config_path):
        """Initialize the encoder class."""
        self.active_config = yaml.safe_load(open(os.path.join(config_path,
                                                              'activeConfig.yaml'), 'r'))
        self.active_map = yaml.safe_load(open(os.path.join(config_path,
                                                           'active_map.yaml'), 'r'))

        self.directory = os.path.join(
            get_package_share_directory('farmbot_hardware_comm'),
            'config'
        )
        self.cmd_validation = yaml.safe_load(open(os.path.join(self.directory,
                                                               'CommandValidation.yaml'), 'r'))

    # Water commands
    def encode_watering(self, req: Watering.Request) -> str:
        """
        Watering style command.

        Used to set the watering to be either time based (1) or
        measured using a flow meter (2)
        """
        WATERING_VALID_COMMAND = self.cmd_validation['watering_valid_command']

        if req.command in WATERING_VALID_COMMAND:
            if req.dose <= 0:
                raise EncodeError('The time constraint/volume constraint was not set!')
            return f'F0{req.command} {"T" if req.command == 1 else "N"}{req.dose}'

        raise EncodeError(f'command {req.command} is not a valid watering command type '
                          f'{WATERING_VALID_COMMAND}. First element should be 1 '
                          '(timed pulses msec) or 2 (volume pulses)!')

    # I2C commands (Not implemented on the Farmduino yet)
    def encode_read_i2c(self, req: ReadI2C.Request) -> str:
        """
        Encode the ReadI2C request into a FCode command.

        Used to read from the I2C devices connected to the farmduino.
        """
        VALID_I2C_ELEMENTS = self.cmd_validation['valid_i2c_element']

        if req.element not in VALID_I2C_ELEMENTS:
            raise EncodeError(f'element {req.element} is not a valid i2c element '
                              f'{VALID_I2C_ELEMENTS}')
        return f'F52 E{req.element} P{req.parameter}'

    def encode_set_i2c(self, req: SetI2C.Request) -> str:
        """
        Encode the SetI2C request into a FCode command.

        Used to write to the I2C devices connected to the farmduino.
        """
        VALID_I2C_ELEMENTS = self.cmd_validation['valid_i2c_element']

        if req.element not in VALID_I2C_ELEMENTS:
            raise EncodeError(f'element {req.element} is not a valid i2c element '
                              f'{VALID_I2C_ELEMENTS}')
        return f'F51 E{req.element} P{req.parameter} V{req.value}'

    # Pin commands
    def encode_configure_pin(self, req: ConfigurePin.Request) -> str:
        """
        Encode the ConfigurePin request into a FCode command.

        Used to change the input/output mode of a pin on the Farmduino.
        """
        VALID_PINS = (i for i in range(self.cmd_validation['valid_pins_bounds'][0],
                                       self.cmd_validation['valid_pins_bounds'][1]))

        if req.pin not in VALID_PINS:
            raise EncodeError(f'pin {req.pin} is not a configurable pin')
        return f'F43 P{req.pin} M{int(req.output)}'

    def encode_read_pin(self, req: ReadPin.Request) -> str:
        """
        Encode the ReadPin request into a FCode command.

        Used to change the digital/analog mode of a pin on the Farmduino.
        """
        VALID_PINS = (i for i in range(self.cmd_validation['valid_pins_bounds'][0],
                                       self.cmd_validation['valid_pins_bounds'][1]))

        if req.pin not in VALID_PINS:
            raise EncodeError(f'pin {req.pin} is not a readable pin')
        return f'F42 P{req.pin} M{int(req.mode)}'

    def encode_write_pin(self, req: WritePin.Request) -> str:
        """
        Encode the WritePin request into a FCode command.

        Used to change the value of a pin on the Farmduino.
        """
        VALID_PINS = (i for i in range(self.cmd_validation['valid_pins_bounds'][0],
                                       self.cmd_validation['valid_pins_bounds'][1]))

        if req.pin not in VALID_PINS:
            raise EncodeError(f'pin {req.pin} is not a writable pin')
        if req.pulse:
            return f'F44 P{req.pin} V{req.value} W{req.value2} T{req.delay_ms} M{int(req.mode)}'

        return f'F41 P{req.pin} V{req.value} M{int(req.mode)}'

    # Gantry commands
    def encode_move_gantry(self, req: MoveGantry.Goal) -> str:
        """
        Handle gantry commands.

        Note that homing and calibration must be done through the
        HomeAxes action
        """
        x_edge = self.active_map['map_reference']['x_len']
        y_edge = self.active_map['map_reference']['y_len']
        # z_edge = self.active_map['map_reference']['z_len']

        # edges of 0 mean "not calibrated yet" -> no bounds check
        if x_edge > 0 and y_edge > 0:
            if (req.target.x > x_edge or req.target.y > y_edge
               or req.target.x < 0 or req.target.y < 0):
                raise EncodeError(f"Point {req.target} is outside the greenhouse's boundaries"
                                  f'({x_edge}, {y_edge})')

        if req.interpolated:
            return f'G01 X{req.target.x} Y{req.target.y} Z{req.target.z}'
        return (f'G00 X{req.target.x} Y{req.target.y} Z{req.target.z} '
                f'A{req.speed_percent_x / 100 * self.active_config[71]:.0f} '
                f'B{req.speed_percent_y / 100 * self.active_config[72]:.0f} '
                f'C{req.speed_percent_z / 100 * self.active_config[73]:.0f}')

    # Home commands
    def encode_home_axes(self, req: HomeAxes.Goal) -> str:
        """Handle homing and calibration commands."""
        HOMING_VALID_COMMAND = self.cmd_validation['homing_valid_command']

        if req.op in HOMING_VALID_COMMAND:
            if req.op == 0:
                return 'G28'
            elif req.op in [1, 2]:
                if not req.x and not req.y and not req.z:
                    raise EncodeError('No axis selected for homing/calibration!')
                elif [req.x, req.y, req.z].count(True) > 1:
                    raise EncodeError('Select a single axis for homing/calibration!')
                else:
                    if req.x:                            # Find home or calibrate x axis
                        return 'F11' if req.op == 1 else 'F14'
                    if req.y:                            # Find home or calibrate y axis
                        return 'F12' if req.op == 1 else 'F15'
                    if req.z:                            # Find home or calibrate z axis
                        return 'F13' if req.op == 1 else 'F16'
            return f'F84 X{int(req.x)} Y{int(req.y)} Z{int(req.z)}'  # Set Home

        raise EncodeError(f'command {req.op} is not a valid homing command type '
                          f'{HOMING_VALID_COMMAND}.')

    # Servo commands
    def encode_move_servo(self, req: MoveServo.Request) -> str:
        """
        Handle servo command interpretation.

        Note that servos can be attached only on pins
        4, 5, 6 and 11 on the Farmduino
        """
        VALID_SERVO_PINS = self.cmd_validation['valid_servo_pins']

        if req.pin not in VALID_SERVO_PINS:
            raise EncodeError(f'pin {req.pin} is not a valid servo pin {VALID_SERVO_PINS}')
        return f'F61 P{req.pin} V{req.angle:.0f}'

    # Parameter commands
    def encode_read_parameter(self, req: ReadParameter.Request) -> str:
        """
        Encode the ReadParameter request into a FCode command.

        Used to read the parameters from the Farmduino.
        """
        VALID_PARAMETERS = list(self.active_config.keys())          # TODO: Checks Parameters values

        if req.param not in VALID_PARAMETERS:
            raise EncodeError(f'parameter {req.param} is not a valid parameter')
        return f'F21 P{req.param}'

    def encode_write_parameter(self, req: WriteParameter.Request) -> str:
        """
        Encode the WriteParameter request into a FCode command.

        Used to write to the parameters on the Farmduino.
        """
        VALID_PARAMETERS = list(self.active_config.keys())

        if req.param not in VALID_PARAMETERS:
            raise EncodeError(f'parameter {req.param} is not a valid parameter')
        if req.during_calibration:
            return f'F23 P{req.param} V{req.value}'
        return f'F22 P{req.param} V{req.value}'

    # DEPRECATED
    # def state_cmd(self, command: list):
    #     """
    #     State Command Handler.

    #     Used as the main pipeline for specific commands.
    #     E - End Stop, @ - Abort Movement, F09 - Report Endstop, F81 - Report
    #     Current Position and F83 - Report Software Version
    #     """
    #     state_counter = command.count('True')

    #     # Check that only one start command is in effect
    #     if state_counter != 1:
    #         self.node.get_logger().error(
    #             'Make sure to include only 1 state handler command!'
    #             'Your input has ' + str(state_counter) + ' commands')
    #     else:
    #         if command[0] == 'True':         # Electronic stop
    #             self.uart_cmd = 'E'
    #         elif command[1] == 'True':       # Abort current movement command
    #             self.uart_cmd = '@'
    #         elif command[2] == 'True':       # Reset the Electronic stop
    #             self.uart_cmd = 'F09'
    #         elif command[3] == 'True':       # Report End Stop
    #             self.uart_cmd = 'F81'
    #         elif command[4] == 'True':       # Report Current Position
    #             self.uart_cmd = 'F82'
    #         elif command[5] == 'True':       # Report Software Version
    #             self.uart_cmd = 'F83'

    #         self.node.get_logger().info(self.uart_cmd)
    #         return self.uart_cmd

    # def status_cmd(self, cmd: StatusCommand):
    #     """
    #     Allow the read and write of a status command for the Farmbot.

    #     Check documentation for more information on status commands
    #     """
    #     if cmd.mode:    # Write mode
    #         self.uart_cmd = 'F32 P' + str(cmd.p) + ' V' + str(cmd.v)
    #     else:           # Read mode
    #         self.uart_cmd = 'F31 P' + str(cmd.p)

    #     self.node.get_logger().info(self.uart_cmd)
    #     return self.uart_cmd
