"""
Module that converts ROS 2 service and action requests into FarmBot FCode commands.

This module provides encoders used to translate high-level commands received
through services or actions into the FCode format understood by the FarmBot.
"""
from farmbot_hardware_comm.modules.exceptions import EncodeError, YAMLError
from farmbot_hardware_comm.modules.yaml_handler import YAMLHandler

from farmbot_interfaces.action import HomeAxes, MoveGantry
from farmbot_interfaces.srv import (ConfigurePin, MoveServo, ReadI2C, ReadParameter, ReadPin,
                                    SetI2C, Watering, WriteParameter, WritePin)


class Encoder:
    """Encode incoming FarmBot service/action requests into FCode commands."""

    def __init__(self, config_path: str):
        """Initialise the encoder class."""
        self.directory = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')
        self.cmd_validation = YAMLHandler.load_yaml(self.directory, 'CommandValidation.yaml')

        try:
            self.active_map = YAMLHandler.load_yaml(config_path, 'active_map.yaml')
        except YAMLError as e:
            self.get_logger().warn(f'yaml error: {e}')
            return

    # Water commands
    def encode_watering(self, req: Watering.Request) -> str:
        """
        Encode a watering command into a FarmBot FCode instruction.

        Converts a watering service request into the corresponding FCode command.
        The watering mode can be time-based or flow-meter-based depending on the
        selected command type.

        Args:
            req {Watering.Request}: Watering command request containing the
                watering mode and dose value.
        """
        valid_watering_commands = self.cmd_validation['valid_watering_commands']

        if req.command in valid_watering_commands:
            if req.dose <= 0:
                raise EncodeError('The time constraint/volume constraint was not set!')
            return f'F0{req.command} {"T" if req.command == 1 else "N"}{req.dose}'

        raise EncodeError(f'command {req.command} is not a valid watering command type '
                          f'{valid_watering_commands}. First element should be 1 '
                          '(timed pulses msec) or 2 (volume pulses)!')

    # I2C commands (Not implemented on the Farmduino yet)
    def encode_read_i2c(self, req: ReadI2C.Request) -> str:
        """
        Encode a ReadI2C request into a FarmBot FCode command.

        Converts an I2C read request into the corresponding FCode instruction used
        to retrieve data from an I2C device connected to the Farmduino.

        Args:
        req {ReadI2C.Request}: I2C read request containing the device element
            identifier and parameter to read.
        """
        valid_i2c_elements = self.cmd_validation['valid_i2c_elements']

        if req.element not in valid_i2c_elements:
            raise EncodeError(f'element {req.element} is not a valid i2c element '
                              f'{valid_i2c_elements}')
        return f'F52 E{req.element} P{req.parameter}'

    def encode_set_i2c(self, req: SetI2C.Request) -> str:
        """
        Encode a SetI2C request into a FarmBot FCode command.

        Converts an I2C write request into the corresponding FCode instruction used
        to send data to an I2C device connected to the Farmduino.

        Args:
            req {SetI2C.Request}: I2C write request containing the device element,
            parameter, and value to send.
        """
        valid_i2c_elements = self.cmd_validation['valid_i2c_elements']

        if req.element not in valid_i2c_elements:
            raise EncodeError(f'element {req.element} is not a valid i2c element '
                              f'{valid_i2c_elements}')
        return f'F51 E{req.element} P{req.parameter} V{req.value}'

    # Pin commands
    def encode_configure_pin(self, req: ConfigurePin.Request) -> str:
        """
        Encode a ConfigurePin request into a FarmBot FCode command.

        Converts a pin configuration request into the corresponding FCode instruction
        used to change the input/output mode of a pin on the Farmduino.

        Args:
            req {ConfigurePin.Request}: Pin configuration request containing the pin
                identifier and the desired input/output mode.
        """
        valid_pins = (i for i in range(self.cmd_validation['valid_pin_bounds'][0],
                                       self.cmd_validation['valid_pin_bounds'][1]))

        if req.pin not in valid_pins:
            raise EncodeError(f'pin {req.pin} is not a configurable pin')
        return f'F43 P{req.pin} M{int(req.output)}'

    def encode_read_pin(self, req: ReadPin.Request) -> str:
        """
        Encode a ReadPin request into a FarmBot FCode command.

        Converts a pin read request into the corresponding FCode instruction used
        to read the digital or analog value of a pin on the Farmduino.

        Args:
            req {ReadPin.Request}: Pin read request containing the pin identifier
            and the reading mode (digital or analog).
        """
        valid_pins = (i for i in range(self.cmd_validation['valid_pin_bounds'][0],
                                       self.cmd_validation['valid_pin_bounds'][1]))

        if req.pin not in valid_pins:
            raise EncodeError(f'pin {req.pin} is not a readable pin')
        return f'F42 P{req.pin} M{int(req.mode)}'

    def encode_write_pin(self, req: WritePin.Request) -> str:
        """
        Encode a WritePin request into a FarmBot FCode command.

        Converts a pin write request into the corresponding FCode instruction used to change the
        value of a pin on the Farmduino.

        Args:
            req {WritePin.Request}: Pin write request containing the pin identifier,
                                    different values to set, and an optional delay
                                    between two settings.
        """
        valid_pins = (i for i in range(self.cmd_validation['valid_pin_bounds'][0],
                                       self.cmd_validation['valid_pin_bounds'][1]))

        if req.pin not in valid_pins:
            raise EncodeError(f'pin {req.pin} is not a writable pin')
        if req.pulse:
            return f'F44 P{req.pin} V{req.value} W{req.value2} T{req.delay_ms} M{int(req.mode)}'

        return f'F41 P{req.pin} V{req.value} M{int(req.mode)}'

    # Gantry commands
    def encode_move_gantry(self, req: MoveGantry.Goal, params: dict) -> str:
        """
        Encode a MoveGantry request into a FarmBot FCode command.

        Converts a move gantry request into the corresponding FCode instruction
        used to move the gantry to a target position at the requested speed.
        Note that homing and calibration must be done through the
        HomeAxes action

        Args:
            req {MoveGantry.Goal}: Gantry movement request containing the target
                                   position, movement type, and axis speed percentages.
            params {dict}: Parameter table currently loaded on the Farmduino.
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
                f'A{req.speed_percent_x / 100 * params[71]:.0f} '
                f'B{req.speed_percent_y / 100 * params[72]:.0f} '
                f'C{req.speed_percent_z / 100 * params[73]:.0f}')

    # Home commands
    def encode_home_axes(self, req: HomeAxes.Goal) -> str:
        """
        Encode a HomeAxes request into a FarmBot FCode command.

        Converts a home axes request into the corresponding FCode
        instruction used to run homing or calibration on the selected axes
        for the robot.

        Args:
            req {HomeAxes.Goal}: Homing request containing the operation type
            and the selected axis.
        """
        valid_homing_commands = self.cmd_validation['valid_homing_commands']

        if req.op in valid_homing_commands:
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
                          f'{valid_homing_commands}.')

    # Servo commands
    def encode_move_servo(self, req: MoveServo.Request) -> str:
        """
        Encode a MoveServo request into a FarmBot FCode command.

        Converts a MoveServo request into the corresponding FCode instruction
        used to set the angle of a servo connected to a valid Farmduino pin.

        Args:
            req {MoveServo.Request}: Servo movement request containing the servo
                                  pin and the target angle.
        """
        valid_servo_pins = self.cmd_validation['valid_servo_pins']

        if req.pin not in valid_servo_pins:
            raise EncodeError(f'pin {req.pin} is not a valid servo pin {valid_servo_pins}')
        return f'F61 P{req.pin} V{req.angle:.0f}'

    # Parameter commands
    def encode_read_parameter(self, req: ReadParameter.Request, params: dict) -> str:
        """
        Encode a ReadParameter request into a FarmBot FCode command.

        Converts an read parameter request into the corresponding FCode instruction
        used to read the parameters from the Farmduino.

        Args:
            req {ReadParameter.Request}: parameter read request containing the parameter to read.
            params {dict}: Parameter table currently loaded on the Farmduino.
        """
        valid_parameters = list(params.keys())        # TODO: Checks Parameters values

        if req.param not in valid_parameters:
            raise EncodeError(f'parameter {req.param} is not a valid parameter')
        return f'F21 P{req.param}'

    def encode_write_parameter(self, req: WriteParameter.Request, params: dict) -> str:
        """
        Encode a WriteParameter request into a FCode command.

        Converts a write parameter request into the corresponding FCode instruction
        used to write a value to a Farmduino parameter.

        Args:
            req {WriteParameter.Request}: parameter write request containing the parameter
                                        identifier and its new value.
            params {dict}: Parameter table currently loaded on the Farmduino.
        """
        valid_parameters = list(params.keys())

        if req.param not in valid_parameters:
            raise EncodeError(f'parameter {req.param} is not a valid parameter')
        if req.during_calibration:
            return f'F23 P{req.param} V{req.value}'
        return f'F22 P{req.param} V{req.value}'
