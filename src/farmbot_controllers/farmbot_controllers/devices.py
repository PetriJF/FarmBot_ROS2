#!/usr/bin/env python3
"""
Device control utilities for ROS2 Farmbot.

ROS2 Python Module that enables the control of the different devices
that are connected to the Farmbot
"""
from farmbot_interfaces.srv import (ConfigurePin, MoveServo, ReadI2C, ReadPin,
                                    SetI2C, Watering, WritePin)

from rclpy.node import Node

# from std_msgs.msg import String


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class DeviceControl:
    """Publish device commands (pin, water, I2C, servo) to the 'farmbot_command' topic."""

    def __init__(self, node: Node):
        """Initialize DeviceControl with a ROS2 node and its publishers."""
        self.node = node

        # # Variables used to store the device commands
        # self.pin_cmd = String()
        # self.water_cmd = String()
        # self.i2c_cmd = String()
        # self.servo_cmd = String()

        self.read_i2c_client = self.node.create_client(ReadI2C, 'read_i2c')
        self.set_i2c_client = self.node.create_client(SetI2C, 'set_i2c')
        self.watering_client = self.node.create_client(Watering, 'watering')
        self.read_pin_client = self.node.create_client(ReadPin, 'read_pin')
        self.write_pin_client = self.node.create_client(WritePin, 'write_pin')
        self.configure_pin_client = self.node.create_client(ConfigurePin, 'configure_pin')
        self.move_servo_client = self.node.create_client(MoveServo, 'move_servo')

        # # Publishers for the command types
        # self.devices_pub = self.node.create_publisher(String, 'farmbot_command', 10)

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            raise ServerError(f'{cmd_name} Server not available!')

    # I2C Control Handlers
    def i2c_read(self, element: int, parameter: int):
        """Service client to read from an I2C device."""
        try:
            self._server_availability('ReadI2C', self.read_i2c_client)
        except ServerError:
            raise

        request = ReadI2C.Request()
        request.element = element
        request.parameter = parameter

        future = self.read_i2c_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def i2c_set(self, element: int, parameter: int, value: int):
        """Service client to set a value to an I2C device."""
        try:
            self._server_availability('SetI2C', self.set_i2c_client)
        except ServerError:
            raise

        request = SetI2C.Request()
        request.element = element
        request.parameter = parameter
        request.value = value

        future = self.set_i2c_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    # Water Control Handlers
    # NOTE: The documentation mentions that the commands are not implemented. Need to investigate

    def water_command(self, mode: bool, unit: float):
        """
        Service client to set up the water control commands.

        The water control commands. NOTE: The documentation mentions that the commands
        are not implemented! Did not check if this is true, but added the command handling
        anyway if support will be added to the arduino firmware.

        Args:
            mode {bool}: False (or 1) for time based watering, True (or 2) for valume flow based
                         watering
            unit {int}: The amount of time (in time based watering) in millisec. or the pulse
                        count of the flow meter.
        """
        try:
            self._server_availability('Watering', self.watering_client)
        except ServerError:
            raise

        request = Watering.Request()
        if mode:
            request.command = request.PULSE_DOSING
        else:
            request.command = request.TIMED_DOSING

        request.dose = unit

        future = self.watering_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    # Pin Control Handlers
    def set_pin_value(self, pin: int, value: int, pin_mode: bool,
                      pulse: bool, value2=0, delay_ms=0):
        """
        Service client to set a farmduino pin to a selected value and pin_mode.

        Args:
            pin {int}: The pin the value is set on
            value {int}: Value to set
            pin_mode {bool}: 0 for digital, 1 for analog
        """
        try:
            self._server_availability('WritePin', self.write_pin_client)
        except ServerError:
            raise

        request = WritePin.Request()
        request.pin = pin
        if pin_mode:
            request.mode = WritePin.Request.ANALOG
        else:
            request.mode = WritePin.Request.DIGITAL
        request.value = value
        request.pulse = pulse
        request.value2 = value2
        request.delay_ms = delay_ms

        future = self.write_pin_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def read_pin(self, pin: int, pin_mode: bool):
        """
        Service client to read the value from the selected pin in the parsed pin_mode.

        Args:
            pin {int}: The pin to read the value from
            pin_mode{bool}: 0 for digital, 1 for analog
        """
        try:
            self._server_availability('ReadPin', self.read_pin_client)
        except ServerError:
            raise

        request = ReadPin.Request()
        if pin_mode:
            request.mode = ReadPin.Request.ANALOG
        else:
            request.mode = ReadPin.Request.DIGITAL

        request.pin = pin

        future = self.read_pin_client.call_async(request=request)
        future.add_done_callback(self.read_client_callback)

    def read_client_callback(self, future):
        """Service client callback once the request is send."""
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            elif response.value != -1:
                self.node.get_logger().info(f'The value of the pin is {response.value}')

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    def set_pin_io(self, pin: int, io_mode: bool):
        """
        Service client to set the IO of the selected pin.

        Args:
            pin {int}: the pin to set the IO for
            io_mode(bool): 0 for input, 1 for output
        """
        try:
            self._server_availability('ConfigurePin', self.configure_pin_client)
        except ServerError:
            raise

        request = ConfigurePin.Request()
        request.output = io_mode
        request.pin = pin

        future = self.configure_pin_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def move_servo(self, pin: int, angle: float):
        """Create the move servo command."""
        try:
            self._server_availability('MoveServo', self.move_servo_client)
        except ServerError:
            raise

        request = MoveServo.Request()
        request.pin = pin
        request.angle = angle

        future = self.move_servo_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def client_callback(self, future):
        """Service client callback once the request is send."""
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    # I2C Control Handlers

    # def i2c_read(self, pin: int, element: int):
    #     """Read from an I2C device."""
    #     self.i2c_handler(mode=False, pin=pin, element=element, value=0)

    # def i2c_set(self, pin: int, element: int, value: int):
    #     """Set a value to an I2C device."""
    #     self.i2c_handler(mode=True, pin=pin, element=element, value=value)

    # def i2c_handler(self, mode: bool, element: int, pin: int, value: int):
    #     """
    #     Create the I2C command interface handler.

    #     Args:
    #         mode {bool}: False for READ, True for SET
    #         pin {int}: Pin number
    #         element {int}: Element in tool mount
    #         value {int}: Value number
    #     """
    #     self.i2c_cmd.data = ('i2c_command ' + str(mode) + ' ' + str(element)
    #                          + ' ' + str(pin) + ' ' + str(value))

    #     self.devices_pub.publish(self.i2c_cmd)

    # Water Control Handlers

    # def water_command(self, mode: bool, unit: int):
    #     """
    #     Set up the water control commands.

    #     The water control commands. NOTE: The documentation mentions that the commands
    #     are not implemented! Did not check if this is true, but added the command handling
    #     anyway if support will be added to the arduino firmware.

    #     Args:
    #         mode {bool}: False (or 1) for time based watering, True (or 2) for valume flow based
    #                      watering
    #         unit {int}: The amount of time (in time based watering) in millisec. or the pulse
    #                     count of the flow meter.
    #     """
    #     self.water_cmd.data = 'water_command ' + str(int(mode)+1) + ' ' + str(unit)

    #     self.devices_pub.publish(self.water_cmd)

    # Pin Control Handlers

    # def set_pin_value(self, pin: int, value: int, pin_mode: bool):
    #     """
    #     Set a farmduino pin to a selected value and pin_mode.

    #     Args:
    #         pin {int}: The pin the value is set on
    #         value {int}: Value to set
    #         pin_mode {bool}: 0 for digital, 1 for analog
    #     """
    #     self.manipulate_pin(mode=True, set_value1=True, pin=pin, value1=value, pin_mode=pin_mode)

    # def set_pin_value_2(self, pin: int, value1: int, delay: int, value2: int, pin_mode: bool):
    #     """
    #     Set a farmduino pin to a selected value and pin_mode with a delay.

    #     Setting value1 to pin and waiting for delay milliseconds. After the delay,
    #     value2 is set. Both sets are done in the selected pin_mode (0 for digital
    #     and 1 for analog)

    #     Args:
    #         pin {int}: The pin the value is set on
    #         value1 {int}: First value to set
    #         delay {int}: delay between the 2 values in milliseconds
    #         value2 {int}: Second value to set
    #         pin_mode {bool}: 0 for digital, 1 for analog
    #     """
    #     self.manipulate_pin(mode=True, set_value2=True, pin=pin, value1=value1,
    #                         delay=delay, value2=value2, pin_mode=pin_mode)

    # def read_pin(self, pin: int, pin_mode: bool):
    #     """
    #     Read the value from the selected pin in the parsed pin_mode.

    #     Args:
    #         pin {int}: The pin to read the value from
    #         pin_mode{bool}: 0 for digital, 1 for analog
    #     """
    #     self.manipulate_pin(mode=False, pin=pin, pin_mode=pin_mode)

    # def set_pin_io(self, pin: int, io_mode: bool):
    #     """
    #     Sett the IO of the selected pin.

    #     Args:
    #         pin {int}: the pin to set the IO for
    #         io_mode(bool): 0 for input, 1 for output
    #     """
    #     self.manipulate_pin(mode=True, set_io=True, pin=pin, pin_mode=io_mode)

    # def manipulate_pin(self, mode: bool = False, set_io: bool = False, set_value1: bool = False,
    #                    set_value2: bool = False, pin: int = 0, value1: int = 0,
    #                    value2: int = 0, delay: int = 0, pin_mode: bool = False):
    #     """
    #     Create the pin command used to handle and translate to the specific FarmBot commands.

    #     Args:
    #         mode{bool}: 1 (True) for SET, 0 (False) for READ
    #         set_io{bool}: If mode = True, and set_IO, the IO state of the pin will be set
    #         set_value1{bool}: If mode = True, and set)value, the value on the pin will be set
    #         set_value2{bool}: If mode and set_value2 are true, the value set command is used
    #         pin{int}: PIN worked on
    #         value1{int}: value to write on pin
    #         value2{int}: a second value to set on the pin after a delay
    #         delay{int}: time delay in millis
    #         pin_mode{bool}: (0-digital / 1-analog) OR (0-input / 1-output)
    #     """
    #     self.pin_cmd.data = ('pin_command ' + str(mode) + ' ' + str(set_io) + ' '
    #                          + str(set_value1)
    #                          + ' ' + str(set_value2) + ' ' + str(pin) + ' ' + str(value1)
    #                          + ' ' + str(value2) + ' ' + str(delay) + ' ' + str(int(pin_mode)))

    #     self.devices_pub.publish(self.pin_cmd)

    # def move_servo(self, pin: int, angle: float):
    #     """Create the move servo command."""
    #     self.servo_cmd.data = 'move_servo ' + str(pin) + ' ' + str(angle)
    #     self.devices_pub.publish(self.servo_cmd)
