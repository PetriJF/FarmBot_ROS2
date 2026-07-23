#!/usr/bin/env python3
"""
Device control utilities for ROS2 Farmbot.

ROS2 Python Module that enables the control of the different devices
that are connected to the Farmbot
"""
from farmbot_interfaces.srv import (ConfigurePin, MoveServo, ReadI2C, ReadPin,
                                    SetI2C, Watering, WritePin)

from rclpy.node import Node


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class DeviceControl:
    """Publish device commands (pin, water, I2C, servo) to the 'farmbot_command' topic."""

    def __init__(self, node: Node):
        """Initialize DeviceControl with a ROS2 node and its publishers."""
        self.node = node

        self.read_i2c_client = self.node.create_client(ReadI2C, 'read_i2c')
        self.set_i2c_client = self.node.create_client(SetI2C, 'set_i2c')
        self.watering_client = self.node.create_client(Watering, 'watering')
        self.read_pin_client = self.node.create_client(ReadPin, 'read_pin')
        self.write_pin_client = self.node.create_client(WritePin, 'write_pin')
        self.configure_pin_client = self.node.create_client(ConfigurePin, 'configure_pin')
        self.move_servo_client = self.node.create_client(MoveServo, 'move_servo')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Devices module failed: server unavailable')

    # I2C Control Handlers
    def i2c_read(self, element: int, parameter: int):
        """Service client to read from an I2C device."""
        self._server_availability('ReadI2C', self.read_i2c_client)

        request = ReadI2C.Request()
        request.element = element
        request.parameter = parameter

        future = self.read_i2c_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def i2c_set(self, element: int, parameter: int, value: int):
        """Service client to set a value to an I2C device."""
        self._server_availability('SetI2C', self.set_i2c_client)

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
        self._server_availability('Watering', self.watering_client)

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
        self._server_availability('WritePin', self.write_pin_client)

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
        self._server_availability('ReadPin', self.read_pin_client)

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
        self._server_availability('ConfigurePin', self.configure_pin_client)

        request = ConfigurePin.Request()
        request.output = io_mode
        request.pin = pin

        future = self.configure_pin_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def move_servo(self, pin: int, angle: float):
        """Create the move servo command."""
        self._server_availability('MoveServo', self.move_servo_client)

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
