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
    """Device module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the device module and the ROS2 service clients."""
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

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report to not leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        self._log_response(response)
        if on_done is not None:
            on_done(response)

    def _log_response(self, response):
        """Log a service response generically across the device response types."""
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not getattr(response, 'success', True):
            self.node.get_logger().warn(f'Command failed: {getattr(response, "message", "")}')
        elif hasattr(response, 'value'):
            self.node.get_logger().info(f'The value read is {response.value}')
        else:
            self.node.get_logger().info('Command successful')

    # I2C Control Handlers
    def i2c_read(self, element: int, parameter: int, on_done=None):
        """
        Call the ReadI2C service.

        Used to read the parameter of an i2c element.

        Args:
            element {int}: element in question
            parameter {int}: parameter of the element
            on_done {callable}: Optional completion callback
        """
        self._server_availability('ReadI2C', self.read_i2c_client)

        request = ReadI2C.Request()
        request.element = element
        request.parameter = parameter

        self.read_i2c_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def i2c_set(self, element: int, parameter: int, value: int, on_done=None):
        """
        Call the SetI2C service.

        Used to set a value to an I2C device.

        Args:
            element {int}: element in question
            parameter {int}: parameter of the element
            value {int}: new value of the parameter
            on_done {callable}: Optional completion callback
        """
        self._server_availability('SetI2C', self.set_i2c_client)

        request = SetI2C.Request()
        request.element = element
        request.parameter = parameter
        request.value = value

        self.set_i2c_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    # Water Control Handlers
    # NOTE: The documentation mentions that the commands are not implemented. Need to investigate

    def water_command(self, mode: bool, unit: float, on_done=None):
        """
        Service client to set up the water control commands.

        The water control commands. NOTE: The documentation mentions that the commands
        are not implemented! Did not check if this is true, but added the command handling
        anyway if support will be added to the arduino firmware.

        Args:
            mode {bool}: False (or 1) for time based watering, True (or 2) for valume flow based
                         watering
            unit {int}: The amount of time (in time based watering) in millisec. or the pulse
                        count of the flow meter
            on_done {callable}: Optional completion callback
        """
        self._server_availability('Watering', self.watering_client)

        request = Watering.Request()
        if mode:
            request.command = request.PULSE_DOSING
        else:
            request.command = request.TIMED_DOSING

        request.dose = unit

        self.watering_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    # Pin Control Handlers
    def set_pin_value(self, pin: int, value: int, pin_mode: bool,
                      pulse: bool, value2=0, delay_ms=0, on_done=None):
        """
        Service client to set a farmduino pin to a selected value and pin_mode.

        Args:
            pin {int}: The pin the value is set on
            value {int}: Value to set
            pin_mode {bool}: 0 for digital, 1 for analog
            pulse {bool}: Whether to apply a second value after a delay
            value2 {int}: Value to set after the delay
            delay_ms {int}: Delay before applying the second value in milliseconds
            on_done {callable}: Optional completion callback
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

        self.write_pin_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def read_pin(self, pin: int, pin_mode: bool, on_done=None):
        """
        Service client to read the value from the selected pin in the parsed pin_mode.

        Args:
            pin {int}: The pin to read the value from
            pin_mode{bool}: 0 for digital, 1 for analog
            on_done {callable}: Optional completion callback (`value` field holds reading)
        """
        self._server_availability('ReadPin', self.read_pin_client)

        request = ReadPin.Request()
        if pin_mode:
            request.mode = ReadPin.Request.ANALOG
        else:
            request.mode = ReadPin.Request.DIGITAL

        request.pin = pin

        self.read_pin_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def set_pin_io(self, pin: int, io_mode: bool, on_done=None):
        """
        Service client to set the IO of the selected pin.

        Args:
            pin {int}: the pin to set the IO for
            io_mode(bool): 0 for input, 1 for output
            on_done {callable}: Optional completion callback
        """
        self._server_availability('ConfigurePin', self.configure_pin_client)

        request = ConfigurePin.Request()
        request.output = io_mode
        request.pin = pin

        self.configure_pin_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def move_servo(self, pin: int, angle: float, on_done=None):
        """
        Call the FarmBot MoveServo service.

        Creates and sends a request to move a servo connected to a Farmduino
        pin to the specified angle.

        Args:
            pin {int}: Servo pin identifier
            angle {float}: Target servo angle
            on_done {callable}: Optional completion callback
        """
        self._server_availability('MoveServo', self.move_servo_client)

        request = MoveServo.Request()
        request.pin = pin
        request.angle = angle

        self.move_servo_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))
