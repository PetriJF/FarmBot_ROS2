"""
Straight-through command servers for the FarmBot SerialController.

Every service here encodes the request into FCode, then runs it and copy the outcome
into the response.
"""
from farmbot_interfaces.srv import (ConfigurePin, MoveServo, ReadI2C,
                                    ReadParameter, ReadPin, SetI2C, Watering,
                                    WriteParameter, WritePin)

from farmbot_utils.exceptions import EncodeError

from rclpy.node import Node

from std_srvs.srv import Trigger


class CommandServers:
    """Pass the services that map one request onto one FCode command."""

    def __init__(self, node: Node, encoder, run_command, param_vals, callback_group=None):
        """
        Create every straight-through command server on the node.

        Args:
            node {Node}: Node the servers are created on.
            encoder {Encoder}: Encoder turning a request into FCode.
            run_command {callable}: Awaitable the node uses to send one FCode.
            param_vals {callable}: Returns the current parameter table.
            callback_group: Callback group the servers run in.
        """
        self.node = node
        self.encoder = encoder
        self.run_command = run_command
        self.param_vals = param_vals

        # Device servers
        self.watering_server = node.create_service(
            Watering, 'watering', self.watering_command_server, callback_group=callback_group)
        self.read_i2c_server = node.create_service(
            ReadI2C, 'read_i2c', self.read_i2c_command_server, callback_group=callback_group)
        self.set_i2c_server = node.create_service(
            SetI2C, 'set_i2c', self.set_i2c_command_server, callback_group=callback_group)
        self.configure_pin_server = node.create_service(
            ConfigurePin, 'configure_pin', self.configure_pin_command_server,
            callback_group=callback_group)
        self.read_pin_server = node.create_service(
            ReadPin, 'read_pin', self.read_pin_command_server, callback_group=callback_group)
        self.write_pin_server = node.create_service(
            WritePin, 'write_pin', self.write_pin_command_server, callback_group=callback_group)
        self.move_servo_server = node.create_service(
            MoveServo, 'move_servo', self.move_servo_command_server,
            callback_group=callback_group)

        # Parameter servers
        self.read_parameter_server = node.create_service(
            ReadParameter, 'read_parameter', self.read_parameter_command_server,
            callback_group=callback_group)
        self.write_parameter_server = node.create_service(
            WriteParameter, 'write_parameter', self.write_parameter_command_server,
            callback_group=callback_group)
        self.list_all_parameter_server = node.create_service(
            Trigger, 'list_all_parameters', self.list_all_command_server,
            callback_group=callback_group)

        # Report servers
        self.end_stop_trigger_server = node.create_service(
            Trigger, 'end_stop', self.end_stop_command_server, callback_group=callback_group)
        self.sw_version_trigger_server = node.create_service(
            Trigger, 'sw_version', self.sw_version_command_server, callback_group=callback_group)
        self.curr_pos_trigger_server = node.create_service(
            Trigger, 'curr_pos', self.curr_position_command_server,
            callback_group=callback_group)

    async def _encoded(self, encode, request, response, returns_value=False):
        """Encode the request, then run it; an encoding error fails the response."""
        try:
            fcode = encode(request)
        except EncodeError as e:
            response.success = False
            response.message = str(e)
            if returns_value:
                response.value = -1
            return response

        return await self._send(fcode, response, returns_value)

    async def _send(self, fcode: str, response, returns_value=False):
        """Run one FCode command and copy its outcome into the response."""
        success, message, value = await self.run_command(fcode)
        response.success = success
        response.message = message
        if returns_value:
            response.value = value
        return response

    # Device commands
    async def watering_command_server(self, request: Watering.Request,
                                      response: Watering.Response) -> Watering.Response:
        """Handle a watering command service request."""
        return await self._encoded(self.encoder.encode_watering, request, response)

    async def read_i2c_command_server(self, request: ReadI2C.Request,
                                      response: ReadI2C.Response) -> ReadI2C.Response:
        """Handle a i2c reading command service request."""
        return await self._encoded(self.encoder.encode_read_i2c, request, response,
                                   returns_value=True)

    async def set_i2c_command_server(self, request: SetI2C.Request,
                                     response: SetI2C.Response) -> SetI2C.Response:
        """Handle a i2c setting command service request."""
        return await self._encoded(self.encoder.encode_set_i2c, request, response)

    async def configure_pin_command_server(
            self, request: ConfigurePin.Request,
            response: ConfigurePin.Response) -> ConfigurePin.Response:
        """Handle a configure pin command service request."""
        return await self._encoded(self.encoder.encode_configure_pin, request, response)

    async def read_pin_command_server(self, request: ReadPin.Request,
                                      response: ReadPin.Response) -> ReadPin.Response:
        """Handle a read pin command service request."""
        return await self._encoded(self.encoder.encode_read_pin, request, response,
                                   returns_value=True)

    async def write_pin_command_server(self, request: WritePin.Request,
                                       response: WritePin.Response) -> WritePin.Response:
        """Handle a write pin command service request."""
        return await self._encoded(self.encoder.encode_write_pin, request, response)

    async def move_servo_command_server(self, request: MoveServo.Request,
                                        response: MoveServo.Response) -> MoveServo.Response:
        """Handle a move servo command service request."""
        return await self._encoded(self.encoder.encode_move_servo, request, response)

    # Parameter commands
    async def read_parameter_command_server(
            self, request: ReadParameter.Request,
            response: ReadParameter.Response) -> ReadParameter.Response:
        """Handle a read parameter command service request."""
        return await self._encoded(
            lambda req: self.encoder.encode_read_parameter(req, self.param_vals()),
            request, response, returns_value=True)

    async def write_parameter_command_server(
            self, request: WriteParameter.Request,
            response: WriteParameter.Response) -> WriteParameter.Response:
        """Handle a write parameter command service request."""
        return await self._encoded(
            lambda req: self.encoder.encode_write_parameter(req, self.param_vals()),
            request, response)

    async def list_all_command_server(self, request: Trigger.Request,
                                      response: Trigger.Response) -> Trigger.Response:
        """Handle the list all parameter command service request (F20)."""
        return await self._send('F20', response)

    # Report commands
    async def end_stop_command_server(self, request: Trigger.Request,
                                      response: Trigger.Response) -> Trigger.Response:
        """Handle the end stops command service request (F81)."""
        return await self._send('F81', response)

    async def sw_version_command_server(self, request: Trigger.Request,
                                        response: Trigger.Response) -> Trigger.Response:
        """Handle the software version command service request (F83)."""
        return await self._send('F83', response)

    async def curr_position_command_server(self, request: Trigger.Request,
                                           response: Trigger.Response) -> Trigger.Response:
        """Handle the current position command service request (F82)."""
        return await self._send('F82', response)
