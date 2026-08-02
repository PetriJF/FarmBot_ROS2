"""
FarmBot parameter command module.

Provides helper methods to read, list, and write FarmBot parameters
through ROS2 service interactions.
"""
from farmbot_interfaces.srv import ReadParameter, WriteParameter

from rclpy.node import Node

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class Parameters:
    """Parameter module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the parameter module and the ROS2 service clients."""
        self.node = node

        self.read_param_client = self.node.create_client(ReadParameter, 'read_parameter')
        self.write_param_client = self.node.create_client(WriteParameter, 'write_parameter')
        self.list_all_param_client = self.node.create_client(Trigger, 'list_all_parameters')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Parameter module failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report to not leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not getattr(response, 'success', True):
            self.node.get_logger().warn(f'Command failed: {getattr(response, "message", "")}')
        elif hasattr(response, 'value'):
            self.node.get_logger().info(f'The value of the parameter is {response.value}')
        else:
            self.node.get_logger().info('Command successful')
        if on_done is not None:
            on_done(response)

    def read_param(self, param: int, on_done=None):
        """
        Call the ReadParameter service.

        Used to read the value on parameter {param}.

        Args:
            param {int}: Parameter in question
            on_done {callable}: Optional completion callback invoked with the response
                                (its 'value' field holds the reading).
        """
        self._server_availability('ReadParameter', self.read_param_client)

        request = ReadParameter.Request()
        request.param = param

        self.read_param_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))

    def list_all_params(self, on_done=None):
        """
        Call the ListAllParameters service.

        Used to all the parameters on the Farmduino.
        """
        self._server_availability('ListAllParameters', self.list_all_param_client)

        self.list_all_param_client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._complete(future, on_done))

    def write_param(self, param: int, value: int, during_calibration: bool, on_done=None):
        """
        Call the WriteParameter service.

        Used  to write {value} to parameter {param}.

        Args:
            param {int}: Parameter in question
            value {int}: Value written to param if write or update modes are active
            during_calibration {bool}: Indicates whether Farmbot is currently calibrating or not
            on_done {callable}: Optional completion callback
        """
        self._server_availability('WriteParameter', self.write_param_client)

        request = WriteParameter.Request()
        request.param = param
        request.value = value
        request.during_calibration = during_calibration

        self.write_param_client.call_async(request=request).add_done_callback(
            lambda future: self._complete(future, on_done))
