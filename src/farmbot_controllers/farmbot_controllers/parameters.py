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

    def read_param(self, param: int):
        """
        Call the ReadParameter service.

        Used to read the value on parameter {param}.

        Args:
            param {int}: Parameter in question
        """
        self._server_availability('ReadParameter', self.read_param_client)

        request = ReadParameter.Request()
        request.param = param

        future = self.read_param_client.call_async(request=request)
        future.add_done_callback(self.read_client_callback)

    def read_client_callback(self, future):
        """
        Handle the response of a parameter read service request.

        Processes the service response and logs the retrieved parameter value
        or the command status depending on the result of the request.
        """
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            elif response.value != -1:
                self.node.get_logger().info(f'The value of the parameter is {response.value}')

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    def list_all_params(self):
        """
        Call the ListAllParameters service.

        Used to all the parameters on the Farmduino.
        """
        self._server_availability('ListAllParameters', self.list_all_param_client)

        request = Trigger.Request()

        future = self.list_all_param_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def write_param(self, param: int, value: int, during_calibration: bool):
        """
        Call the WriteParameter service.

        Used  to write {value} to parameter {param}.

        Args:
            param {int}: Parameter in question
            value {int}: Value written to param if write or update modes are active
            during_calibration {bool}: Indicates whether Farmbot is currently calibrating or not
        """
        self._server_availability('WriteParameter', self.write_param_client)

        request = WriteParameter.Request()
        request.param = param
        request.value = value
        request.during_calibration = during_calibration

        future = self.write_param_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def client_callback(self, future):
        """
        Handle the response of a service request except parameter read requests.

        Processes the service response and logs the command status depending
        on the result of the request.
        """
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
