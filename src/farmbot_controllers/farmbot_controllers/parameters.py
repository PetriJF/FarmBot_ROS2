"""
Parameter command module for the FarmBot controller node.

Provides read, list, and write parameter operations using the ROS2
farmbot_command publisher.
"""
from farmbot_interfaces.srv import ReadParameter, WriteParameter

from rclpy.node import Node

# from std_msgs.msg import String

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class Parameters:
    """Helper for FarmBot parameter commands via the ROS2 farmbot_command publisher."""

    def __init__(self, node: Node):
        """Initialize the parameter module and the ROS2 farmbot_command publisher."""
        self.node = node

        self.read_param_client = self.node.create_client(ReadParameter, 'read_parameter')
        self.write_param_client = self.node.create_client(WriteParameter, 'write_parameter')
        self.list_all_param_client = self.node.create_client(Trigger, 'list_all_parameters')

        # # Used for reading and writing to the FarmBot Parameters
        # self.paramHandler = String()
        # # Parameter Command Publisher
        # self.paramCmdPub = self.node.create_publisher(String, 'farmbot_command', 10)

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Parameter module failed: server unavailable')

    def readParam(self, param: int):
        """
        Service client to read the value on parameter {param}.

        Args:
            param {Int}: Parameter in question
        """
        self._server_availability('ReadParameter', self.read_param_client)

        request = ReadParameter.Request()
        request.param = param

        future = self.read_param_client.call_async(request=request)
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
                self.node.get_logger().info(f'The value of the parameter is {response.value}')

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    def listAllParams(self):
        """Service client to list all parameters."""
        self._server_availability('ListAllParameters', self.list_all_param_client)

        request = Trigger.Request()

        future = self.list_all_param_client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    def writeParam(self, param: int, value: int, during_calibration: bool):
        """
        Service client to write {value} to parameter {param}.

        Args:
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        """
        self._server_availability('WriteParameter', self.write_param_client)

        request = WriteParameter.Request()
        request.param = param
        request.value = value
        request.during_calibration = during_calibration

        future = self.write_param_client.call_async(request=request)
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

    # Parameter Handling Commands

    # def readParam(self, param=int):
    #     """
    #     Read the value on parameter {param}.

    #     Args:
    #         param {Int}: Parameter in question
    #     """
    #     self.parameterHandler(list=False, write=False, read=True, update=False, param=param)

    # def listAllParams(self):
    #     """List all the parameters and their values."""
    #     self.parameterHandler(list=True, write=False, read=False, update=False)

    # def writeParam(self, param=int, value=int):
    #     """
    #     Write {value} to parameter {param}.

    #     Args:
    #         param {Int}: Parameter in question
    #         value {Int}: Value written to param if write or update modes are active
    #     """
    #     self.parameterHandler(list=False, write=True, read=False,
    #                           update=False, param=param, value=value)

    # def updateParam(self, param=int, value=int):
    #     """
    #     Update parameter {param} with {value}.

    #     Args:
    #         param {Int}: Parameter in question
    #         value {Int}: Value written to param if write or update modes are active
    #     """
    #     self.parameterHandler(list=False, write=False, read=False, update=True,
    #                           param=param, value=value)

    # def parameterHandler(self, parameter_list=bool, write=bool, read=bool,
    #                      update=bool, param=int, value=int):
    #     """
    #     Handle parameter commands.

    #     Args:
    #         parameter_list {bool}: If true, all the parameters will be listed
    #         write {bool}: If true, value V will be written to parameter P
    #         read {bool}: If true, parameter P will be listed
    #         update {bool}: If true, parameter P will be updated with value V (e.g. during calib.)
    #         param {Int}: Parameter in question
    #         value {Int}: Value written to param if write or update modes are active
    #     """
    #     self.paramHandler.data = ('parameter_command ' + str(parameter_list) + ' ' + str(write)
    #                               + ' ' + str(read) + ' ' + str(update) + ' ' + str(param) + ' '
    #                               + str(value))

    #     self.paramCmdPub.publish(self.paramHandler)
