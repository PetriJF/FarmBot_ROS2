"""
ROS2 Farmbot state module.

Provides state command helpers for emergency stop, abort, reset, and other
command-handler interactions via the farmbot_command publisher.
"""
from farmbot_controller import ServerError

from rclpy.node import Node

# from std_msgs.msg import String

from std_srvs.srv import Trigger


class State:
    """
    ROS2 Farmbot State module.

    Forms the commands that are sent to the command handling package
    """

    def __init__(self, node: Node):
        """
        ROS2 Farmbot State Module Constructor.

        Args:
            node {Node}: The node the module extends
        """
        self.node = node

        self.estop_client = self.node.create_client(Trigger, 'estop')
        self.abort_client = self.node.create_client(Trigger, 'abort')
        self.resume_client = self.node.create_client(Trigger, 'resume')
        self.end_stop_client = self.node.create_client(Trigger, 'end_stop')
        self.sw_version_client = self.node.create_client(Trigger, 'sw_version')
        self.curr_position_client = self.node.create_client(Trigger, 'curr_pos')

        # # The state interface object
        # self.state = String()
        # # The publisher for the state commands
        # self.state_pub = self.node.create_publisher(String, 'farmbot_command', 10)

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            raise ServerError(f'{cmd_name} Server not available!')

        request = Trigger.Request()
        future = client.call_async(request=request)
        future.add_done_callback(self.client_callback)

    # Service Client
    def estop(self):
        """Service client for estop."""
        try:
            self._server_availability('Estop', self.estop_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def abort_movement(self):
        """Service client for abort."""
        try:
            self._server_availability('Abort', self.abort_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def reset_estop(self):
        """Service client for abort."""
        try:
            self._server_availability('Reset estop', self.resume_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def request_end_stop(self):
        """Service client to request the end stops."""
        try:
            self._server_availability('End stop request', self.end_stop_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def request_sw_version(self):
        """Service client to request the software version."""
        try:
            self._server_availability('Software version request', self.sw_version_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def request_curr_pos(self):
        """Service client to request the current position."""
        try:
            self._server_availability('Current position request', self.curr_position_client)
        except ServerError as e:
            self.node.get_logger().fatal(e)
            raise

    def client_callback(self, future):
        """Service client callback once the request is send."""
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            else:
                self.node.get_logger().info(response.message)

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    # # State handling functions
    # def estop(self):
    #     """Impose the electronic stop, turning off all the motors."""
    #     self.define_state(estop=True)

    # def abort_movement(self):
    #     """Abort the current movement command."""
    #     self.define_state(abort_movement=True)

    # def reset_estop(self):
    #     """Reset the electronic stop."""
    #     self.define_state(reset_estop=True)

    # def request_end_stop(self):
    #     """Request the end stops."""
    #     self.define_state(rep_end_stop=True)

    # def request_sw_version(self):
    #     """Request the software version."""
    #     self.define_state(rep_sw_ver=True)

    # def request_curr_pos(self):
    #     """Request the current position of the extruder."""
    #     self.define_state(rep_curr_pos=True)

    # def define_state(self, estop=False, abort_movement=False, reset_estop=False,
    #                  rep_end_stop=False, rep_curr_pos=False, rep_sw_ver=False):
    #     """
    #     Create the state definition request.

    #     Args:
    #         estop {Bool}: Initializes the Electronic Stop, stopping all the motors
    #         abort_movement {Bool}: Aborts the current movement command
    #         reset_estop {Bool}: Resets the farmbot from the Electronic Stop state
    #         rep_end_stop {Bool}: Requests the end stops
    #         rep_curr_pos {Bool}: Requests the current position of the gantry
    #         rep_sw_ver   {Bool}: Requests the software version
    #     """
    #     self.state.data = ('state_command ' + str(estop) + ' ' + str(abort_movement) + ' '
    #                        + str(reset_estop) + ' ' + str(rep_end_stop) + ' ' + str(rep_curr_pos)
    #                        + ' ' + str(rep_sw_ver))

    #     self.state_pub.publish(self.state)
