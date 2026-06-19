"""
ROS2 Farmbot state module.

Provides state command helpers for emergency stop, abort, reset, and other
command-handler interactions via the farmbot_command publisher.
"""
from rclpy.node import Node

from std_msgs.msg import String


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

        # The state interface object
        self.state = String()
        # The publisher for the state commands
        self.state_pub = self.node.create_publisher(String, 'farmbot_command', 10)

    # State handling functions

    def estop(self):
        """Impose the electronic stop, turning off all the motors."""
        self.define_state(estop=True)

    def abort_movement(self):
        """Abort the current movement command."""
        self.define_state(abort_movement=True)

    def reset_estop(self):
        """Reset the electronic stop."""
        self.define_state(reset_estop=True)

    def request_end_stop(self):
        """Request the end stops."""
        self.define_state(rep_end_stop=True)

    def request_sw_version(self):
        """Request the software version."""
        self.define_state(rep_sw_ver=True)

    def request_curr_pos(self):
        """Request the current position of the extruder."""
        self.define_state(rep_curr_pos=True)

    def define_state(self, estop=False, abort_movement=False, reset_estop=False,
                     rep_end_stop=False, rep_curr_pos=False, rep_sw_ver=False):
        """
        Create the state definition request.

        Args:
            estop {Bool}: Initializes the Electronic Stop, stopping all the motors
            abort_movement {Bool}: Aborts the current movement command
            reset_estop {Bool}: Resets the farmbot from the Electronic Stop state
            rep_end_stop {Bool}: Requests the end stops
            rep_curr_pos {Bool}: Requests the current position of the gantry
            rep_sw_ver   {Bool}: Requests the software version
        """
        self.state.data = ('state_command ' + str(estop) + ' ' + str(abort_movement) + ' '
                           + str(reset_estop) + ' ' + str(rep_end_stop) + ' ' + str(rep_curr_pos)
                           + ' ' + str(rep_sw_ver))

        self.state_pub.publish(self.state)
