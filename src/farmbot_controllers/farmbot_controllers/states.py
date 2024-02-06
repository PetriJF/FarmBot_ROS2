from rclpy.node import Node
from farmbot_interfaces.msg import StateCommand

class State:
    def __init__(self, node: Node):
        self.node = node

        # The state interface object
        self.state_ = StateCommand()
        # The publisher for the state commands
        self.statePub_ = self.node.create_publisher(StateCommand, 'state_command', 10)
    
    ## State handling functions
    
    def electronicStop(self):
        """
        Imposes the electronic stop, turning off all the motors 
        """
        self.defineState(estop = True)

    def abortMovement(self):
        """
        Aborts the current movement command
        """
        self.defineState(abort_movement = True)

    def resetElectronicStop(self):
        """
        Resets the electronic stop
        """
        self.defineState(reset_estop = True)

    def requestEndStop(self):
        """
        Requests the end stops
        """
        self.defineState(rep_end_stop = True)

    def requestSoftwareVersion(self):
        """
        Requests the software version
        """
        self.defineState(rep_sw_ver = True)
    
    def requestCurrentPosition(self):
        """
        Requests the current position of the extruder
        """
        self.defineState(rep_curr_pos = True)

    def defineState(self, estop = False, abort_movement = False, reset_estop = False,
                    rep_end_stop = False, rep_curr_pos = False, rep_sw_ver = False):
        """
        Creates the state definition request.

        Args:
            estop {Bool}: Initializes the Electronic Stop, stopping all the motors
            abort_movement {Bool}: Aborts the current movement command
            reset_estop {Bool}: Resets the farmbot from the Electronic Stop state
            rep_end_stop {Bool}: Requests the end stops
            rep_curr_pos {Bool}: Requests the current position of the gantry
            rep_sw_ver   {Bool}: Requests the software version
        """
        
        self.state_.estop = estop
        self.state_.abort_movement = abort_movement
        self.state_.reset_estop = reset_estop
        self.state_.rep_end_stop = rep_end_stop
        self.state_.rep_curr_pos = rep_curr_pos
        self.state_.rep_sw_ver = rep_sw_ver

        self.statePub_.publish(self.state_)