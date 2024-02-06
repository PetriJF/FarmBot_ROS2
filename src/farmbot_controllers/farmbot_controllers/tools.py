from rclpy.node import Node
from farmbot_interfaces.srv import StringRepReq
from farmbot_controllers.movement import Movement


class ToolCommands:
    def __init__(self, node: Node, mvm: Movement):
        # The farmbot node extension
        self.node_ = node
        # Objects linking to the state and movement modules
        self.mvm_ = mvm

        # The ID of the current tool mounted. Should be 0 when no tool mounted!
        self.current_tool_id_ = 0

    def check_if_used(self):
        """
        Check if the tool unit has a toolhead mounted to it

        Returns:{bool}: True if a tool is mounted
        """
        # Check if there is continuity between pins B and C
        

        pass

    ## Tool Exchanging Client
    def tool_exchange_client(self, cmd = str):
        '''
        Tool command service client used to communicate between the farmbot
        controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        '''
        # Initializing the client and wait for map server confirmation
        client = self.node_.create_client(StringRepReq, 'map_cmd')
        while not client.wait_for_service(1.0):
            self.node_.get_logger().warn("Waiting for Map Server...")
        
        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = cmd

        # Call async and add the response callback
        future = client.call_async(request = request)
        future.add_done_callback(self.tool_cmd_sequence_callback)

    def tool_cmd_sequence_callback(self, future):
        '''
        Tool command service response callback from the map handler. Returns
        the processed information or task success state for the given request

        Args:
            future{Service Response}: Contains the response from the service
        '''
        # Register the response of the server
        cmd = future.result().data.split('\n')
        # For a coordinate command response
        if cmd[0][:2] == 'CC':
            for mvm in cmd[1:]: # move extruder to all coordinates in the string list
                coords = mvm.split(' ')
                self.mvm_.moveGantryAbsolute(x_coord = float(coords[0]), 
                                             y_coord = float(coords[1]), 
                                             z_coord = float(coords[2]))