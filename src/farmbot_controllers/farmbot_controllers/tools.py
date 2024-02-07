from rclpy.node import Node
from farmbot_interfaces.srv import StringRepReq
from farmbot_controllers.movement import Movement
from farmbot_controllers.devices import DeviceControl


class ToolCommands:
    def __init__(self, node: Node, mvm: Movement, devices: DeviceControl):
        # The farmbot node extension
        self.node_ = node
        # Objects linking to the state and movement modules
        self.mvm_ = mvm
        self.devices_ = devices

        # The ID of the current tool mounted. Should be 0 when no tool mounted!
        self.current_tool_id_ = 0

    def check_if_used(self):
        """
        Check if the tool unit has a toolhead mounted to it

        Returns:{bool}: True if a tool is mounted
        """
        # Check if there is continuity between pins B and C
        

        pass

    # Peripheral control functions

    def vacuum_pump_on(self):
        vacuum_pin = 9
        self.devices_.set_pin_value(pin = vacuum_pin, value = 1, pin_mode = False)
    
    def vacuum_pump_off(self):
        vacuum_pin = 9
        self.devices_.set_pin_value(pin = vacuum_pin, value = 0, pin_mode = False)

    def water_pump_on(self):
        water_pin = 8
        self.devices_.set_pin_value(pin = water_pin, value = 1, pin_mode = False)
    
    def water_pump_off(self):
        water_pin = 8
        self.devices_.set_pin_value(pin = water_pin, value = 0, pin_mode = False)

    def led_strip_on(self):
        light_pin = 7
        self.devices_.set_pin_value(pin = light_pin, value = 1, pin_mode = False)
    
    def led_strip_off(self):
        light_pin = 7
        self.devices_.set_pin_value(pin = light_pin, value = 0, pin_mode = False)

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