from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from std_msgs.msg import Bool 
from farmbot_interfaces.action import GetUARTResponse
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
        self.current_tool_id_ = 0# Get UART Response to Request Client
        
        self.sequence_ = []
        self.command_type_ = ''
        self.farmbot_busy_ = False

        self.get_response_client_ = ActionClient(self.node_, GetUARTResponse, 'uart_response')
        self.busy_state_sub_ = self.node_.create_subscription(Bool, 'busy_state', self.status_callback, 10)
        self.sequencing_timer_ = self.node_.create_timer(1.0, self.sequencing_timer)

    ## NOT IN USE WIP
    def get_pin_response(self, code: str, timeout: int):
        # Waiting for server to be ready
        self.get_response_client_.wait_for_server()

        # Create the goal
        goal = GetUARTResponse.Goal()
        goal.code = code
        goal.timeout_sec = timeout

        # Send the goal
        self.get_response_client_. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)
    ## NOT IN USE WIP
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_. \
                get_result_async(). \
                    add_done_callback(self.goal_result_callback)
    ## NOT IN USE WIP
    def goal_result_callback(self, future):
        self.node_.get_logger().info("SS")
        message = future.result().result.msg.split(' ')
        self.node_.get_logger().info(future.result().result.msg)
        if message[0] == 'R41' and message[1] == 'P63':
            self.node_.get_logger().info(f"A tool is {'not ' if bool(message[2][-1]) else ''} mounted on the tool element")
 
    
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
    def map_cmd_client(self, cmd = str):
        '''
        Tool command service client used to communicate between the farmbot
        controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        '''
        # Initializing the client and wait for map server confirmation
        client = self.node_.create_client(StringRepReq, 'map_info')
        while not client.wait_for_service(1.0):
            self.node_.get_logger().warn("Waiting for Map Server...")
        
        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = cmd

        # Call async and add the response callback
        future = client.call_async(request = request)
        future.add_done_callback(self.cmd_sequence_callback)

    def cmd_sequence_callback(self, future):
        '''
        Tool command service response callback from the map handler. Returns
        the processed information or task success state for the given request

        Args:
            future{Service Response}: Contains the response from the service
        '''
        # Register the response of the server
        cmd = future.result().data.split('\n')
        # For a coordinate command response
     
        self.node_.get_logger().info(future.result().data)

        self.sequence_.extend(cmd)
        # cmdType = ''
        # for mvm in cmd: # move extruder to all coordinates in the string list
        #     if mvm[:2] == 'CC':
        #         cmdType = 'CC'
        #         continue

        #     if cmdType == 'CC':
        #         coords = mvm.split(' ')
        #         self.mvm_.moveGantryAbsolute(x_coord = float(coords[0]), 
        #                                      y_coord = float(coords[1]), 
        #                                      z_coord = float(coords[2]))
        #     # Check if tool is mounted properly
            #self.devices_.read_pin(63, False)
            
            #self.get_pin_response('63', -1)

    def sequencing_timer(self):
        if not len(self.sequence_):
            return

        if self.sequence_[0][:2] in ['CC', 'DC']:
            self.command_type_ = self.sequence_[0][:2]
            self.sequence_.pop(0)

        if not self.farmbot_busy_:
            if self.command_type_ == '':
                self.node_.get_logger().warn(f"Command type not set! Not enough context! Command '{self.sequence_[0]}' ignored")
                return
            
            if self.command_type_ == 'CC':
                coords = self.sequence_[0].split(' ')
                self.mvm_.moveGantryAbsolute(x_coord = float(coords[0]), 
                                             y_coord = float(coords[1]), 
                                             z_coord = float(coords[2]))
                self.sequence_.pop(0)
                return
            elif self.command_type_ == 'DC':
                cmd = self.sequence_[0].split(' ')
                if cmd[0] == 'Vacuum':
                    if cmd[1] == '1':
                        self.vacuum_pump_on()
                    elif cmd[1] == '0':
                        self.vacuum_pump_off()
                    else:
                        self.node_.get_logger().warn("Vacuum pump command has a state other than on or off. Command ignored!")
                self.sequence_.pop(0)


    def status_callback(self, state: Bool):
        self.farmbot_busy_ = state.data