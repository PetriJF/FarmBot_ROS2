from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from std_msgs.msg import Bool 
from farmbot_interfaces.action import GetUARTResponse
from farmbot_interfaces.srv import StringRepReq
from farmbot_controllers.movement import Movement
from farmbot_controllers.devices import DeviceControl

class WaitForRequest:
    def __init__(self):    
        self.wait_flag = False
        self.wait_for = -1
        self.result = -1
        self.expected = -1


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
        
        self.wait_for_request_ = WaitForRequest()
        self.farmbot_busy_ = False
        self.wait_for_camera_ = False

        self.get_response_client_ = ActionClient(self.node_, GetUARTResponse, 'uart_response')
        self.busy_state_sub_ = self.node_.create_subscription(Bool, 'busy_state', self.status_callback, 10)
        self.sequencing_timer_ = self.node_.create_timer(1.0, self.sequencing_timer)
 
    def clear_sequence(self):
        '''
        Clearing the sequence in cases such as an electronic stop
        '''
        self.sequence_ = []


    # Peripheral control functions TODO: Improve

    # Turning on or off the vacuum pump
    def vacuum_pump(self, state: int):
        vacuum_pin = 9
        self.devices_.set_pin_value(pin = vacuum_pin, value = state, pin_mode = False)

    # Turning on or off the water pump
    def water_pump(self, state: int):
        water_pin = 8
        self.devices_.set_pin_value(pin = water_pin, value = state, pin_mode = False)

    # Turning the water pump on, waiting for the specified time in ms and turning it off
    def water_pulses(self, delay = 500):
        water_pin = 8
        self.devices_.set_pin_value_2(pin = water_pin, value1 = 1, delay = delay, value2 = 0, pin_mode = False)

    # Turning on or off the LED strip
    def led_strip(self, state: int):
        light_pin = 7
        self.devices_.set_pin_value(pin = light_pin, value = state, pin_mode = False)
    
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
            self.node_.get_logger().warn('Waiting for Map Server...')
        
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

    def sequencing_timer(self):
        '''
        Sequencing timer that handles sending commands to the farmbot. If the 
        Farmbot is busy, the commands are added in the sequence queue and the 
        queue is emptied in FIFO fashion.
        '''
        # If waiting for camera or queue is empty, return
        if self.wait_for_camera_:
            return
        if not len(self.sequence_):
            return

        # Set the sequence command type. CC - Coord. Cmd, DC - Device Cmd, 
        # VC - Vision Cmd
        if self.sequence_[0][:2] in ['CC', 'DC', 'VC']:
            self.command_type_ = self.sequence_[0][:2]
            self.sequence_.pop(0)

        # If the farmbot is not busy and a request's response is not processed
        if not self.farmbot_busy_ and not self.wait_for_request_.wait_flag:
            # If the command type was not set, ignore
            if self.command_type_ == '':
                self.node_.get_logger().warn(f"Command type not set! Not enough context! Command '{self.sequence_[0]}' ignored")
                return
            
            # Used to check if a tool was mounted/unmounted properly. Note that 63 represents the connection between pins B and C on the UTP
            if (self.wait_for_request_.result != -1 and self.wait_for_request_.wait_for == 63
                    and self.wait_for_request_.result == self.wait_for_request_.expected):
                self.node_.get_logger().info(f"Tool {'mounted' if self.wait_for_request_.expected == 0 else 'unmounted'} successfully")
                self.wait_for_request_.expected = -1
                self.wait_for_request_.result = -1
                self.wait_for_request_.wait_for = -1
            elif (self.wait_for_request_.result != -1 and self.wait_for_request_.wait_for == 63
                    and self.wait_for_request_.result != self.wait_for_request_.expected):
                self.node_.get_logger().warn(f"FAILED TOOL {'MOUNTING' if self.wait_for_request_.expected == 0 else 'UNMOUNTING'}!! Stopping sequence")
                self.wait_for_request_.expected = -1
                self.wait_for_request_.result = -1
                self.wait_for_request_.wait_for = -1
                self.clear_sequence()
                return

            # Move the gantry to the parsed coordinates
            if self.command_type_ == 'CC':
                coords = self.sequence_[0].split(' ')
                self.mvm_.move_gantry_abs(x_coord = float(coords[0]), 
                                             y_coord = float(coords[1]), 
                                             z_coord = float(coords[2]))
                self.sequence_.pop(0)
                return
            # Manipulate the device as indicated in the command
            elif self.command_type_ == 'DC':
                cmd = self.sequence_[0].split(' ')
                # Checking if a tool was mounted properly
                if cmd[0] == 'CHECK':
                    self.devices_.read_pin(63, False)
                    self.wait_for_request_.wait_flag = True
                    self.wait_for_request_.wait_for = 63
                    self.wait_for_request_.expected = int(cmd[1])
                if cmd[0] == 'Vacuum':
                    if cmd[1] in ['0', '1']:
                        self.vacuum_pump(state = int(cmd[1]))
                    else:
                        self.node_.get_logger().warn(f'Vacuum pump command has a state other than on or off. Command ignored!')
                if cmd[0] == 'WaterPulses':
                    if int(cmd[1]):
                        self.water_pulses(delay = int(cmd[1]))
                self.sequence_.pop(0)
            # Handle Vision Commands
            elif self.command_type_ == 'VC':
                # Vision command
                self.stitch_panorama_client()

    def uart_message(self, msg: str):
        '''
        Getting the responses to the requests done in the sequencer
        '''
        if ' ' in msg:
            info = msg.split(' ')
            if info[0] == 'R41' and info[1] == f'P{str(self.wait_for_request_.wait_for)}':
                self.wait_for_request_.result = int(info[2][1:])
                self.wait_for_request_.wait_flag = False

    def stitch_panorama_client(self):
        '''
        Tool command service client used to communicate between the farmbot
        controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        '''
        # Block sequencing here async
        self.wait_for_camera_ = True

        # Initializing the client and wait for map server confirmation
        client = self.node_.create_client(StringRepReq, 'form_panorama')
        while not client.wait_for_service(1.0):
            self.node_.get_logger().warn('Waiting for Camera Stitching Server...')
        
        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = 'ADD HERE ANY SETUP THAT MIGHT CHANGE'

        # Call async and add the response callback
        future = client.call_async(request = request)
        future.add_done_callback(self.stitch_callback)

    def stitch_callback(self, future):
        '''
        Camera Service Server callback
        '''
        self.wait_for_camera_ = False

    def cam_calib_client(self):
        '''
        Tool command service client used to communicate between the farmbot
        controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        '''
        # Initializing the client and wait for map server confirmation
        client = self.node_.create_client(StringRepReq, 'calibrate_luxonis')
        while not client.wait_for_service(1.0):
            self.node_.get_logger().warn('Waiting for Camera Calibration Server...')
        
        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = 'ADD HERE ANY SETUP THAT MIGHT CHANGE'

        # Call async and add the response callback
        future = client.call_async(request = request)
        future.add_done_callback(self.cmd_sequence_callback)
    
    def panorama_client(self):
        '''
        Tool command service client used to communicate between the farmbot
        controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        '''
        # Initializing the client and wait for map server confirmation
        client = self.node_.create_client(StringRepReq, 'panorama_sequence')
        while not client.wait_for_service(1.0):
            self.node_.get_logger().warn('Waiting for Panorama Sequencing Server...')
        
        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = 'ADD HERE ANY SETUP THAT MIGHT CHANGE'

        # Call async and add the response callback
        future = client.call_async(request = request)
        future.add_done_callback(self.cmd_sequence_callback)

    def status_callback(self, state: Bool):
        '''
        Callback from the UART Handler that transmits the busy state of the farmbot
        '''
        self.farmbot_busy_ = state.data