"""
Sequencer module for FarmBot command flow control.

Defines a simple request wait state and the Sequencer class used by the
controller to coordinate movement and device actions.
"""
from farmbot_controllers.devices import DeviceControl
from farmbot_controllers.movement import Movement

from farmbot_interfaces.srv import StringRepReq

from rclpy.node import Node

from std_msgs.msg import Bool, String


class WaitForRequest:
    """Class used to control the flow of commands exported by the controller."""

    def __init__(self):
        """Initialize the sequencer with the ROS node, movement module, and device controller."""
        self.wait_flag = False
        self.wait_for = -1
        self.result = -1
        self.expected = -1
        self.index = -1


class Sequencer:
    """Coordinate FarmBot movement and device command sequencing."""

    def __init__(self, node: Node, mvm: Movement, devices: DeviceControl):
        """
        Initialize the sequencer with the ROS node, movement module, and device controller.

        Args:
            node (Node): ROS2 node used for publishing/subscribing.
            mvm (Movement): Movement helper instance.
            devices (DeviceControl): Device controller instance.
        """
        # The farmbot node extension
        self.node = node
        # Objects linking to the state and movement modules
        self.mvm = mvm
        self.devices = devices

        self.x = 0
        self.y = 0
        self.z = 0

        self.sequence = []
        self.command_type = ''

        self.wait_for_request = WaitForRequest()
        self.farmbot_estop = False
        self.wait_for_camera = False
        self.general_wait_flag = False
        self.ticks = 0

        self.estop_sub = self.node.create_subscription(Bool, 'estop',
                                                       self.status_callback, 10)
        self.sequencer_sub = self.node.create_subscription(String, 'sequencer',
                                                           self.extend_sequence, 10)
        self.sequencing_timer_ = self.node.create_timer(1.0, self.sequencing_timer)

    def clear_sequence(self):
        """Clear the sequence in cases such as an electronic stop."""
        self.sequence.clear()

    # Peripheral control functions TODO: Improve

    def vacuum_pump(self, state: int):
        """Turn on or off the vacuum pump."""
        vacuum_pin = 9
        self.devices.set_pin_value(pin=vacuum_pin, value=state, pin_mode=False)

    def water_pump(self, state: int):
        """Turn on or off the water pump."""
        water_pin = 8
        self.devices.set_pin_value(pin=water_pin, value=state, pin_mode=False)

    def water_pulses(self, delay=500):
        """Turn the water pump on, wait for the specified time in ms and turn it off."""
        water_pin = 8
        self.devices.set_pin_value_2(pin=water_pin, value1=1, delay=delay, value2=0, pin_mode=False)

    def peripheral_4(self, state: int):
        """Turn on or off the peripheral 4 pin."""
        peripheral4_pin = 10
        self.devices.set_pin_value(pin=peripheral4_pin, value=state, pin_mode=False)

    def peripheral4_pulses(self, delay=500):
        """Open and close secondary solenoid on peripheral 4 pin, wait for the specified time."""
        peripheral4_pin = 10
        self.devices.set_pin_value_2(pin=peripheral4_pin, value1=1, delay=delay,
                                     value2=0, pin_mode=False)

    def peripheral_5(self, state: int):
        """Turn on or off the peripheral 5 pin."""
        peripheral5_pin = 12
        self.devices.set_pin_value(pin=peripheral5_pin, value=state, pin_mode=False)

    def led_strip(self, state: int):
        """Turn on or off the LED strip."""
        light_pin = 7
        self.devices.set_pin_value(pin=light_pin, value=state, pin_mode=False)

    # Map Handler Client
    def map_cmd_client(self, cmd: str):
        """
        Service client used to communicate between the farmbot controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        """
        # Initializing the client and wait for map server confirmation
        client = self.node.create_client(StringRepReq, 'map_info')
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn('Waiting for Map Server...')

        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = cmd

        # Call async and add the response callback
        future = client.call_async(request=request)
        future.add_done_callback(self.cmd_sequence_callback)

    def extend_sequence(self, cmd: String):
        """Append commands from a newline-separated String message to the current sequence."""
        temp = cmd.data.split('\n')
        self.sequence.extend(temp)

    def cmd_sequence_callback(self, future):
        """
        Tool command service response callback from the map handler.

        Returns the processed information or task success state for the given request
        Args:
            future{Service Response}: Contains the response from the service
        """
        # Register the response of the server
        cmd = future.result().data.split('\n')
        # For a coordinate command response

        self.node.get_logger().info(future.result().data)

        if cmd[0] == 'SUCCESS' and self.general_wait_flag:
            self.general_wait_flag = False
        elif cmd[0] == 'FAILED' and self.general_wait_flag:
            self.general_wait_flag = False
            self.node.get_logger().warning('A Server Response Failed.. Clearing sequence')
            self.clear_sequence()

        elif cmd[0] not in ['', 'SUCCESS', 'FAILED', 'UNRECOGNIZED']:
            self.sequence.extend(cmd)

    def sequencing_timer(self):
        """
        Sequencing timer that handles sending commands to the farmbot.

        If the Farmbot is busy, the commands are added in the sequence queue and the queue is
        emptied in FIFO fashion.
        """
        # If waiting for camera or queue is empty, return
        if self.ticks:
            self.node.get_logger().info(f'Waiting for {self.ticks} more ticks')
            self.ticks -= 1
            return

        if self.wait_for_camera or self.general_wait_flag:
            return
        if not len(self.sequence):
            return

        # Set the sequence command type. CC - Coord. Cmd, DC - Device Cmd,
        # VC - Vision Cmd, SC - Servo Cmd
        if self.sequence[0][:2] in ['CC', 'DC', 'SC', 'VC', 'TD']:
            self.command_type = self.sequence[0][:2]
            self.sequence.pop(0)
            return

        # If the farmbot is not busy and a request's response is not processed
        if not self.farmbot_estop and not self.wait_for_request.wait_flag:
            # If the command type was not set, ignore
            if self.command_type == '':
                self.node.get_logger().warn(f"Command type not set! Not enough context! \
                                            Command '{self.sequence[0]}' ignored")
                return

            # Used to check if a tool was mounted/unmounted properly.
            # Note that 63 represents the connection between pins B and C on the UTP
            if (self.wait_for_request.result != -1 and self.wait_for_request.wait_for == 63
                    and self.wait_for_request.result == self.wait_for_request.expected):
                self.node.get_logger().info(
                    f"Tool {'mounted' if self.wait_for_request.expected == 0 else 'unmounted'} \
                        successfully")
                self.wait_for_request.expected = -1
                self.wait_for_request.result = -1
                self.wait_for_request.wait_for = -1
                self.wait_for_request.index = -1
            elif (self.wait_for_request.result != -1 and self.wait_for_request.wait_for == 63
                    and self.wait_for_request.result != self.wait_for_request.expected):
                self.node.get_logger().warn(
                    f"FAILED TOOL \
                        {'MOUNTING' if self.wait_for_request.expected == 0 else 'UNMOUNTING'}!! \
                            Stopping sequence")
                self.wait_for_request.expected = -1
                self.wait_for_request.result = -1
                self.wait_for_request.wait_for = -1
                self.wait_for_request.index = -1
                self.clear_sequence()
                return

            if (self.wait_for_request.result != -1 and self.wait_for_request.wait_for == 59):
                reading: str = f'SoilReading\
                    {self.wait_for_request.index} {self.wait_for_request.result}'
                self.map_cmd_client(cmd=reading)

                self.wait_for_request.expected = -1
                self.wait_for_request.result = -1
                self.wait_for_request.wait_for = -1
                self.wait_for_request.index = -1

            if self.sequence[0] == '':
                self.sequence.pop(0)

            # Move the gantry to the parsed coordinates
            if self.command_type == 'CC':
                coords = self.sequence[0].split(' ')
                self.mvm.move_gantry_abs(x_coord=float(coords[0]),
                                         y_coord=float(coords[1]),
                                         z_coord=float(coords[2]))
                self.sequence.pop(0)
                return
            # Node a servo to the parsed angle
            if self.command_type == 'SC':
                cmd = self.sequence[0].split(' ')
                pin = int(cmd[0])
                angle = float(cmd[1])

                self.devices.move_servo(pin, angle)
                self.sequence.pop(0)
            # Manipulate the device as indicated in the command
            elif self.command_type == 'DC':
                cmd = self.sequence[0].split(' ')
                # Checking if a tool was mounted properly
                if cmd[0] == 'CHECK':
                    self.devices.read_pin(63, False)
                    self.wait_for_request.wait_flag = True
                    self.wait_for_request.wait_for = 63
                    self.wait_for_request.expected = int(cmd[1])
                    self.wait_for_request.index = -1
                if cmd[0] == 'READSOIL':
                    self.devices.read_pin(59, True)
                    self.wait_for_request.wait_flag = True
                    self.wait_for_request.wait_for = 59
                    self.wait_for_request.expected = -1
                    self.wait_for_request.index = int(cmd[1])
                if cmd[0] == 'Vacuum':
                    if cmd[1] in ['0', '1']:
                        self.vacuum_pump(state=int(cmd[1]))
                    else:
                        self.node.get_logger().warn('Vacuum pump command has a state other \
                                                    than on or off. Command ignored!')
                if cmd[0] == 'WaterPulses':
                    if int(cmd[1]):
                        self.water_pulses(delay=int(cmd[1]))
                if cmd[0] == 'P4_Pulses':
                    if int(cmd[1]):
                        self.peripheral4_pulses(delay=int(cmd[1]))
                self.sequence.pop(0)
            # Handle Vision Commands
            elif self.command_type == 'VC':
                if not len(self.sequence):
                    return
                cmd = self.sequence[0].split(' ')
                # Vision command
                if cmd[0] == 'CALIB':
                    self.cam_calib_client(cmd=(self.sequence[0] + ' ' + str(self.x) + ' '
                                               + str(self.y) + ' ' + str(self.z)))
                elif cmd[0] == 'PAN':
                    self.stitch_panorama_client(calib=False, update_map=False, mosaic=False,
                                                detect_weeds=False, x=self.x,
                                                y=self.y, z=self.z)
                elif cmd[0] == 'MOSAIC':
                    self.stitch_panorama_client(calib=False, update_map=False, mosaic=True,
                                                detect_weeds=False, x=self.x,
                                                y=self.y, z=self.z, num=int(cmd[1]))
                elif cmd[0] == 'M_CAM_TAKE':
                    self.macro_client(topic='multicam_toggle', info='TAKE')
                self.sequence.pop(0)
            # The amount of ticks the farmbot should wait in the sequence
            # before moving to the next command
            elif self.command_type == 'TD':
                self.ticks = int(self.sequence[0][1:])
                self.sequence.pop(0)

    def uart_message(self, msg: str):
        """Get the responses to the requests done in the sequencer."""
        if ' ' in msg:
            info = msg.split(' ')
            if info[0] == 'R41' and info[1] == f'P{str(self.wait_for_request.wait_for)}':
                self.wait_for_request.result = int(info[2][1:])
                self.wait_for_request.wait_flag = False

    def macro_client(self, topic: str, info: str):
        """Use to implement experimental features."""
        # Set the wait flag
        self.general_wait_flag = True

        # Initializing the client and wait for map server confirmation
        client = self.node.create_client(StringRepReq, topic)
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn(f'Waiting for Server with topic /{topic}...')

        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = info

        # Call async and add the response callback
        future = client.call_async(request=request)
        future.add_done_callback(self.cmd_sequence_callback)

    def stitch_panorama_client(self, detect_weeds: bool, calib: bool, update_map: bool,
                               mosaic: bool, x: float, y: float, z: float, num=int(-1)):
        """
        Service client used to communicate between the farmbot controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        """
        # Block sequencing here async
        self.wait_for_camera = True

        if sum([calib, update_map, mosaic]) > 1:
            self.node.get_logger().warn('Cannot have more than 1 command \
                                        type sent at the same time!')

        # Initializing the client and wait for map server confirmation
        client = self.node.create_client(StringRepReq, 'form_panorama')
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn('Waiting for Camera Stitching Server...')

        # Set the command to the service request
        request = StringRepReq.Request()

        if calib:
            request.data = 'CALIB'
        elif update_map:
            request.data = 'MAP ' + str(x) + ' ' + str(y)
        if detect_weeds:
            request.data = 'DETECT_WEEDS' + ' ' + str(x) + ' ' + str(y)
        elif mosaic:
            request.data = 'MOSAIC ' + str(num)
        else:
            request.data = str(x) + ' ' + str(y) + ' ' + str(z)

        # Call async and add the response callback
        future = client.call_async(request=request)
        future.add_done_callback(self.stitch_callback)

    def stitch_callback(self, future):
        """Camera Service Server callback."""
        if future.result().data == 'FAILED':
            self.clear_sequence()

        self.wait_for_camera = False

    def cam_calib_client(self, cmd: str):
        """
        Service client used to communicate between the farmbot controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        """
        if not cmd:
            self.node.get_logger().warn('Calibration command type not set! Command ignored')
            return

        # Initializing the client and wait for map server confirmation
        client = self.node.create_client(StringRepReq, 'camera_calibration')
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn('Waiting for Camera Calibration Server...')

        # Set the command to the service request
        request = StringRepReq.Request()
        request.data = cmd

        # Call async and add the response callback
        future = client.call_async(request=request)
        future.add_done_callback(self.cmd_sequence_callback
                                 if cmd == 'GET' else self.stitch_callback)

    def panorama_client(self, mosaic=False):
        """
        Service client used to communicate between the farmbot controller and the map handler.

        Args:
            cmd {str}: The command that is sent to the map handler
        """
        # Initializing the client and wait for map server confirmation
        client = self.node.create_client(StringRepReq, 'panorama_sequence')
        while not client.wait_for_service(1.0):
            self.node.get_logger().warn('Waiting for Panorama Sequencing Server...')

        # Set the command to the service request
        request = StringRepReq.Request()
        if mosaic:
            request.data = 'MOSAIC'
        else:
            request.data = 'ADD HERE ANY SETUP THAT MIGHT CHANGE'

        # Call async and add the response callback
        future = client.call_async(request=request)
        future.add_done_callback(self.cmd_sequence_callback)

    def status_callback(self, state: Bool):
<<<<<<< HEAD
        """Transmit the busy statefrom the Uart controller to the sequencer timer."""
        self.farmbot_busy = state.data
=======
        """Callback from the UART Handler that transmits the busy state of the farmbot."""
        self.farmbot_estop = state.data
>>>>>>> 707544a (Modification to the busy_state topic so that it is used only for estop. Related modification in the sequencer.)
