#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, HomeCommand, ParameterCommand, StateCommand
from std_msgs.msg import String
from farmbot_interfaces.msg import ParameterList, MapCommand, PlantManage
from farmbot_interfaces.srv import ParameterConfig, LoadParamConfig, StringRepReq

import time

class KeyboardTeleOp(Node):
    # Node contructor
    def __init__(self):
        super().__init__("FarmBotController")

        # Memory
        self.cur_x_ = 0.0
        self.cur_y_ = 0.0
        self.cur_z_ = 0.0

        self.len_x_ = 1000.0
        self.len_y_ = 1000.0
        self.len_z_ = 1000.0

        self.X_MAX_SPEED = 800.0
        self.Y_MAX_SPEED = 800.0
        self.Z_MAX_SPEED = 1000.0

        # Variables used to create the commands
        self.gantryConf_ = HomeCommand()    # Used for gantry configuration (homing, calibration)
        self.gantryMove_ = GantryCommand()  # Used for moving the gantry along the 3 axis
        self.state_ = StateCommand()
        self.paramHandler_ = ParameterCommand() # Used for reading and writing to the FarmBot Parameters

        # Temporary Keyboard subscriber
        self.cur_increment_ = 10.0
        self.inputSub_ = self.create_subscription(String, 'input_topic', self.commandInterpretationCallback, 10)

        # UART Rx Subscriber
        self.uartRxSub_ = self.create_subscription(String, 'uart_receive', self.farmbotFeedbackCallback, 10)

        # Control publishers
        self.gantryMovePub_ = self.create_publisher(GantryCommand, 'move_gantry', 10)
        self.gantryConfPub_ = self.create_publisher(HomeCommand, 'home_handler', 10)
        self.statePub_ = self.create_publisher(StateCommand, 'state_command', 10)
        self.paramCmdPub_ = self.create_publisher(ParameterCommand, 'parameter_command', 10)

        # Map publishers
        self.plantMngPub_ = self.create_publisher(PlantManage, 'plant_mng', 10)

        # Log the initialization
        self.get_logger().info("Farmbot Controller Initialized..")

    def commandInterpretationCallback(self, cmd = String):
        match cmd.data:
            case 'e':
                self.electronicStop()
            case 'E':
                self.resetElectronicStop()
            case 'w':
                self.cur_x_ += self.cur_increment_
                self.moveGantryAbsolute(x_coord = self.cur_x_ + self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'a':
                self.cur_y_ -= self.cur_increment_
                self.moveGantryAbsolute(x_coord = self.cur_x_, y_coord = self.cur_y_ - self.cur_increment_, z_coord = self.cur_z_)
            case 's':
                self.cur_x_ -= self.cur_increment_
                self.moveGantryAbsolute(x_coord = self.cur_x_ - self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'd':
                self.cur_y_ += self.cur_increment_
                self.moveGantryAbsolute(x_coord = self.cur_x_, y_coord = self.cur_y_ + self.cur_increment_, z_coord = self.cur_z_)
            case '1':
                self.cur_increment_ = 10.0
            case '2':
                self.cur_increment_ = 100.0
            case '3':
                self.cur_increment_ = 500.0
            case 'i':
                self.configLoaderClient(standard = False, ver = "labFB")
            case 'h': 
                self.goHome()
            case 'j':
                self.findAxisHome(x = True)
            case 'k':
                self.findAxisHome(y = True)
            case 'l':
                self.findAxisHome(z = True)
            case 'fh':
                self.findAllHomes()
            case 'c':
                self.calibrateAllLens()
            case 'v':
                self.calibrateAxisLen(x = True)
            case 'b':
                self.calibrateAxisLen(y = True)
            case 'n':
                self.calibrateAxisLen(z = True)
            case 'o':
                self.parameterConfigClient(cmd = 'SAVE')
            case 'p':
                self.parameterConfigClient(cmd = 'MAP')
            case 'T10': # new tool marked
                self.tool_exchange_client(cmd = "T_1_0\nSeeder\n1198.0 332.4 -240.0 1")
            case 'T11':
                self.tool_exchange_client(cmd = "T_1_1")
            case 'T12':
                self.tool_exchange_client(cmd = "T_1_2")
            case 'T20': # new tool marked
                self.tool_exchange_client(cmd = "T_2_0\nSoil\n1198.0 432.2 -240.0 1")
            case 'T21':
                self.tool_exchange_client(cmd = "T_2_1")
            case 'T22':
                self.tool_exchange_client(cmd = "T_2_2")
            case 'T30': # new tool marked
                self.tool_exchange_client(cmd = "T_3_0\nWater\n1198.0 532.2 -240.0 1")
            case 'T31':
                self.tool_exchange_client(cmd = "T_3_1")
            case 'T32':
                self.tool_exchange_client(cmd = "T_3_2")

    ## Tool Handling Client
    def tool_exchange_client(self, cmd = str):
        client = self.create_client(StringRepReq, 'map_cmd')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Parameter Loading Server...")
        
        request = StringRepReq.Request()
        request.data = cmd

        future = client.call_async(request = request)
        future.add_done_callback(self.tool_cmd_sequence_callback)

    def tool_cmd_sequence_callback(self, future):
        #self.get_logger().info(f"AAAAAAAAAAAA")
        #self.get_logger().info(future.result().data)
        
        cmd = future.result().data.split('\n')
        if cmd[0][:2] == 'CC':
            for mvm in cmd[1:]:
                coords = mvm.split(' ')
                self.moveGantryAbsolute(x_coord = float(coords[0]), 
                                        y_coord = float(coords[1]), 
                                        z_coord = float(coords[2]))


    ## UART Handling Callback
    def farmbotFeedbackCallback(self, msg = String):
        msgSplit = (msg.data).split(' ')
        reportCode = msgSplit[0]
        #if reportCode == 'R21' or reportCode == 'R23':
        #    self.parameterConfigClient(cmd = msg.data)
        if reportCode == 'R82':
            self.cur_x_ = float(msgSplit[1][1:])
            self.cur_y_ = float(msgSplit[2][1:])
            self.cur_z_ = float(msgSplit[3][1:])
        
    
    ## Parameter Loading Service Client
    def configLoaderClient(self, standard = True, ver = ""):
        if not standard and ver == "":
            self.get_logger().warn("Can't set non standard configuration if the version of the farmbot is not set!")
        
        client = self.create_client(LoadParamConfig, 'load_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Parameter Loading Server...")
        
        request = LoadParamConfig.Request()
        request.standard = standard
        request.ver = ver

        future = client.call_async(request = request)
        future.add_done_callback(self.callbackConfigLoading)

    def callbackConfigLoading(self, future):
        try:
            response = future.result()
            if not response:
                self.get_logger().warn("Failure in Parameter Config Loading!")
            
            ## TODO: Add a response to the future
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e, ))

    ## Parameter Config Service Client
    def parameterConfigClient(self, cmd = String):
        client = self.create_client(ParameterConfig, 'manage_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Parameter Config Server...")

        request = ParameterConfig.Request()
        request.data = cmd

        future = client.call_async(request = request)
        future.add_done_callback(self.callbackParamConfig)

    def callbackParamConfig(self, future):
        try:
            response = future.result()
            if not response:
                self.get_logger().warn("Failure in Parameter Config Handling!")
            
            ## TODO: Add a response to the future

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e, ))

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

    def defineState(self, estop = False, abort_movement = False, reset_estop = False,\
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

    ## Calibration and Homing Functions

    def goHome(self):
        """
        Goes to the home position on each axis
        """
        self.manipulateConfig(allHome = True)

    def findAllHomes(self):
        """
        Finds the home for all the axis on the farmbot.
        """
        self.findAxisHome(x = True, y = False, z = False)
        self.findAxisHome(x = False, y = True, z = False)
        self.findAxisHome(x = False, y = False, z = True)

    def findAxisHome(self, x = False, y = False, z = False):
        """Homes the selected axis on the farmbot. For example, for homing x and y you set them to True.

        Args:
            x {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            y {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            z {Bool}: True if the X-Axis should have it's home position found. Defaults to False
        """
        self.manipulateConfig(x_axis = x, y_axis = y, z_axis = z)

    def calibrateAllLens(self):
        """
        Calibrates the lengths for all the axis on the farmbot.
        """
        self.calibrateAxisLen(x = True, y = True, z = True)

    def calibrateAxisLen(self, x = False, y = False, z = False):
        """Calibrates the selected axis on the farmbot. For example, for calibrating the length along the x and y axis, you set them to True.

        Args:
            x {Bool}: True if the X-Axis is to be calibrated. Defaults to False
            y {Bool}: True if the Y-Axis is to be calibrated. Defaults to False
            z {Bool}: True if the Z-Axis is to be calibrated. Defaults to False
        """
        self.manipulateConfig(calibrate = True, x_axis = x, y_axis = y, z_axis = z)

    def setCurrentPosHomeAll(self):
        """
        Sets the current position of the extruder as the home position
        """
        self.setAxisCurrentPosHome(x = True, y = True, z = True)
    
    def setAxisCurrentPosHome(self, x = False, y = False, z = False):
        """Sets the current position of the extruder as the home position for the selected axis.

        Args:
            x {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
            y {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
            z {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
        """
        self.manipulateConfig(set_this_home = True, x_axis = x, y_axis = y, z_axis = z)

    def manipulateConfig(self, allHome = False, set_this_home = False, calibrate = False, x_axis = False, y_axis = False, z_axis = False):
        """
        Creates the command that is to be handled and translated to the FarmBot specific commands for homing and calibrating the robot.

        Note:
            The if you want to FIND HOME, you should set allHome and calibrate to False, followed by the axis you want the find home operation to work on.

        Args:
            allHome {Bool}: If it is True, the gantry goes to home (Note this is GO home not FIND home). Defaults to False 
            set_this_home {Bool}: Determines if the extruder's position should be set as the home. Defaults to False
            calibrate {Bool}: Determines if the command will be taken as a homing or a calibration command. 
                                If it is False, it Homes; if it is True, it calibrates. Defaults to False
            x_axis {Bool}: Specifies wheather the command manipulates the X-Axis. Defaults to False
            y_axis {Bool}: Specifies wheather the command manipulates the Y-Axis. Defaults to False
            z_axis {Bool}: Specifies wheather the command manipulates the Z-Axis. Defaults to False
        """
        self.gantryConf_.go_home = allHome
        self.gantryConf_.current_pos_home = set_this_home
        self.gantryConf_.calib = calibrate
        self.gantryConf_.x = x_axis
        self.gantryConf_.y = y_axis
        self.gantryConf_.z = z_axis

        self.gantryConfPub_.publish(self.gantryConf_)

    ## Gantry Movement Functions

    def moveGantryAbsolute(self, x_coord = float, y_coord = float, z_coord = float):
        """
        Moves the Gantry at max speed to the desired coordinates

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
        """
        self.moveGantry(x_coord = x_coord, y_coord = y_coord, z_coord = z_coord,\
                        x_speed = 100.0, y_speed = 100.0, z_speed = 100.0)

    def moveGantry(self, x_coord = float, y_coord = float, z_coord = float, speed = float):
        """
        Moves the Gantry to the desired coordinates at the the speed specified

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
            speed {float}: Desired Speed for all the axis in percent format (0 - lowest speed, 100 - highest speed)
        """
        self.moveGantry(x_coord = x_coord, y_coord = y_coord, z_coord = z_coord, x_speed = speed, y_speed = speed, z_speed = speed)
    
    def moveGantry(self, x_coord = float, y_coord = float, z_coord = float, x_speed = float, y_speed = float, z_speed = float):
        """
        Moves the Gantry to the desired coordinates at the specified per axis speed

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
            x_speed {float}: Desired Speed for X-Axis in percent format (0 - lowest speed, 100 - highest speed)
            y_speed {float}: Desired Speed for Y-Axis in percent format (0 - lowest speed, 100 - highest speed)
            z_speed {float}: Desired Speed for Z-Axis in percent format (0 - lowest speed, 100 - highest speed)
        """
        self.manipulateMovement(movementMode = False, x_coord = x_coord, y_coord = y_coord, z_coord = z_coord,\
                                x_speed = x_speed / 100.0 * self.X_MAX_SPEED,\
                                y_speed = y_speed / 100.0 * self.Y_MAX_SPEED,\
                                z_speed = z_speed / 100.0 * self.Z_MAX_SPEED)
        
    def manipulateMovement(self, movementMode = False, x_coord = float, y_coord = float, z_coord = float, x_speed = float, y_speed = float, z_speed = float):
        """
        Creates the command that is to be handled and translated to the FarmBot specific commands for moving the gantry along the axis.

        Args:
            movementMode {Bool}: The movement mode. False - standard, True - with interpolation (needs constant feed rate). Defaults to False
            x_coord {Int}: The desired x-axis coordinate relative to home
            y_coord {Int}: The desired y-axis coordinate relative to home
            z_coord {Int}: The desired z-axis coordinate relative to home
            x_speed {Int}: The speed used to reach the x coordinate
            y_speed {Int}: The speed used to reach the y coordinate
            z_speed {Int}: The speed used to reach the z coordinate
        """
        self.gantryMove_.mode = movementMode
        self.gantryMove_.x = x_coord
        self.gantryMove_.y = y_coord
        self.gantryMove_.z = z_coord
        self.gantryMove_.a = x_speed
        self.gantryMove_.b = y_speed
        self.gantryMove_.c = z_speed

        self.gantryMovePub_.publish(self.gantryMove_)
    
    ## Parameter Handling Commands

    def readParam(self, param = int):
        """
        Read the value on parameter {param}.

        Args:
            param {Int}: Parameter in question
        """
        self.parameterHandler(list = False, write = False, read = True, update = False, param = param)

    def listAllParams(self):
        """
        List all the parameters and their values
        """
        self.parameterHandler(list = True, write = False, read = False, update = False)

    def writeParam(self, param = int, value = int):
        """
        Write {value} to parameter {param}

        Args:
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        """
        self.parameterHandler(list = False, write = True, read = False, update = False, param = param, value = value)
    
    def updateParam(self, param = int, value = int):
        """
        Update parameter {param} with {value}.

        Args:
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        """
        self.parameterHandler(list = False, write = False, read = False, update = True, param = param, value = value)

    def parameterHandler(self, list = bool, write = bool, read = bool, update = bool, param = int, value = int):
        """
        Function for parameter handling commands.

        Args:
            list {bool}: If true, all the parameters will be listed
            write {bool}: If true, value V will be written to parameter P
            read {bool}: If true, parameter P will be listed
            update {bool}: If true, parameter P will be updated with value V (e.g. during calib.)
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        """
        self.paramHandler_.list = list
        self.paramHandler_.write = write
        self.paramHandler_.read = read
        self.paramHandler_.update = update
        self.paramHandler_.param = param
        self.paramHandler_.value = value

        self.paramCmdPub_.publish(self.paramHandler_)

def main(args = None):
    rclpy.init(args = args)

    keyboardTeleOpNode = KeyboardTeleOp()
    
    try:
        rclpy.spin(keyboardTeleOpNode)
    except KeyboardInterrupt:
        keyboardTeleOpNode.destroy_node()

    keyboardTeleOpNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()