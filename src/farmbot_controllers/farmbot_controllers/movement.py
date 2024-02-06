from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, HomeCommand

class Movement:
    def __init__(self, node: Node):
        self.node = node

        self.X_MAX_SPEED = 800.0
        self.Y_MAX_SPEED = 800.0
        self.Z_MAX_SPEED = 1000.0

        self.gantryConf_ = HomeCommand()    # Used for gantry configuration (homing, calibration)
        self.gantryMove_ = GantryCommand()  # Used for moving the gantry along the 3 axis

        self.gantryMovePub_ = self.node.create_publisher(GantryCommand, 'move_gantry', 10)
        self.gantryConfPub_ = self.node.create_publisher(HomeCommand, 'home_handler', 10)

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