from rclpy.node import Node
from farmbot_interfaces.msg import GantryCommand, HomeCommand

class Movement:
    '''
    Movement module that extends the farmbot controller node
    '''
    def __init__(self, node: Node):
        '''
        Module constructor

        Args:
            node {Node}: the node the module extends
        '''
        self.node_ = node

        self.X_MAX_SPEED = 800.0
        self.Y_MAX_SPEED = 800.0
        self.Z_MAX_SPEED = 1000.0

        self.gantry_config_ = HomeCommand()    # Used for gantry configuration (homing, calibration)
        self.move_gantry_ = GantryCommand()  # Used for moving the gantry along the 3 axis

        self.gantry_mvm_pub_ = self.node_.create_publisher(GantryCommand, 'move_gantry', 10)
        self.gantry_config_pub_ = self.node_.create_publisher(HomeCommand, 'home_handler', 10)

    ## Calibration and Homing Functions

    def go_home(self):
        '''
        Goes to the home position on each axis
        '''
        self.manip_mvm_config(all_home = True)

    def find_all_homes(self):
        '''
        Finds the home for all the axis on the farmbot.
        '''
        self.find_axis_home(x = True, y = False, z = False)
        self.find_axis_home(x = False, y = True, z = False)
        self.find_axis_home(x = False, y = False, z = True)

    def find_axis_home(self, x = False, y = False, z = False):
        '''Homes the selected axis on the farmbot. For example, for homing x and y you set them to True.

        Args:
            x {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            y {Bool}: True if the X-Axis should have it's home position found. Defaults to False
            z {Bool}: True if the X-Axis should have it's home position found. Defaults to False
        '''
        self.manip_mvm_config(x_axis = x, y_axis = y, z_axis = z)

    def calibrate_all_axis(self):
        '''
        Calibrates the lengths for all the axis on the farmbot.
        '''
        self.calibrate_axis(x = True, y = True, z = True)

    def calibrate_axis(self, x = False, y = False, z = False):
        '''Calibrates the selected axis on the farmbot. For example, for calibrating the length along the x and y axis, you set them to True.

        Args:
            x {Bool}: True if the X-Axis is to be calibrated. Defaults to False
            y {Bool}: True if the Y-Axis is to be calibrated. Defaults to False
            z {Bool}: True if the Z-Axis is to be calibrated. Defaults to False
        '''
        self.manip_mvm_config(calibrate = True, x_axis = x, y_axis = y, z_axis = z)

    def set_curr_to_home(self):
        '''
        Sets the current position of the extruder as the home position
        '''
        self.set_curr_axis_home(x = True, y = True, z = True)
    
    def set_curr_axis_home(self, x = False, y = False, z = False):
        '''
        Sets the current position of the extruder as the home position for the selected axis.

        Args:
            x {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
            y {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
            z {Bool}: True if this axis' position should be set as the axis' home. Defaults to False
        '''
        self.manip_mvm_config(set_this_home = True, x_axis = x, y_axis = y, z_axis = z)

    def manip_mvm_config(self, all_home = False, set_this_home = False, calibrate = False, x_axis = False, y_axis = False, z_axis = False):
        '''
        Creates the command that is to be handled and translated to the FarmBot specific commands for homing and calibrating the robot.

        Note:
            The if you want to FIND HOME, you should set all_home and calibrate to False, followed by the axis you want the find home operation to work on.

        Args:
            all_home {Bool}: If it is True, the gantry goes to home (Note this is GO home not FIND home). Defaults to False 
            set_this_home {Bool}: Determines if the extruder's position should be set as the home. Defaults to False
            calibrate {Bool}: Determines if the command will be taken as a homing or a calibration command. 
                                If it is False, it Homes; if it is True, it calibrates. Defaults to False
            x_axis {Bool}: Specifies wheather the command manipulates the X-Axis. Defaults to False
            y_axis {Bool}: Specifies wheather the command manipulates the Y-Axis. Defaults to False
            z_axis {Bool}: Specifies wheather the command manipulates the Z-Axis. Defaults to False
        '''
        self.gantry_config_.go_home = all_home
        self.gantry_config_.current_pos_home = set_this_home
        self.gantry_config_.calib = calibrate
        self.gantry_config_.x = x_axis
        self.gantry_config_.y = y_axis
        self.gantry_config_.z = z_axis

        self.gantry_config_pub_.publish(self.gantry_config_)

    ## Gantry Movement Functions

    def move_gantry_abs(self, x_coord = float, y_coord = float, z_coord = float):
        '''
        Moves the Gantry at max speed to the desired coordinates

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
        '''
        self.move_gantry(x_coord = x_coord, y_coord = y_coord, z_coord = z_coord,\
                        x_speed = 100.0, y_speed = 100.0, z_speed = 100.0)

    def move_gantry(self, x_coord = float, y_coord = float, z_coord = float, speed = float):
        '''
        Moves the Gantry to the desired coordinates at the the speed specified

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
            speed {float}: Desired Speed for all the axis in percent format (0 - lowest speed, 100 - highest speed)
        '''
        self.move_gantry(x_coord = x_coord, y_coord = y_coord, z_coord = z_coord, x_speed = speed, y_speed = speed, z_speed = speed)
    
    def move_gantry(self, x_coord = float, y_coord = float, z_coord = float, x_speed = float, y_speed = float, z_speed = float):
        '''
        Moves the Gantry to the desired coordinates at the specified per axis speed

        Args:
            x_coord {float}: Desired X-Coordinate to move to
            y_coord {float}: Desired Y-Coordinate to move to.
            z_coord {float}: Desired Z-Coordinate to move to.
            x_speed {float}: Desired Speed for X-Axis in percent format (0 - lowest speed, 100 - highest speed)
            y_speed {float}: Desired Speed for Y-Axis in percent format (0 - lowest speed, 100 - highest speed)
            z_speed {float}: Desired Speed for Z-Axis in percent format (0 - lowest speed, 100 - highest speed)
        '''
        self.manip_movement(mode = False, x_coord = x_coord, y_coord = y_coord, z_coord = z_coord,\
                                x_speed = x_speed / 100.0 * self.X_MAX_SPEED,\
                                y_speed = y_speed / 100.0 * self.Y_MAX_SPEED,\
                                z_speed = z_speed / 100.0 * self.Z_MAX_SPEED)
        
    def manip_movement(self, mode = False, x_coord = float, y_coord = float, z_coord = float, x_speed = float, y_speed = float, z_speed = float):
        '''
        Creates the command that is to be handled and translated to the FarmBot specific commands for moving the gantry along the axis.

        Args:
            mode {Bool}: The movement mode. False - standard, True - with interpolation (needs constant feed rate). Defaults to False
            x_coord {Int}: The desired x-axis coordinate relative to home
            y_coord {Int}: The desired y-axis coordinate relative to home
            z_coord {Int}: The desired z-axis coordinate relative to home
            x_speed {Int}: The speed used to reach the x coordinate
            y_speed {Int}: The speed used to reach the y coordinate
            z_speed {Int}: The speed used to reach the z coordinate
        '''
        self.move_gantry_.mode = mode
        self.move_gantry_.x = x_coord
        self.move_gantry_.y = y_coord
        self.move_gantry_.z = z_coord
        self.move_gantry_.a = x_speed
        self.move_gantry_.b = y_speed
        self.move_gantry_.c = z_speed

        self.gantry_mvm_pub_.publish(self.move_gantry_)