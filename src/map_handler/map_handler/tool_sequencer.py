from rclpy.node import Node

class ToolDetails:
    '''
    Object used to represent all the details representing a tool that are needed
    for sequencing the unmounting and mounting actions
    '''
    def __init__(self, x_pos = 0.0, y_pos = 0.0, z_pos = 0.0, z_safe_inc = 0.0, release_dir = 0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.z_safe_inc = z_safe_inc
        self.release_dir = release_dir

class ToolExchanger:
    '''
    ROS2 Python module that creates the farmbot tool mounting and unmounting sequences
    '''
    def __init__(self, node:  Node, map_max_x = 3000.0, map_max_y = 1500.0, map_max_z = 300.0):
        self.node_ = node
        self.map_max_x = map_max_x
        self.map_max_y = map_max_y
        self.map_max_z = map_max_z

    def mount_tool(self, cmd: ToolDetails):
        '''
        Forming the tool mounting sequence based on the tool details

        Args:
            cmd {ToolDetails}: Contains all the tool information
        '''
        # Check the tool information for any possible errors
        release_x_inc, release_y_inc = self.__get_release_direction(cmd.release_dir)
        if not self.__check_tool_details(cmd, release_x_inc, release_y_inc):
            return 'FAILED'
        
        # Command Identifier
        # CC - Coordinate Command
        # T - Tool command set
        # x - unspecified tool set
        # 1 - tool mount
        cmd_seq = 'CC_T_x_1\n'

        # go to tool position at a safe z distance over it
        cmd_seq += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos + cmd.z_safe_inc}\n"
        # lower the z axis until the exact tool mounting position
        cmd_seq += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos}\n"
        # move towards the release position
        cmd_seq += f"{cmd.x_pos + release_x_inc} {cmd.y_pos + release_y_inc} {cmd.z_pos}\n"
        # raise the tool head to a safe z-axis value
        cmd_seq += f"{cmd.x_pos + release_x_inc} {cmd.y_pos + release_y_inc} {cmd.z_pos + cmd.z_safe_inc}\n"
        # check if tool was mounted properly
        cmd_seq += 'DC_T_x_1\n'
        cmd_seq += 'CHECK 0'

        return cmd_seq
    
    def unmount_tool(self, cmd: ToolDetails):
        '''
        Forming the tool unmounting sequence based on the tool details

        Args:
            cmd {ToolDetails}: Contains all the tool information
        '''
        # Check the tool information for any possible errors
        release_x_inc, release_y_inc = self.__get_release_direction(cmd.release_dir)
        if not self.__check_tool_details(cmd, release_x_inc, release_y_inc):
            return 'FAILED'
        
        # Command Identifier
        # CC - Coordinate Command
        # T - Tool command set
        # x - unspecified tool set
        # 2 - tool mount
        cmd_seq = 'CC_T_x_2\n'

        # move over the release position
        cmd_seq += f"{cmd.x_pos + release_x_inc} {cmd.y_pos + release_y_inc} {cmd.z_pos + cmd.z_safe_inc}\n"
        # lower towards the release position
        cmd_seq += f"{cmd.x_pos + release_x_inc} {cmd.y_pos + release_y_inc} {cmd.z_pos}\n"
        # move to the tool's home position
        cmd_seq += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos}\n"
        # raise the z axis to the safe z distance
        cmd_seq += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos + cmd.z_safe_inc}\n"
        # check if tool was unmounted properly
        cmd_seq += 'DC_T_x_2\n'
        cmd_seq += 'CHECK 1'

        return cmd_seq
    
    def __get_release_direction(self, dir: int):
        '''
        Gets the coordinate increments for the release of each tool based 
        on the mounting orientation
        '''
        if dir < 1 or dir > 4: 
            self.node_.get_logger().error('Release direction for the tool unrecognized! Check configuration!')
            return
        if dir == 1:
            return -100.0, 0.0
        if dir == 1:
            return 100.0, 0.0
        if dir == 1:
            return 0.0, -100.0
        if dir == 1:
            return 0.0, 100.0

    def __check_tool_details(self, cmd: ToolDetails, x_inc: float, y_inc: float):
        '''
        Checks if the tool position is reachable and valid
        '''
        # Check if the tool position is reachable
        if not self.__outside_bounds(x_min = 0.0, x_max = self.map_max_x, y_min = 0.0, y_max = self.map_max_y, 
                                     z_min = self.map_max_z, z_max = 0.0, x = cmd.x_pos, y = cmd.y_pos, z = cmd.z_pos):
            self.node_.get_logger().warn(f"Max pos {self.map_max_x}  {self.map_max_y}  {self.map_max_z} ")
            self.node_.get_logger().warn(f"Tool home position {cmd.x_pos} {cmd.y_pos} {cmd.z_pos} is outside of the farmbot's reach!")
            return False
        # Check if the release position is valid
        if not self.__outside_bounds(x_min = 0.0, x_max = self.map_max_x, y_min = 0.0, y_max = self.map_max_y, z_min = self.map_max_z, z_max = 0.0,
                                     x = cmd.x_pos + x_inc, y = cmd.y_pos + y_inc, z = cmd.z_pos):
            self.node_.get_logger().warn(f"Tool release position {cmd.x_pos + x_inc} {cmd.y_pos + y_inc} {cmd.z_pos} is outside of the farmbot's reach!")
            return False
        
        return True
            
    def __outside_bounds(self, x_min: float, x_max: float, y_min: float, y_max: float,
                         z_min: float, z_max: float, x: float, y: float, z: float):
        '''
        Checks if a position (x, y, z) is within the ranges set for each axis
        '''
        if (x >= x_min and x <= x_max) and (y >= y_min and y <= y_max) and (z >= z_min and z <= z_max):
            return True
        return False