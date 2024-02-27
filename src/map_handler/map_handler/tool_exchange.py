from rclpy.node import Node

class ToolDetails:
    def __init__(self, x_pos = 0.0, y_pos = 0.0, z_pos = 0.0, z_safe_inc = 0.0, 
                 release_x_inc = 0, release_y_inc = 0):
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.z_pos = z_pos
        self.z_safe_inc = z_safe_inc
        self.release_x_inc = release_x_inc
        self.release_y_inc = release_y_inc

class ToolExchanger:
    def __init__(self, node =  Node, map_max_x = 3000.0, map_max_y = 1500.0, map_max_z = 300.0):
        self.node = node
        self.map_max_x = map_max_x
        self.map_max_y = map_max_y
        self.map_max_z = map_max_z

    def mount_tool(self, cmd = ToolDetails):
        # Check the tool information for any possible errors
        if not self.__check_tool_details(cmd):
            return "FAILED"
        
        # Command Identifier
        # CC - Coordinate Command
        # T - Tool command set
        # x - unspecified tool set
        # 1 - tool mount
        commandSequence = "CC_T_x_1\n"

        # go to tool position at a safe z distance over it
        commandSequence += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos + cmd.z_safe_inc}\n"
        # lower the z axis until the exact tool mounting position
        commandSequence += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos}\n"
        # move towards the release position
        commandSequence += f"{cmd.x_pos + cmd.release_x_inc} {cmd.y_pos + cmd.release_y_inc} {cmd.z_pos}\n"
        # raise the tool head to a safe z-axis value
        commandSequence += f"{cmd.x_pos + cmd.release_x_inc} {cmd.y_pos + cmd.release_y_inc} {cmd.z_pos + cmd.z_safe_inc}"
        # check if tool was mounted properly
        commandSequence += 'CHECK 0'

        return commandSequence
    
    def unmount_tool(self, cmd = ToolDetails):
        # Check the tool information for any possible errors
        if not self.__check_tool_details(cmd):
            return "FAILED"
        
        # Command Identifier
        # CC - Coordinate Command
        # T - Tool command set
        # x - unspecified tool set
        # 2 - tool mount
        commandSequence = "CC_T_x_2\n"

        # move over the release position
        commandSequence += f"{cmd.x_pos + cmd.release_x_inc} {cmd.y_pos + cmd.release_y_inc} {cmd.z_pos + cmd.z_safe_inc}\n"
        # lower towards the release position
        commandSequence += f"{cmd.x_pos + cmd.release_x_inc} {cmd.y_pos + cmd.release_y_inc} {cmd.z_pos}\n"
        # move to the tool's home position
        commandSequence += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos}\n"
        # raise the z axis to the safe z distance
        commandSequence += f"{cmd.x_pos} {cmd.y_pos} {cmd.z_pos + cmd.z_safe_inc}"
        # check if tool was unmounted properly
        commandSequence += 'CHECK 1'

        return commandSequence
    
    def __check_tool_details(self, cmd = ToolDetails):
        # Check if the tool position is reachable
        if not self.__outside_bounds(x_min = 0.0, x_max = self.map_max_x, y_min = 0.0, y_max = self.map_max_y, 
                                     z_min = self.map_max_z, z_max = 0.0, x = cmd.x_pos, y = cmd.y_pos, z = cmd.z_pos):
            self.node.get_logger().warn(f"Max pos {self.map_max_x}  {self.map_max_y}  {self.map_max_z} ")
            self.node.get_logger().warn(f"Tool home position {cmd.x_pos} {cmd.y_pos} {cmd.z_pos} is outside of the farmbot's reach!")
            return False
        # Check if the release position is valid
        if not self.__outside_bounds(x_min = 0.0, x_max = self.map_max_x, y_min = 0.0, y_max = self.map_max_y, z_min = self.map_max_z, z_max = 0.0,
                                     x = cmd.x_pos + cmd.release_x_inc, y = cmd.y_pos + cmd.release_y_inc, z = cmd.z_pos):
            self.node.get_logger().warn(f"Tool release position {cmd.x_pos + cmd.release_x_inc} {cmd.y_pos + cmd.release_y_inc} {cmd.z_pos} is outside of the farmbot's reach!")
            return False
        
        return True
            
    def __outside_bounds(self, x_min, x_max, y_min, y_max, z_min, z_max, x, y, z):
        if (x >= x_min and x <= x_max) and (y >= y_min and y <= y_max) and (z >= z_min and z <= z_max):
            return True
        return False