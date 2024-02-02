import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from farmbot_interfaces.msg import MapCommand, PlantManage
from farmbot_interfaces.srv import StringRepReq

from map_handler.tool_exchange import ToolDetails, ToolExchanger

import os
import yaml
import copy

class MapController(Node):
    def __init__(self):
        super().__init__("MapController")

        self.safe_z_increment_ = 50.0


        # Relevant directory and file names
        self.directory_ = os.path.join(
            get_package_share_directory('map_handler'),
            'config'
        )
        self.plantRefFile_ = 'plant_reference.yaml'
        self.mapRefFile_ = 'map_references.yaml'
        self.activeMap_ = 'active_map.yaml'
        toolRefFile_ = 'tool_reference.yaml'

        # Loading the map instance from memory
        self.map_instance_ = self.retrieveMap(directory = self.directory_,
                                              fileName1 = self.activeMap_,
                                              fileName2 = self.mapRefFile_)
        
        # Loading the tool exhanging module and the tool command object
        self.tool_exchanger_ = ToolExchanger(node = self, 
                                             map_max_x = self.map_instance_['map_reference']['x_len'],
                                             map_max_y = self.map_instance_['map_reference']['y_len'],
                                             map_max_z = -self.map_instance_['map_reference']['z_len'])
        self.tool_details_ = ToolDetails()

        # Loading the plant referencing method
        self.plantReference_ = self.load_from_yaml(self.directory_, self.plantRefFile_)
        # Loading the tool referencing method
        self.tool_reference_ = self.load_from_yaml(self.directory_, toolRefFile_)

        # MapCommand subscriber
        self.mapCmdSub_ = self.create_subscription(MapCommand, 'map_cmd', self.mapCmdCallback, 10)
        # Plant Manager Command Subscriber
        self.plantMngSub_ = self.create_subscription(PlantManage, 'plant_mng', self.plantMngCallback, 10)
        # Map information service server
        self.mapInfoServer_ = self.create_service(StringRepReq, 'map_cmd', self.map_command_server)

        self.get_logger().info("Map Controller Initialized")


    ## Map command managers
    def mapCmdCallback(self, cmd = MapCommand):
        # Check command validity
        sumP = sum([cmd.sort, cmd.update, cmd.reindex])
        if sumP != 1:
            self.get_logger().warn(f"You can only use 1 command at a time! You used {sumP}")
            return
        
        # TODO Add behaviour for each command type
        if cmd.sort:
            pass
        if cmd.reindex:
            pass
        if cmd.update:
            for update in cmd.update_info:
                cmdSplit = update.split(' ')
                # Updating the map dimensions
                match cmdSplit[0]:
                    case 'X': 
                        self.map_instance_['map_reference']['x_len'] = float(cmdSplit[1])
                        self.tool_exchanger_.map_max_x = float(cmdSplit[1])
                    case 'Y': 
                        self.map_instance_['map_reference']['y_len'] = float(cmdSplit[1])
                        self.tool_exchanger_.map_max_y = float(cmdSplit[1])
                    case 'Z': 
                        self.map_instance_['map_reference']['z_len'] = float(cmdSplit[1])
                        self.tool_exchanger_.map_max_z = float(cmdSplit[1])
                    case _: 
                        self.get_logger().warn(f"Command ({update}) not recognized and ignored!")
            
            self.tool_exchanger_ = ToolExchanger(node = self, 
                                                 map_max_x = self.map_instance_['map_reference']['x_len'],
                                                 map_max_y = self.map_instance_['map_reference']['y_len'],
                                                 map_max_z = -self.map_instance_['map_reference']['z_len'])

            self.save_to_yaml(self.map_instance_, self.directory_, self.activeMap_, createIfNotExisting = True)
        if cmd.back_up:
            pass

    ## Plant Managers
    
    def plantMngCallback(self, cmd = PlantManage):
        # Check command validity
        if (cmd.add and cmd.remove) or (not cmd.add and not cmd.remove):
            self.get_logger().warn(f"You must select either to add or remove a plant! Can't do both/none in a commmand")
            return
        
        if cmd.add:
            if cmd.autopos: # TODO: Implement autopositioning
                pass
            else:
                self.addPlant(x = cmd.x, y = cmd.y, z = cmd.z, max_z = cmd.max_z, 
                              exclusion_radius = cmd.exclusion_radius,
                              canipy_radius = cmd.canipy_radius,
                              water_quantity = cmd.water_quantity,
                              plant_name = cmd.plant_name,
                              growth_stage = cmd.growth_stage)
        if cmd.remove:
            self.removePlant(index = cmd.index)

    def addPlant(self, x = float, y = float, z = float, max_z = float, water_quantity = float, exclusion_radius = float, canipy_radius = float, plant_name = "", growth_stage = ""):
        self.plantReference_['identifiers']['plant_name'] = plant_name
        self.plantReference_['position']['x'] = x
        self.plantReference_['position']['y'] = y
        self.plantReference_['position']['z'] = z
        self.plantReference_['plant_details']['plant_radius'] = exclusion_radius
        self.plantReference_['plant_details']['canipy_radius'] = canipy_radius
        self.plantReference_['plant_details']['max_height'] = max_z
        self.plantReference_['status']['growth_stage'] = growth_stage

        plant_id = len(self.map_instance_['plant_details']['plants']) + 1
        self.plantReference_['identifiers']['index'] = plant_id

        self.map_instance_['plant_details']['plant_count'] += 1
        self.map_instance_['plant_details']['plants'].append(self.plantReference_)

        self.save_to_yaml(self.map_instance_, self.directory_, self.activeMap_, createIfNotExisting = True)

    def removePlant(self, index = int):
        plants = self.map_instance_['plant_details']['plants']
        for plant in plants:
            if plant['identifiers']['index'] == index:
                plants.remove(plant)
                self.map_instance_ ['plant_details']['plants'] -= 1
                self.get_logger().info(f"Removed {[plant['identifiers']['name']]} with index {index} from the map")
                break
    
        self.save_to_yaml(self.map_instance_, self.directory_, self.activeMap_)

    ## Map Info Server

    # TODO: Add functionality
    def map_command_server(self, request, response):
        type = request.data[0]
        # Tool Command Type
        if type == 'T':
            response.data = self.tool_cmd_interpreter(request.data)
        
        return response

    def tool_cmd_interpreter(self, msg = str):  
        elem = msg.split('_')
        index = elem[1]
        cmd = int(elem[2][0])

        # Setting up a new tool
        if cmd == 0:
            self.add_tool(msg, index)
            return "T_x_0 SUCCESS"
        if cmd == 1 or cmd == 2:
            self.tool_details_.x_pos = self.map_instance_['map_reference']['tools']['T' + index]['position']['x']
            self.tool_details_.y_pos = self.map_instance_['map_reference']['tools']['T' + index]['position']['y']
            self.tool_details_.z_pos = self.map_instance_['map_reference']['tools']['T' + index]['position']['z']
            self.tool_details_.z_safe_inc = self.safe_z_increment_
            [self.tool_details_.release_x_inc, self.tool_details_.release_y_inc] = \
                    self.get_release_direction(self.map_instance_['map_reference']['tools']['T' + index]['release_dir'])
        
            self.get_logger().info(f"Mounting {self.map_instance_['map_reference']['tools']['T' + index]['name']}")

            if cmd == 1:
                return self.tool_exchanger_.mount_tool(self.tool_details_)
            else:
                return self.tool_exchanger_.unmount_tool(self.tool_details_)
        
        # Check if the tool command request is valid

        # Start the appropriate command
        self.get_logger().warn(f"Unrecognized command {str(msg)}")
        return "UNRECOGNIZED"

    def add_tool(self, msg = str, index = str):
        tool_ref = copy.deepcopy(self.tool_reference_)
        
        info = msg.split('\n')
        tool_ref['name'] = info[1]
        pos = info[2].split(' ')
        tool_ref['position']['x'] =  float(pos[0])
        tool_ref['position']['y'] =  float(pos[1])
        tool_ref['position']['z'] =  float(pos[2])
        tool_ref['release_dir'] =  int(pos[3])

        self.map_instance_['map_reference']['tools']['T' + index] = tool_ref
        self.get_logger().info(str(self.map_instance_))
        self.save_to_yaml(self.map_instance_, self.directory_, self.activeMap_, createIfNotExisting = True)

    def get_release_direction(self, dir = int):
        if dir < 1 or dir > 4: 
            self.get_logger().error("Release direction for the tool unrecognized! Check configuration!")
            return
        if dir == 1:
            return [-100.0, 0.0]
        if dir == 1:
            return [100.0, 0.0]
        if dir == 1:
            return [0.0, -100.0]
        if dir == 1:
            return [0.0, 100.0]

    def save_to_yaml(self, data = dict, path = '', fileName = '', createIfNotExisting = False):
        if not isinstance(data, dict):
            self.get_logger().warn("Parsed dictionary data is not of type dictionary")
            return
        if path == '':
            self.get_logger().warn("Path not set for retrieving the parameter config file")
            return
        if fileName == '':
            self.get_logger().warn("Parameter Config File name not set")
            return
        if not createIfNotExisting and not os.path.exists(path):
            self.get_logger().warn("File path is invalid")
            return
        
        if createIfNotExisting and not os.path.exists(path):
            self.get_logger().info("Creating the active map configuration file..+")
            os.makedirs(os.path.dirname(os.path.join(path, self.fileName)), exist_ok=True)

        self.get_logger().info(f"Saving current parameter configuration at {os.path.join(path, fileName)}")
            
        with open(os.path.join(path, fileName), 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style = False)

    def load_from_yaml(self, path = '', fileName = ''):
        if path == '':
            self.get_logger().warn("Path not set for retrieving the parameter config file")
            return
        if fileName == '':
            self.get_logger().warn("Parameter Config File name not set")
            return
        if not os.path.exists(path):
            self.get_logger().warn("File path is invalid")
            return
        
        with open(os.path.join(path, fileName), 'r') as yaml_file:
            loaded_data = yaml.safe_load(yaml_file)
            if isinstance(loaded_data, dict):
                return loaded_data
            else:
                self.get_logger().warn("Invalid YAML file format..")

    def retrieveMap(self, directory = '', fileName1 = '', fileName2 = ''):
        '''
        A function that attempts to retrieve the map configuration file from
        memory. If it fails, it either means that the file was deleted or the
        current run is a fresh run.

        args:
            directory {String}: The share directory the yaml files are located at
            fileName1 {String}: The active config (i.e. the one from memory)
            fileName2 {String}: The initial empty config (i.e. fresh run)
        '''
        activeConfig = os.path.join(directory, fileName1)
        if os.path.exists(activeConfig):
            self.get_logger().info("Initialized map from previous run!")
            return self.load_from_yaml(directory, fileName1)
        else:
            self.get_logger().warn("Previous map info could not be found! Unless you have a back-up, previous items need to be re-added")
            return self.load_from_yaml(directory, fileName2)


def main(args=None):
    rclpy.init(args=args)
    node = MapController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()