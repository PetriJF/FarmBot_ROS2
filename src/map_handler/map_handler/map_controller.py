import os
import yaml
import copy

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from farmbot_interfaces.msg import MapCommand, PlantManage
from farmbot_interfaces.srv import StringRepReq
from map_handler.tool_exchange import ToolDetails, ToolExchanger

class MapController(Node):
    '''
    Node that saves, modifies and handles the map information of the 
    farmbot. It saves the active map in the install share directory,
    recording information such as map details (dimensions, tool locations,
    tray locations) and plant details (plant locations, growth information,
    )
    '''
    def __init__(self):
        '''
        Node Constructor
        Loads all the config files and the publishers and subscribers
        '''
        super().__init__('MapController')

        # The safe Z increment for the sequences
        self.safe_z_increment_ = 80.0

        # Relevant directory and file names
        self.directory_ = os.path.join(
            get_package_share_directory('map_handler'),
            'config'
        )
        self.active_map_file_ = 'active_map.yaml'
        tool_ref_file = 'tool_reference.yaml'
        tray_ref_file = 'tray_reference.yaml'
        tray_16_ref_file = '16_seed_tray.yaml'
        reference_plant_file_ = 'plant_reference.yaml'
        reference_map_file_ = 'map_references.yaml'

        # Loading the map instance from memory
        self.map_instance_ = self.retrieve_map(directory = self.directory_,
                                               file_name1 = self.active_map_file_,
                                               file_name2 = reference_map_file_)
        
        # Loading the tool exhanging module and the tool command object
        self.tool_exchanger_ = ToolExchanger(node = self, 
                                             map_max_x = self.map_instance_['map_reference']['x_len'],
                                             map_max_y = self.map_instance_['map_reference']['y_len'],
                                             map_max_z = -self.map_instance_['map_reference']['z_len'])
        self.tool_details_ = ToolDetails()

        # Loading the plant referencing method
        self.plant_ref_ = self.load_from_yaml(self.directory_, reference_plant_file_)
        # Loading the tool referencing method
        self.tool_ref_ = self.load_from_yaml(self.directory_, tool_ref_file)
        # Loading the tray reference and 16 seed tray addon reference
        self.tray_ref_ = self.load_from_yaml(self.directory_, tray_ref_file)
        self.tray_16_ref_ = self.load_from_yaml(self.directory_, tray_16_ref_file)

        # MapCommand subscriber
        self.map_cmd_sub_ = self.create_subscription(MapCommand, 'map_cmd', self.map_cmd_callback, 10)
        # Plant Configuration Subscriber
        self.plant_mng_sub_ = self.create_subscription(PlantManage, 'plant_mng', self.plant_mng_callback, 10)
        # Map information service server
        self.map_info_server_ = self.create_service(StringRepReq, 'map_info', self.map_command_server)

        self.get_logger().info('Map Controller Initialized')

    def map_cmd_callback(self, cmd: MapCommand):
        '''
        Callback handling map commands. 
        Handles the sort, update and reindex commands
        TODO: Add description of each
        '''
        # Check command validity
        cmd_sum = sum([cmd.sort, cmd.update, cmd.reindex])
        if cmd_sum != 1:
            self.get_logger().warn(f'You can only use 1 command at a time! You used {cmd_sum}')
            return
        
        # TODO Add behaviour for each command type
        if cmd.sort:
            pass
        if cmd.reindex:
            pass
        if cmd.update:
            # Update the parsed information
            for update in cmd.update_info:
                cmd_split = update.split(' ')
                # Updating the map dimensions
                match cmd_split[0]:
                    case 'X': 
                        self.map_instance_['map_reference']['x_len'] = float(cmd_split[1])
                        self.tool_exchanger_.map_max_x = float(cmd_split[1])
                    case 'Y': 
                        self.map_instance_['map_reference']['y_len'] = float(cmd_split[1])
                        self.tool_exchanger_.map_max_y = float(cmd_split[1])
                    case 'Z': 
                        self.map_instance_['map_reference']['z_len'] = float(cmd_split[1])
                        self.tool_exchanger_.map_max_z = float(cmd_split[1])
                    case _: 
                        self.get_logger().warn(f'Command ({update}) not recognized and ignored!')
            
            # Reset the tool object with the map reference dimensions
            self.tool_exchanger_ = ToolExchanger(node = self, 
                                                 map_max_x = self.map_instance_['map_reference']['x_len'],
                                                 map_max_y = self.map_instance_['map_reference']['y_len'],
                                                 map_max_z = -self.map_instance_['map_reference']['z_len'])
            # Save the new active map
            self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_, create_if_empty = True)
        if cmd.back_up:
            pass

    def plant_mng_callback(self, cmd: PlantManage):
        '''
        Plant managing commands. Adding and removing plants from the map
        '''
        # Check command validity
        if (cmd.add and cmd.remove) or (not cmd.add and not cmd.remove):
            self.get_logger().warn('You must select either to add or remove a plant! Cannot do both/none in a commmand')
            return
        
        if cmd.add:
            if cmd.autopos: # TODO: Implement autopositioning
                pass
            else:
                self.add_plant(x = cmd.x, y = cmd.y, z = cmd.z, max_z = cmd.max_z, 
                               exclusion_radius = cmd.exclusion_radius,
                               canopy_radius = cmd.canopy_radius,
                               water_quantity = cmd.water_quantity,
                               plant_name = cmd.plant_name,
                               growth_stage = cmd.growth_stage)
        if cmd.remove:
            self.remove_plant(index = cmd.index)

    def add_plant(self, x: float, y: float, z: float, max_z: float, water_quantity: float, exclusion_radius: float, 
                 canopy_radius: float, plant_name: str, growth_stage: str):
        '''
        Creates the reference of the plant based on the parsed informations
        and adds it to the active map
        '''
        self.plant_ref_['identifiers']['plant_name'] = plant_name
        self.plant_ref_['position']['x'] = x
        self.plant_ref_['position']['y'] = y
        self.plant_ref_['position']['z'] = z
        self.plant_ref_['plant_details']['plant_radius'] = exclusion_radius
        self.plant_ref_['plant_details']['canopy_radius'] = canopy_radius
        self.plant_ref_['plant_details']['max_height'] = max_z
        self.plant_ref_['status']['growth_stage'] = growth_stage

        index = self.map_instance_['plant_details']['plant_count'] + 1
        self.plant_ref_['identifiers']['index'] = copy.deepcopy(index)

        self.map_instance_['plant_details']['plant_count'] += 1
        # Case for the first plant being added
        if index == 1:
            self.map_instance_['plant_details']['plants'] = {}

        self.map_instance_['plant_details']['plants'][copy.deepcopy(index)] = copy.deepcopy(self.plant_ref_)

        self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_, create_if_empty = True)

    def remove_plant(self, index: int):
        '''
        Removes the plant of the represented index
        '''
        plants = self.map_instance_['plant_details']['plants']
        if index in plants:
            del plants[index]
    
        self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_)

    def seed_plants(self):
        '''
        Creates the command sequence for planting all the seeds marked
        with the 'Planning' Growth Stage. This sequence is returned 
        to the farmbot controller for execution.

        EXECUTION OF THE SEQUENCE DOES NOT HAPPEN HERE
        '''
        cmd_sequence = ''
        plants = self.map_instance_['plant_details']['plants']
        for plant_index in plants:
            plant = plants[plant_index]
            if plant['status']['growth_stage'] == 'Planning':
                # Check if there are seeds available for the said plant
                plant_type = plant['identifiers']['plant_name']
                available, tray_index = self.__check_loaded_seeds(plant_type)
                if not available:
                    self.get_logger().warn(f"{plant['identifiers']['plant_name']} (index = {plant['identifiers']['index']}) could not be planted as {plant['identifiers']['plant_name']} seeds were not found to be loaded into the seed trays")
                    continue

                cmd_sequence += self.seed_plant(plant, self.map_instance_['map_reference']['trays'][tray_index])
                plant['status']['growth_stage'] = 'Seedling'

        if cmd_sequence == '':
            self.get_logger().warn('No seeds needed planting!')
            return ''

        if cmd_sequence[-1] == '\n':
            cmd_sequence = cmd_sequence[:-1]
        
        self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_, create_if_empty = True)
        return cmd_sequence

    def __check_loaded_seeds(self, type: str):
        '''
        Checks if there is a tray with the seed type loaded in it
        '''
        trays = self.map_instance_['map_reference']['trays']
        self.get_logger().info(str(trays))
        for tray_index in trays:
            tray = trays[tray_index]
            if tray:
                self.get_logger().info(str(tray))
                if tray['seed_type'] == type:
                    return True, tray_index

        return False, -1
    
    def seed_plant(self, plant: dict, tray: dict):
        '''
        Creates the sequence for planting a single seed.
        
        NOTE:
            Plant dictionary must be a child of a plant key in the main active map dictionary!
        '''
        cmd = ''

        plant_x = plant['position']['x']
        plant_y = plant['position']['y']
        plant_z = plant['position']['z']

        tray_x = tray['position']['x']
        tray_y = tray['position']['y']
        tray_z = tray['position']['z']

        cmd = f"CC_P_{plant['identifiers']['index']}_3\n"

        tray_clearance = 30

        # Go over seed tray at safe z
        cmd += f"{tray_x} {tray_y} {tray_z + self.safe_z_increment_}\n"
        # Turn on vacuum pump
        cmd += f"DC_P_{plant['identifiers']['index']}_3\n"
        cmd += 'Vacuum 1\n'
        # Collect a seed
        cmd += f"CC_P_{plant['identifiers']['index']}_3\n"
        cmd += f"{tray_x} {tray_y} {tray_z + tray_clearance}\n"
        # Retract with the seed
        cmd += f"{tray_x} {tray_y} {tray_z + self.safe_z_increment_}\n"
        # Go to the plant at safe z
        cmd += f"{plant_x} {plant_y} {plant_z + self.safe_z_increment_}\n"
        # Plant the seed
        cmd += f"{plant_x} {plant_y} {plant_z}\n"
        # Turn off vacuum pump
        cmd += f"DC_P_{plant['identifiers']['index']}_3\n"
        cmd += 'Vacuum 0\n'
        # Retract the empty seeder
        cmd += f"CC_P_{plant['identifiers']['index']}_3\n"
        cmd += f"{plant_x} {plant_y} {plant_z + self.safe_z_increment_}\n"

        return cmd

    def map_command_server(self, request, response):
        '''
        Map Command Server.
        Receives commands and returns either a sequence or a response to the request
        '''
        type = request.data[0]
        # Tool Command Type
        if type == 'T':
            response.data = self.tool_cmd_interpreter(request.data)
        if type == 'S':
            response.data == self.tray_cmd_interpreter(request.data)
        if request.data == 'P_3':
            response.data = self.seed_plants()
        if request.data == 'P_4':
            response.data = self.water_plants()
            
        return response

    def water_plants(self):
        '''
        Creates the sequence for watering all the plants by appending the sequences
        for watering each individual plant
        '''
        cmd_sequence = ''
        plants = self.map_instance_['plant_details']['plants']
        for plant_index in plants:
            plant = plants[plant_index]
            
            water_pulses = int(plant['plant_details']['water_quantity'])

            cmd_sequence += self.water_plant(plant, water_pulses)

        if cmd_sequence == '':
            self.get_logger().warn('No plants found!')
            return ''

        if cmd_sequence[-1] == '\n':
            cmd_sequence = cmd_sequence[:-1]
        return cmd_sequence

    def water_plant(self, plant: dict, pulses: int):
        '''
        Creates the sequence for watering a single plant
        '''
        cmd = ''

        plant_x = plant['position']['x']
        plant_y = plant['position']['y']
        plant_z = plant['position']['z']

        cmd = f"CC_P_{plant['identifiers']['index']}_4\n"
        # go to seed location
        cmd += f"{plant_x} {plant_y} {-self.safe_z_increment_}\n"
        # Turn on water pump pump
        cmd += f"DC_P_{plant['identifiers']['index']}_4\n"
        for i in range(pulses):
            cmd += f"WaterPulses {2000}\n"

        return cmd

    def tray_cmd_interpreter(self, msg: str):
        '''
        Interpreter for commands around the seed trays
        '''
        elem = msg.split('_')
        index = int(elem[1])
        cmd = int(elem[2])
        type = int(elem[3][0])

        if cmd == 0:
            tray_ref = copy.deepcopy(self.tray_ref_)
            info = elem[3].split('\n')
            tray_ref['name'] = info[1]
            tray_ref['seed_type'] = info[2] if not type else ''
            pos = info[3].split(' ')
            tray_ref['position']['x'] = float(pos[0])
            tray_ref['position']['y'] = float(pos[1])
            tray_ref['position']['z'] = float(pos[2])
            tray_ref['tray_type'] = type

            self.map_instance_['map_reference']['trays'][index] = tray_ref
            self.get_logger().info(str(self.map_instance_))
            self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_, create_if_empty = True)
        if cmd == 1:
            # TODO: Remove tool of index index
            pass
        if cmd == 2:
            # TODO: populate the 16 seed slot tray
            pass


    def tool_cmd_interpreter(self, msg: str):  
        '''
        Interpreter for commands around the tools
        '''
        elem = msg.split('_')
        index = elem[1]
        cmd = int(elem[2][0])

        # Setting up a new tool
        if cmd == 0:
            self.add_tool(msg, index)
            return 'T_x_0 SUCCESS'
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

        # Start the appropriate command
        self.get_logger().warn(f'Unrecognized command {str(msg)}')
        return 'UNRECOGNIZED'

    def add_tool(self, msg: str, index: str):
        '''
        Adding a tool's information to the active map dictionary
        '''
        tool_ref = copy.deepcopy(self.tool_ref_)
        
        info = msg.split('\n')
        tool_ref['name'] = info[1]
        pos = info[2].split(' ')
        tool_ref['position']['x'] =  float(pos[0])
        tool_ref['position']['y'] =  float(pos[1])
        tool_ref['position']['z'] =  float(pos[2])
        tool_ref['release_dir'] =  int(pos[3])

        self.map_instance_['map_reference']['tools']['T' + index] = tool_ref
        self.get_logger().info(str(self.map_instance_))
        self.save_to_yaml(self.map_instance_, self.directory_, self.active_map_file_, create_if_empty = True)

    def get_release_direction(self, dir: int):
        '''
        Gets the coordinate increments for the release of each tool based 
        on the mounting orientation
        '''
        if dir < 1 or dir > 4: 
            self.get_logger().error('Release direction for the tool unrecognized! Check configuration!')
            return
        if dir == 1:
            return [-100.0, 0.0]
        if dir == 1:
            return [100.0, 0.0]
        if dir == 1:
            return [0.0, -100.0]
        if dir == 1:
            return [0.0, 100.0]

    def save_to_yaml(self, data: dict, path = '', file_name = '', create_if_empty = False):
        '''
        Saves a dictionary to a yaml file in the share directory. If the file exists already,
        it updates it.

        Args:
            path {String}: The share directory the yaml files are located at
            file_name {String}: The active config (i.e. the one from memory)
        '''
        if not isinstance(data, dict):
            self.get_logger().warn('Parsed dictionary data is not of type dictionary')
            return
        if path == '':
            self.get_logger().warn('Path not set for retrieving the parameter config file')
            return
        if file_name == '':
            self.get_logger().warn('Parameter Config File name not set')
            return
        if not create_if_empty and not os.path.exists(path):
            self.get_logger().warn('File path is invalid')
            return
        
        if create_if_empty and not os.path.exists(path):
            self.get_logger().info('Creating the active map configuration file..')
            os.makedirs(os.path.dirname(os.path.join(path, file_name)), exist_ok=True)

        self.get_logger().info(f'Saving current parameter configuration at {os.path.join(path, file_name)}')
            
        with open(os.path.join(path, file_name), 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style = False)

    def load_from_yaml(self, path = '', file_name = ''):
        '''
        Leads a dictionary from a yaml file in the share directory

        Args:
            path {String}: The share directory the yaml files are located at
            file_name {String}: The active config (i.e. the one from memory)
        '''
        if path == '':
            self.get_logger().warn('Path not set for retrieving the parameter config file')
            return
        if file_name == '':
            self.get_logger().warn('Parameter Config File name not set')
            return
        if not os.path.exists(path):
            self.get_logger().warn('File path is invalid')
            return
        
        with open(os.path.join(path, file_name), 'r') as yaml_file:
            loaded_data = yaml.safe_load(yaml_file)
            if isinstance(loaded_data, dict):
                return loaded_data
            else:
                self.get_logger().warn('Invalid YAML file format..')

    def retrieve_map(self, directory = '', file_name1 = '', file_name2 = ''):
        '''
        Attempts to retrieve the map configuration file from memory.
        If it fails, it either means that the file was deleted or the
        current run is a fresh run.

        Args:
            directory {String}: The share directory the yaml files are located at
            file_name1 {String}: The active config (i.e. the one from memory)
            file_name2 {String}: The initial empty config (i.e. fresh run)
        '''
        active_config = os.path.join(directory, file_name1)
        if os.path.exists(active_config):
            self.get_logger().info('Initialized map from previous run!')
            return self.load_from_yaml(directory, file_name1)
        else:
            self.get_logger().warn('Previous map info could not be found! Unless you have a back-up, previous items need to be re-added')
            return self.load_from_yaml(directory, file_name2)


def main(args=None):
    rclpy.init(args=args)
    node = MapController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()