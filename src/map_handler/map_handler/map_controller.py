"""
Map controller node for ROS2 Farmbot.

Manages and persists Farmbot map information including dimensions, tool locations,
tray positions, and plant data. Provides services for map modifications and queries.
"""
import ast
import copy

from farmbot_interfaces.srv import AddPlant, AddSeedTray, AddTool, RemoveMapObject, UpdateMap

from farmbot_utils.exceptions import YAMLError
from farmbot_utils.yaml_handler import YAMLHandler

# from map_handler.tool_sequencer import ToolDetails, ToolExchanger

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from std_srvs.srv import Trigger


class MapController(Node):
    """
    Save, modify and handle the map information of the farmbot.

    The node saves the active map in the install share directory, recording information such as
    map details (dimensions, tool locations, tray locations) and plant details (plant locations,
    growth information).
    """

    def __init__(self):
        """
        Node Constructor.

        Loads all the config files and the publishers and subscribers
        """
        super().__init__('map_controller')

        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)

        ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        folder_config_name = self.get_parameter(
            'folder_config_name').get_parameter_value().string_value

        self.config_path = YAMLHandler.join_path(ws_path, folder_config_name)
        YAMLHandler.make_dir(self.config_path)

        self.directory = YAMLHandler.get_directory_package('map_handler', 'config')

        try:
            # self.water_guide_instance =YAMLHandler.load_yaml(self.directory,'watering_guide.yaml')
            self.plant_ref = YAMLHandler.load_yaml(self.directory, 'plant_reference.yaml')
            self.tool_ref = YAMLHandler.load_yaml(self.directory, 'tool_reference.yaml')
            self.tray_ref = YAMLHandler.load_yaml(self.directory, 'tray_reference.yaml')
            self.tray_16_ref = YAMLHandler.load_yaml(self.directory, '16_seed_tray.yaml')
            self.map_ref = YAMLHandler.load_yaml(self.directory, 'map_references.yaml')
            # self.watering_thresholds = YAMLHandler.load_yaml(self.directory,
            # 'watering_threshold.yaml')
        except YAMLError as e:
            self.get_logger().warn(f'yaml error: {e}')
            return

        self.active_map = 'active_map.yaml'

        # Loading the map instance from memory
        self.retrieve_map(path=self.config_path, file_name1=self.active_map,
                          file_name2='map_references.yaml')

        self.cmd_callback_group = ReentrantCallbackGroup()

        # Initialise state servers
        self.get_map_trigger = self.create_service(Trigger, 'map_cmd/get_map',
                                                   self.get_map_server,
                                                   callback_group=self.cmd_callback_group)

        # Map command service server
        self.add_plant = self.create_service(AddPlant, 'map_cmd/add_plant', self.add_plant_server,
                                             callback_group=self.cmd_callback_group)
        self.update_map = self.create_service(UpdateMap, 'map_cmd/update_map',
                                              self.update_map_server,
                                              callback_group=self.cmd_callback_group)
        self.add_tool = self.create_service(AddTool, 'map_cmd/add_tool', self.add_tool_server,
                                            callback_group=self.cmd_callback_group)
        self.add_tray = self.create_service(AddSeedTray, 'map_cmd/add_seed_tray',
                                            self.add_tray_server,
                                            callback_group=self.cmd_callback_group)
        self.remove_map_object = self.create_service(RemoveMapObject, 'map_cmd/remove_map_object',
                                                     self.remove_object_server,
                                                     callback_group=self.cmd_callback_group)

        self.get_logger().info('Map Controller Initialised')

    def add_plant_server(self, request: AddPlant.Request,
                         response: AddPlant.Response) -> AddPlant.Response:
        """
        Add a new plant to the map instance.

        The plant information is parsed from the provided string, assigned a unique index, and
        stored in the map instance. If this is the first plant being added, the plants dictionary
        is initialised.
        """
        if request.autopos:  # TODO: Implement autopositioning
            pass
        try:
            plant = copy.deepcopy(self.plant_ref)

            plant['identifiers']['plant_name'] = request.plant_name
            plant['position']['x'] = request.position.x
            plant['position']['y'] = request.position.y
            plant['position']['z'] = request.position.z
            plant['plant_details']['plant_radius'] = request.plant_radius
            plant['plant_details']['canopy_radius'] = request.canopy_radius
            plant['plant_details']['max_height'] = request.max_height
            plant['plant_details']['water_quantity'] = request.water_quantity
            plant['status']['growth_stage'] = request.growth_stage

            self.map_instance['plant_details']['plant_count'] += 1
            index = self.map_instance['plant_details']['plant_count']
            plant['identifiers']['index'] = index

            # Case for the first plant being added
            if index == 1:
                self.map_instance['plant_details']['plants'] = {}

            self.map_instance['plant_details']['plants'][index] = copy.deepcopy(plant)
            YAMLHandler.save_to_yaml(self.map_instance, self.config_path, self.active_map)
        except Exception as e:
            response.success = False
            response.message = f'{e}'
            self.get_logger().error(f'{e}')
            return response

        response.success = True
        response.message = ''
        return response

    def update_map_server(self, request: UpdateMap.Request,
                          response: UpdateMap.Response) -> UpdateMap.Response:
        """Update values in the map instance using key paths provided in the input."""
        try:
            for update in request.update_info:
                cmd_split = update.split(' ')

                n = len(cmd_split)
                map_copy = self.map_instance

                for keys in cmd_split[:n-2]:
                    try:
                        keys = ast.literal_eval(keys)
                    except (ValueError, SyntaxError):
                        pass
                    map_copy = map_copy[keys]
                map_copy[cmd_split[-2]] = ast.literal_eval(cmd_split[-1])
            YAMLHandler.save_to_yaml(self.map_instance, self.config_path, self.active_map)

        except Exception as e:
            response.success = False
            response.message = f'{e}'
            self.get_logger().error(f'{e}')
            return response

        response.success = True
        response.message = ''
        return response

    def reindex_plants(self):
        """Reindex all the plants after the removal of one in the list."""
        index = 1
        plants = self.map_instance['plant_details']['plants']
        for plant_index in list(plants):
            if int(plant_index) != index:
                plants[index] = plants.pop(plant_index)
                plant = plants[index]
                plant['identifiers']['index'] = copy.deepcopy(index)

            index += 1

    def add_tool_server(self, request: AddTool.Request,
                        response: AddTool.Response) -> AddTool.Response:
        """Add a tool's information to the active map dictionary."""
        try:
            tool = copy.deepcopy(self.tool_ref)

            tool['name'] = request.tool_name
            tool['position']['x'] = request.position.x
            tool['position']['y'] = request.position.y
            tool['position']['z'] = request.position.z
            tool['release_dir'] = request.release_dir
            index = str(request.index)

            self.map_instance['map_reference']['tools']['T' + index] = copy.deepcopy(tool)
            YAMLHandler.save_to_yaml(self.map_instance, self.config_path,
                                     self.active_map)
        except Exception as e:
            response.success = False
            response.message = f'{e}'
            self.get_logger().error(f'{e}')
            return response

        response.success = True
        response.message = ''
        return response

    def add_tray_server(self, request: AddSeedTray.Request,
                        response: AddSeedTray.Response) -> AddSeedTray.Response:
        """Add a tray's information to the active map dictionary."""
        # TODO: populate the 16 seed slot tray
        try:
            tray = copy.deepcopy(self.tray_ref)

            tray['name'] = request.name
            tray['seed_type'] = request.seed_type
            tray['position']['x'] = request.position.x
            tray['position']['y'] = request.position.y
            tray['position']['z'] = request.position.z
            tray['tray_type'] = request.tray_type

            if not self.map_instance['map_reference']['trays']:
                self.map_instance['map_reference']['trays'] = {}

            self.map_instance['map_reference']['trays'][request.index] = copy.deepcopy(tray)
            YAMLHandler.save_to_yaml(self.map_instance, self.config_path, self.active_map)
        except Exception as e:
            response.success = False
            response.message = f'{e}'
            self.get_logger().error(f'{e}')
            return response

        response.success = True
        response.message = ''
        return response

    def remove_object_server(self, request: RemoveMapObject.Request,
                             response: RemoveMapObject.Response) -> RemoveMapObject.Response:
        """Remove an object (plant, tool or seed_tray) from the active map."""
        index = request.index
        try:
            if request.op == RemoveMapObject.Request.PLANT:
                plants = self.map_instance['plant_details']['plants']
                del plants[index]
                self.get_logger().info(f'Removed plant with index {index}')
                self.reindex_plants()

            elif request.op == RemoveMapObject.Request.TOOL:
                tools = self.map_instance['map_reference']['tools']
                tools['T' + str(index)] = copy.deepcopy(
                    self.map_ref['map_reference']['tools']['T' + str(index)])
                self.get_logger().info(f'Removed tool with index {index}')

            elif request.op == RemoveMapObject.Request.SEED_TRAY:
                tray = self.map_instance['map_reference']['trays']
                del tray[index]
                self.get_logger().info(f'Removed tray with index {index}')

            YAMLHandler.save_to_yaml(self.map_instance, self.config_path, self.active_map)

        except KeyError:
            response.success = False
            response.message = (f'The object with index {index} that you want to delete is not '
                                'present in the active_map')
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = ''
        return response

    async def get_map_server(self, request: Trigger.Request,
                             response: Trigger.Response) -> Trigger.Response:
        """
        Handle get_map request.

        Sends the card to the modules that need it.

        Args:
            request {Trigger.Request}: Empty trigger request.
            response {Trigger.Response}: Service response object.
        """
        response.success = True
        response.message = str(self.map_instance)
        return response

    def retrieve_map(self, path='', file_name1='', file_name2=''):
        """
        Attempt to retrieve the map configuration file from memory.

        If it fails, it either means that the file was deleted or the
        current run is a fresh run.
        Args:
            path {String}: The path to the directory containing the yaml files
            file_name1 {String}: The active config (i.e. the one from memory)
            file_name2 {String}: The initial empty config (i.e. fresh run)
        """
        try:
            self.map_instance = YAMLHandler.load_yaml(path, file_name1)
            self.get_logger().info('Initialised map from previous run!')
        except YAMLError as e:
            try:
                self.node.get_logger().warn(f'{e}. Previous map could not be found! Unless you have'
                                            ' a back-up, previous items need to be re-added')
                self.map_instance = YAMLHandler.load_yaml(self.directory, file_name2)
            except YAMLError as e:
                self.node.get_logger().error(f'{e}. Failed to load any map configuration.')
                return


def main(args=None):
    """Initialise and run the map_controller node."""
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
