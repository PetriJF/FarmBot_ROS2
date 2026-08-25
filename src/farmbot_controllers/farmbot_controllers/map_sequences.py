"""
FarmBot map sequences module.

Provides helper methods for watering and check moisture commands.
"""
import ast
import math

from farmbot_controllers.command_map import Call
from farmbot_controllers.sequences.check_moisture import check_moisture
from farmbot_controllers.sequences.seed_plant import seed_plant
from farmbot_controllers.sequences.water_plant import water_plant

from farmbot_interfaces.srv import UpdateMap

from farmbot_utils.exceptions import ServerError, YAMLError
from farmbot_utils.yaml_handler import YAMLHandler

from rclpy.node import Node

from std_srvs.srv import Trigger


class MapSequences:
    """Map sequences module that extends the farmbot controller node."""

    def __init__(self, node: Node):
        """Initialise the map_sequences module."""
        self.node = node
        self.current_map = {}

        # Config service client
        self.get_map_client = self.node.create_client(Trigger, 'get_map')

        # Update map client
        self.update_map_client = self.node.create_client(UpdateMap, 'update_map')

        self.directory = YAMLHandler.get_directory_package('farmbot_controllers', 'config')
        try:
            self.watering_thresholds = YAMLHandler.load_yaml(self.directory,
                                                             'watering_threshold.yaml')
            self.water_guide_instance = YAMLHandler.load_yaml(self.directory, 'watering_guide.yaml')
        except YAMLError as e:
            self.node.get_logger().warn(f'yaml error: {e}')
            return

    def get_map(self, on_done=None):
        """Request the current map from the MapController node."""
        self._server_availability('GetMap', self.get_map_client)

        self.get_map_client.call_async(Trigger.Request()).add_done_callback(
            lambda future: self._complete(future, on_done))

    def update_map_cmd(self, update_info: list, on_done=None):
        """Send a command to the map service."""
        self._server_availability('UpdateMap', self.update_map_client)
        request = UpdateMap.Request()
        request.update_info = update_info
        self.update_map_client.call_async(request=request).add_done_callback(
                    lambda future: self._complete(future, on_done))

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('Map_controller failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report, never leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not response.success:
            self.node.get_logger().warn(f'Command failed: {response.message}')
        elif response.message:
            parsed_map = ast.literal_eval(response.message)
            if isinstance(parsed_map, dict):
                self.current_map = parsed_map
        else:
            self.node.get_logger().info('Command successful')
            self.current_map = ast.literal_eval(response.message)
        if on_done is not None:
            on_done(response)

    def seed_plants_command(self, on_done=None):
        """
        Create the command sequence for planting the seeds marked with the 'Planning' Growth Stage.

        This sequence is returned to the farmbot controller for execution.

        EXECUTION OF THE SEQUENCE DOES NOT HAPPEN HERE
        """

        def seed_all_plants(response):
            if response is None or not response.success:
                error = response.message if response is not None else 'no response'
                raise ServerError(f'get_map server failed:{error}')

            updated_map = []

            plants = self.current_map['plant_details']['plants']
            for plant_index in plants:
                plant = plants[plant_index]
                if plant['status']['growth_stage'] == 'Planning':
                    # Check if there are seeds available for the said plant
                    plant_type = plant['identifiers']['plant_name']
                    available, tray_index = self.__check_loaded_seeds(plant_type)
                    if not available:
                        self.node.get_logger().warn(f"{plant['identifiers']['plant_name']} "
                                                    f"(index = {plant['identifiers']['index']}) "
                                                    'could not be planted as '
                                                    f"{plant['identifiers']['plant_name']} seeds "
                                                    'were not found to be loaded into '
                                                    'the seed trays')
                        continue

                    tray = self.current_map['map_reference']['trays'][tray_index]

                    plant_x = plant['position']['x']
                    plant_y = plant['position']['y']
                    plant_z = (-1.0) * self.current_map['map_reference']['z_len']

                    tray_x = tray['position']['x']
                    tray_y = tray['position']['y']
                    tray_z = tray['position']['z']

                    safe_z_increment = self.current_map['map_reference']['safe_z_increment']

                    on_done(seed_plant(plant_x=plant_x, plant_y=plant_y, plant_z=plant_z,
                                       tray_x=tray_x, tray_y=tray_y, tray_z=tray_z,
                                       z_increment=safe_z_increment))

                    updated_map.append(f'plant_details plants {plant_index} status '
                                       'growth_stage "Seedling"')

                    self.update_map_cmd(update_info=updated_map)

            if not updated_map:
                self.node.get_logger().warn('No seeds needed planting!')

        self.get_map(on_done=seed_all_plants)

    def __check_loaded_seeds(self, seed_type: str):
        """Check if there is a tray with the seed type loaded in it."""
        trays = self.current_map['map_reference']['trays']

        for tray_index in trays:
            tray = trays[tray_index]
            if tray:
                self.node.get_logger().info(str(tray))
                if tray['seed_type'] == seed_type:
                    return True, tray_index

        return False, -1

    def water_plants_cmd(self, rigid=False, on_done=None):
        """Create a watering sequence for each plant in the current map."""

        def general_watering(response):
            if response is None or not response.success:
                error = response.message if response is not None else 'no response'
                raise ServerError(f'get_map server failed:{error}')

            plant_nb = 0
            plants = self.current_map['plant_details']['plants']
            for plant_index in plants:
                plant = plants[plant_index]

                plant_x = plant['position']['x']
                plant_y = plant['position']['y']
                # plant_z = plant['position']['z']

                water_pulses = (
                    int(plant['plant_details']['water_quantity']) if rigid
                    else (self._map_moisture_reading(reading=int(
                                                           plant['plant_details']['soil_moisture']),
                                                     plant_name=plant['identifiers']['plant_name']))
                    )

                on_done(water_plant(plant_x=plant_x, plant_y=plant_y, delay_ms=water_pulses))

                plant_nb += 1

            if plant_nb == 0:
                raise ValueError('No plants found!')

        self.get_map(on_done=general_watering)

    def _map_moisture_reading(self, reading: int, plant_name: str) -> int:
        """Determine the watering quantity based on soil moisture."""
        DRY_THRESHOLD_MAX = self.watering_thresholds['dry_threshold_max']
        AVERAGE_THRESHOLD_MAX = self.watering_thresholds['average_threshold_max']
        WET_THRESHOLD_MAX = self.watering_thresholds['wet_threshold_max']

        if reading <= DRY_THRESHOLD_MAX:
            return self.water_guide_instance[plant_name]['dry']
        if reading <= AVERAGE_THRESHOLD_MAX:
            return self.water_guide_instance[plant_name]['average']
        if reading <= WET_THRESHOLD_MAX:
            return self.water_guide_instance[plant_name]['wet']
        # If it gets here it means that it is too wet and therefore no watering happens
        return 0

    def check_moisture_cmd(self, on_done=None) -> str:
        """
        Generate a sequence of commands to probe the soil moisture around each plant.

        Returns:
        str: A sequence of commands for probing soil moisture.
        """

        def moisture_checking(response):
            if response is None or not response.success:
                error = response.message if response is not None else 'no response'
                raise ServerError(f'get_map server failed:{error}')

            # Get the constraints of the map
            max_x = self.current_map['map_reference']['x_len']
            max_y = self.current_map['map_reference']['y_len']
            max_z = (-1.0) * self.current_map['map_reference']['z_len']

            # Get the details of all the plants and iterate through them
            plants = self.current_map['plant_details']['plants']
            for plant_index in plants:
                plant = plants[plant_index]

                # Get the probing location
                index = plant['identifiers']['index']
                location = self.get_probing_location(plants=plants,
                                                     x=plant['position']['x'],
                                                     y=plant['position']['y'],
                                                     exl_r=plant['plant_details']['plant_radius'],
                                                     max_x=max_x,
                                                     max_y=max_y,
                                                     index=index)
                if location is None:
                    self.node.get_logger().warn('No valid probing location found for '
                                                f'plant {index}.')
                    return

                x, y = location
                on_done(check_moisture(max_z=max_z, ticj_delay=2, x=x, y=y, plant_index=index))

            # Return home
            on_done(Call('home', 'movement', 'go_home'))

        self.get_map(on_done=moisture_checking)

    def get_probing_location(self, plants: dict, x: float, y: float, exl_r: float,
                             max_x: float, max_y: float, index: int) -> tuple[float, float] | None:
        """
        Determine a probing location around a plant.

        Args:
        plants (dict): Dictionary with all the plants.
        x (float): x-coordinate of the plant.
        y (float): y-coordinate of the plant.
        exl_r (float): Exclusion radius around the plant.
        max_x (float): Maximum x-axis position
        max_y (float): Maximum y-axis position

        Returns:
        (float, float): New (x, y) coordinates for the probing location.
        """
        # Define boundary limits
        threshold = self.watering_thresholds['threshold']

        min_x, min_y = threshold, threshold
        max_x, max_y = max_x - threshold, max_y - threshold

        # Function to check if a point is within the exclusion radius of any plant except itself
        def is_within_exclusion_radius(px, py, plant_id):
            for other_plant_id, plant_info in plants.items():
                if other_plant_id == plant_id:
                    continue
                plant_x = plant_info['position']['x']
                plant_y = plant_info['position']['y']
                plant_radius = plant_info['plant_details']['plant_radius']
                distance = math.sqrt((px - plant_x) ** 2 + (py - plant_y) ** 2)
                if distance < plant_radius + exl_r:
                    return True
            return False

        # Iterate over angles to find a valid position on the exclusion radius
        for angle in range(0, 360, 5):  # Check every 5 degrees
            radians = math.radians(angle)
            probe_x = x + exl_r * math.cos(radians)
            probe_y = y + exl_r * math.sin(radians)

            # Check if the position is within bounds
            if min_x <= probe_x <= max_x and min_y <= probe_y <= max_y:
                # Check if the position is not within the exclusion radius of any other plant
                if not is_within_exclusion_radius(probe_x, probe_y, index):
                    return probe_x, probe_y

        # If no valid position is found, return None
        return None
