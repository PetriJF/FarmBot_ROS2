"""
Configuration parameter management module for the FarmBot SerialController.

Handles loading, writing and storing FarmBot firmware parameters on YAML file.
Provides the parameter management logic used by the SerialController to
synchronize configuration values with the firmware.
"""
from farmbot_hardware_comm.modules.exceptions import ServerError, YAMLError
from farmbot_hardware_comm.modules.param_info import ParameterList
from farmbot_hardware_comm.modules.yaml_handler import YAMLHandler

from farmbot_interfaces.action import LoadingParameters
from farmbot_interfaces.msg import MapCommand
from farmbot_interfaces.srv import ParameterConfig, StringRepReq

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node


class ConfigServer:
    """Handle the parameter recording and loading onto the farmduino."""

    # Node contructor
    def __init__(self, node: Node, config_path):
        """
        Config Handling Node Constructor.

        Sets up all the paths, servers, publishers and subscribers for the node.
        """
        self.node = node

        # Flag waiting for initialization to be done before the config is loaded
        self.firmware_init_done = False

        # The dictionary containing all of the parameters for the farmbot
        self.params = ParameterList()
        param_names = vars(self.params)
        self.param_vals = {
            param_names[name]: 0
            for name in param_names
        }

        self.default_path = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')
        self.config_path = config_path

        self.base_config = 'firmwareDefault.yaml'  # default config loaded by the firmware
        self.custom1_config = 'Custom1.yaml'       # custom configuration,modify in source and build
        self.genesis_config = 'Genesis.yaml'       # farmbot genesis config
        self.express_config = 'Express.yaml'       # farmbot express config
        self.active_config = 'activeConfig.yaml'   # configuration loaded from previous run

        YAMLHandler.make_dir(YAMLHandler.join_path(config_path, self.active_config))

        # TODO: Add more default configurations other than the labFB one

        # Config Service Servers
        self.config_server = self.node.create_service(ParameterConfig,
                                                      'manage_param_config',
                                                      self.config_request_server)
        self.config_loading_server = self.node.create_service(StringRepReq,
                                                              'load_param_config',
                                                              self.param_loading_server)

        # Config parameter clients
        self.load_params_client = ActionClient(self.node, LoadingParameters, 'loading_params')

        # Map updating publisher
        self.map_cmd = MapCommand()
        self.map_cmd_pub = self.node.create_publisher(MapCommand, 'map_cmd', 10)

        # Log the initialization
        self.node.get_logger().info('Config Server Initialized..')

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_server(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('ConfigParameter node failed: server unavailable')

    def retrieve_config(self):
        """Load the active configuration file, if it exists, and write it to the Farmduino."""
        if not self.firmware_init_done:
            self.firmware_init_done = True
            active_config_path = YAMLHandler.join_path(self.config_path, self.active_config)
            if YAMLHandler.existing_path(active_config_path):
                self.param_vals = YAMLHandler.load_yaml(self.config_path, self.active_config)
                self.load_params()
                self.node.get_logger().info('Initialized with active config from previous run')
            else:
                self.node.get_logger().warn(
                    'Previous config could not be found! You '
                    'will need to initialize the appropriate parameter config'
                )

    def param_loading_server(self, request, response):
        """Service Server that loads the default parameter configurations onto the Farmduino."""
        try:
            if request.data in ['Genesis', 'genesis', 'Gen', 'gen']:
                self.param_vals = YAMLHandler.load_yaml(path=self.default_path,
                                                        file_name=self.genesis_config)
                self.node.get_logger().info('Loading the genesis configuration')
                self.load_params()
            elif request.data in ['Express', 'express', 'exp', 'Exp']:
                self.param_vals = YAMLHandler.load_yaml(path=self.default_path,
                                                        file_name=self.express_config)
                self.node.get_logger().info('Loading the express configuration')
                self.load_params()
            # A configuration more specific to the model you are running
            elif request.data in ['Custom', 'custom']:
                self.param_vals = YAMLHandler.load_yaml(path=self.default_path,
                                                        file_name=self.custom1_config)
                self.node.get_logger().info('Loading the custom configuration')
                self.load_params()
            else:
                self.node.get_logger().warning('Config type unrecognized or not set.'
                                               'Nothing Loaded!')
                response.data = 'FAILED'
                return response
            response.data = 'LOADED'
            return response

        except YAMLError as e:
            self.node.get_logger().warn(f'yaml error: {e}')
            response.data = 'FAILED'
            return response

    def load_params(self):
        """
        Load the required parameters on the Farmduino.

        Compares the current firmware configuration with the desired parameter
        values and sends only the parameters that need to be updated.
        """
        try:
            loaded_firmware_config = YAMLHandler.load_yaml(self.default_path, self.base_config)
        except YAMLError as e:
            self.node.get_logger().warn(f'yaml error: {e}')
            return

        params = []
        param_values = []
        for key, value in self.param_vals.items():
            if (loaded_firmware_config[key] != value):
                params.append(key)
                param_values.append(value)

        self.send_loading_params_goal(params, param_values)

    def send_loading_params_goal(self, params: list, values: list):
        """
        Send a LoadingParameters goal to the FarmBot action server.

        Creates and sends a LoadingParameters goal containing the parameters
            to load and their associated values.

        Args:
            params (list): List of parameter names to load.
            values (list): List of values associated with each parameter.
        """
        self._server_availability('LoadingParameters', self.load_params_client)

        goal = LoadingParameters.Goal()
        goal.params = params
        goal.values = values

        self.load_params_client.send_goal_async(
            goal,
            feedback_callback=self.loading_goal_feedback_callback
            ).add_done_callback(self.goal_response_callback)

    def loading_goal_feedback_callback(self, feedback_msg: LoadingParameters.Feedback):
        """
        Handle feedback messages from the LoadingParameters action server.

        Logs the progress of he parameter loading on the Farmbot.
        """
        progress = feedback_msg.feedback.progress

        self.node.get_logger().info(f'Loading parameter progression: {progress*100:.2f} %')

    def goal_response_callback(self, future):
        """
        Handle the action server goal response.

        Processes the goal response and retrieves the result asynchronously if
        the goal has been accepted.
        """
        self.goal_handle: ClientGoalHandle = future.result()

        if self.goal_handle.accepted:
            self.node.get_logger().info('Goal accepted')

            self.goal_handle.get_result_async().add_done_callback(
                self.goal_result_callback
            )

        else:
            self.busy_state = False
            self.node.get_logger().warn('Goal rejected')

    def goal_result_callback(self, future):
        """
        Handle the final result from the action server.

        Processes the action result and logs the command status returned by the
        FarmBot action server.
        """
        result = future.result().result
        cmd_status = result.code

        if cmd_status == result.ESTOPPED:
            self.node.get_logger().info('The current command has been stopped by a estop request')

        elif cmd_status == result.ABORTED:
            self.node.get_logger().info('The Farmbot has been paused.')

        elif cmd_status == result.FIRMWARE_ERROR:
            self.node.get_logger().info('The command has finished with due to a firmware error.')

        elif cmd_status == result.REJECTED:
            self.node.get_logger().info(f'The command has been rejected. {result.message}')

        elif cmd_status == result.OK:
            self.node.get_logger().info('Parameter loading complete!')

    def config_request_server(self, request, response):
        """Service server that receives commands, returns responses, and executes instructions."""
        response.success = True    # success until proven otherwise
        response.cmd = request.data
        msg_split = (request.data).split(' ')
        code = msg_split[0]
        # Using both command and report codes so that the commands themselves
        # can be just fed into it with ease
        if code == 'R21' or code == 'R23' or code == 'F22' or code == 'F23':
            self.set_value(int(msg_split[1][1:]), int(msg_split[2][1:]))
            response.value = 0
            return response
        if code == 'F21':
            response.value = self.__get_value(msg_split[1][1:])
            return response

        # Requests with non farmbot commands
        if code == 'S':  # Format S PARAM_INDEX PARAM_VALUE. e.g. S 2 1
            self.set_value(msg_split[1], msg_split[2])

            return response
        if code == 'G':
            response.value = self.__get_value(msg_split[1])
            return response
        if code == 'MAP':
            response.value = 0
            self.map_cmd.sort = False
            self.map_cmd.reindex = False
            self.map_cmd.back_up = False
            self.map_cmd.update = True
            self.map_cmd.update_info = [
                'X ' + str(
                    self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_X]
                    / self.param_vals[self.params.MOVEMENT_STEP_PER_MM_X]
                ),
                'Y ' + str(
                    self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_Y]
                    / self.param_vals[self.params.MOVEMENT_STEP_PER_MM_Y]
                ),
                'Z ' + str(
                    self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_Z]
                    / self.param_vals[self.params.MOVEMENT_STEP_PER_MM_Z]
                ),
            ]

            response.cmd = ('MAP ' + self.map_cmd.update_info[0]
                            + ' ' + self.map_cmd.update_info[1]
                            + ' ' + self.map_cmd.update_info[2])

            self.map_cmd_pub.publish(self.map_cmd)
            return response
        if code == 'SAVE':
            response.value = 0
            YAMLHandler.save_to_yaml(self.param_vals, self.config_path, self.active_config)
            self.node.get_logger().info('Saving current parameter configuration at'
                                        f'{YAMLHandler.join_path(self.config_path,
                                                                 self.active_config)}')
            return response

        # If the service gets here, the request could not be processed
        self.node.get_logger().warn('Config managing service could not process'
                                    f'request {request.data}')
        response.success = False
        response.value = 0
        return response

    def set_value(self, param, value):
        """Set the selected parameter to the parsed value."""
        self.node.get_logger().info(f'Set parameter {param} to {value}')
        self.param_vals[param] = value

    def __get_value(self, param):
        """Return a value of a selected parameter."""
        return self.param_vals[param]
