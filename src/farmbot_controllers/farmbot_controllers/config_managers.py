"""
Config server node for ROS2 Farmbot.

Handles parameter recording, loading, and publishing for Farmbot firmware
configuration management, including parameter service interfaces.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory

from farmbot_controllers.param_info import ParameterList

from farmbot_interfaces.msg import MapCommand
from farmbot_interfaces.srv import ParameterConfig, StringRepReq

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import yaml


class ConfigServer(Node):
    """Handle the parameter recording and loading onto the farmduino."""

    # Node contructor
    def __init__(self):
        """
        Config Handling Node Constructor.

        Sets up all the paths, servers, publishers and subscribers for the node.
        """
        super().__init__('ConfigServer')
        # Flag waiting for initialization to be done before the config is loaded
        self.firmware_init_done = False

        # The dictionary containing all of the parameters for the farmbot
        self.params = ParameterList()
        param_names = vars(self.params)
        self.param_vals = {
            param_names[name]: 0
            for name in param_names
        }

        # The share directory path and the file names for all the files
        self.default_path = os.path.join(
            get_package_share_directory('farmbot_controllers'),
            'config'
        )

        self.declare_parameter('ws_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('folder_config_name', rclpy.Parameter.Type.STRING)

        ws_path = self.get_parameter('ws_path').get_parameter_value().string_value
        folder_config_name = self.get_parameter(
            'folder_config_name').get_parameter_value().string_value

        self.config_path = os.path.join(ws_path, folder_config_name)
        os.makedirs(self.config_path, exist_ok=True)

        self.base_config = 'firmwareDefault.yaml'  # default config loaded by the firmware
        self.custom1_config = 'Custom1.yaml'       # custom configuration,modify in source and build
        self.genesis_config = 'Genesis.yaml'       # farmbot genesis config
        self.express_config = 'Express.yaml'       # farmbot express config
        self.active_config = 'activeConfig.yaml'   # configuration loaded from previous run

        # TODO: Add more default configurations other than the labFB one

        # Config Service Servers
        self.config_server = self.create_service(ParameterConfig,
                                                 'manage_param_config', self.config_request_server)
        self.config_loading_server = self.create_service(StringRepReq,
                                                         'load_param_config',
                                                         self.param_loading_server)

        # Parameter Command publisher (Used for loading up parameters)
        self.param_cmd = String()
        self.param_cmd_pub = self.create_publisher(String, 'farmbot_command', 10)

        # Farmbot Feedback Subscriber
        self.fb_feedback_sub = self.create_subscription(String,
                                                        'farmbot_feedback',
                                                        self.fb_feedback_callback, 10)

        # Map updating publisher
        self.map_cmd = MapCommand()
        self.map_cmd_pub = self.create_publisher(MapCommand, 'map_cmd', 10)

        # Log the initialization
        self.get_logger().info('Config Server Initialized..')

    def fb_feedback_callback(self, msg: String):
        """Check for parameter value updates and for when the farmduino startup completes."""
        msg_split = (msg.data).split(' ')
        reportCode = msg_split[0]
        if msg.data == 'R99 ARDUINO STARTUP COMPLETE' and not self.firmware_init_done:
            self.firmware_init_done = True
            self.retrieve_config()
        if reportCode == 'R21' or reportCode == 'R23':
            self.__set_value(int(float(msg_split[1][1:])), int(float(msg_split[2][1:])))
            self.get_logger().info(f'Updated parameter {msg_split[1]} to {msg_split[2]}')

    def retrieve_config(self):
        """Load the active configuration file, if it exists, and write it to the Farmduino."""
        active_config_path = os.path.join(self.config_path, self.active_config)
        if os.path.exists(active_config_path):
            self.load_from_yaml(self.config_path, self.active_config)
            self.load_params()
            self.get_logger().info('Initialized with active config from previous run')
        else:
            self.get_logger().warn(
                'Previous config could not be found! You '
                'will need to initialize the appropriate parameter config'
            )

    def param_loading_server(self, request, response):
        """Service Server that loads the default parameter configurations onto the Farmduino."""
        if request.data in ['Genesis', 'genesis', 'Gen', 'gen']:
            self.load_from_yaml(path=self.default_path, file_name=self.genesis_config)
            self.get_logger().info('Loading the genesis configuration')
            self.load_params()
        elif request.data in ['Express', 'express', 'exp', 'Exp']:
            self.load_from_yaml(path=self.default_path, file_name=self.express_config)
            self.get_logger().info('Loading the express configuration')
            self.load_params()
        # A configuration more specific to the model you are running
        elif request.data in ['Custom', 'custom']:
            self.load_from_yaml(path=self.default_path, file_name=self.custom1_config)
            self.get_logger().info('Loading the custom configuration')
            self.load_params()
        else:
            self.get_logger().warning('Config type unrecognized or not set. Nothing Loaded!')
            response.data = 'FAILED'
            return response

        response.data = 'LOADED'
        return response

    def load_params(self):
        """
        Load all the parameters on the farmduino.

        list = False
        write = True
        read = False
        update = False
        """
        # Load only parameters that differ from the desired values (improves upload speed).
        with open(os.path.join(self.default_path, self.base_config), 'r') as yaml_file:
            loaded_firmware_config = yaml.safe_load(yaml_file)
            for key, value in self.param_vals.items():
                if (loaded_firmware_config[key] != value):
                    self.param_cmd.data = ('parameter_command False True False False ' + str(key)
                                           + ' ' + str(value))
                    self.param_cmd_pub.publish(self.param_cmd)
                    time.sleep(0.1)

        self.get_logger().info('Parameter loading complete!')

    def config_request_server(self, request, response):
        """Service server that receives commands, returns responses, and executes instructions."""
        response.success = True    # success until proven otherwise
        response.cmd = request.data
        msg_split = (request.data).split(' ')
        code = msg_split[0]
        # Using both command and report codes so that the commands themselves
        # can be just fed into it with ease
        if code == 'R21' or code == 'R23' or code == 'F22' or code == 'F23':
            self.__set_value(int(msg_split[1][1:]), int(msg_split[2][1:]))
            response.value = 0
            return response
        if code == 'F21':
            response.value = self.__get_value(msg_split[1][1:])
            return response

        # Requests with non farmbot commands
        if code == 'S':  # Format S PARAM_INDEX PARAM_VALUE. e.g. S 2 1
            self.__set_value(msg_split[1], msg_split[2])

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
            os.makedirs(os.path.dirname(os.path.join(self.config_path, self.active_config)),
                        exist_ok=True)
            self.save_to_yaml(self.config_path, self.active_config)
            return response

        # If the service gets here, the request could not be processed
        self.get_logger().warn(f'Config managing service could not process request {request.data}')
        response.success = False
        response.value = 0
        return response

    def save_to_yaml(self, path='', file_name=''):
        """Save a yaml file to a specified path and with a specified file name."""
        if path == '':
            self.get_logger().warn('Path not set for retrieving the parameter config file')
            return
        if file_name == '':
            self.get_logger().warn('Parameter Config File name not set')
            return
        if not os.path.exists(path):
            self.get_logger().warn('File path is invalid')
            return

        self.get_logger().info(f'Saving current parameter configuration \
                               at {os.path.join(path, file_name)}')

        with open(os.path.join(path, file_name), 'w') as yaml_file:
            yaml.dump(self.param_vals, yaml_file, default_flow_style=False)

    def load_from_yaml(self, path='', file_name=''):
        """Read the file with the specified file_name from the path as a dictionary."""
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
                self.param_vals = loaded_data
            else:
                self.get_logger().warn('Invalid YAML file format..')

    def __set_value(self, param, value):
        """Set the selected parameter to the parsed value."""
        self.get_logger().info(f'Set parameter {param} to {value}')
        self.param_vals[param] = value

    def __get_value(self, param):
        """Return a value of a selected parameter."""
        return self.param_vals[param]


def main(args=None):
    """Initialize and run the ConfigServer ROS2 node."""
    rclpy.init(args=args)

    config_server = ConfigServer()

    try:
        rclpy.spin(config_server)
    except KeyboardInterrupt:
        pass
    finally:
        config_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
