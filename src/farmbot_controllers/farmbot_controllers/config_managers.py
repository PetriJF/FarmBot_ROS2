import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import MapCommand
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from farmbot_interfaces.msg import ParameterCommand
from farmbot_interfaces.srv import ParameterConfig, LoadParamConfig

import os
import time
import yaml
from farmbot_controllers.param_info import ParameterList

class ConfigServer(Node):
    '''
    Node handling the parameter recording and loading onto the farmduino
    '''
    # Node contructor
    def __init__(self):
        '''
         Config Handling Node Constructor
         Sets up all the paths, servers, publishers and subscribers for the node
        '''
        super().__init__('ConfigServer')
        # Flag waiting for initialization to be done before the config is loaded
        self.firmware_init_done_ = False

        # The dictionary containing all of the parameters for the farmbot
        self.params = ParameterList()
        param_names = vars(self.params)
        self.param_vals = {
            param_names[name]: 0
            for name in param_names
        }

        # The share directory path and the file names for all the files
        self.default_path_ = os.path.join(
            get_package_share_directory('farmbot_controllers'),
            'config'
        )
        self.base_config_ = 'firmwareDefault.yaml'          # default config loaded by the firmware
        self.default_config_ = 'defaultParameterConfig.yaml'
        self.lab_fb_config_ = 'labFB.yaml'
        self.active_config_ = 'activeConfig.yaml'
        
        # TODO: Add more default configurations other than the labFB one

        # Config Service Servers
        self.config_server_ = self.create_service(ParameterConfig, 'manage_param_config', self.config_request_server)
        self.config_loading_server_ = self.create_service(LoadParamConfig, 'load_param_config', self.param_loading_server)

        # Parameter Command publisher (Used for loading up parameters)
        self.param_cmd_ = ParameterCommand()
        self.param_cmd_pub_ = self.create_publisher(ParameterCommand, 'parameter_command', 10)

        # UART Rx Subscriber
        self.uart_rx_sub_ = self.create_subscription(String, 'uart_receive', self.uart_rx_callback, 10)

        # Map updating publisher
        self.map_cmd_ = MapCommand()
        self.map_cmd_pub_ = self.create_publisher(MapCommand, 'map_cmd', 10)

        # Log the initialization
        self.get_logger().info('Config Server Initialized..')

    def uart_rx_callback(self, msg: String):
        '''
        Subscriber to the Serial Response from the Farmduino.
        Checks for parameter value updates and for when the farmduino startup completes 
        '''
        msg_split = (msg.data).split(' ')
        reportCode = msg_split[0]
        if msg.data == 'R99 ARDUINO STARTUP COMPLETE' and not self.firmware_init_done_:
            self.firmware_init_done_ = True
            self.retrieve_config()
        if reportCode == 'R21' or reportCode == 'R23':
            self.__set_value(int(float(msg_split[1][1:])), int(float(msg_split[2][1:])))
            self.get_logger().info(f'Updated parameter {msg_split[1]} to {msg_split[2]}')

    def retrieve_config(self):
        '''
        If it exists, the active parameter configuration file is loaded in and
        written to the Farmduino
        '''
        active_config_path = os.path.join(self.default_path_, self.active_config_)
        if os.path.exists(active_config_path):
            self.load_from_yaml(self.default_path_, self.active_config_)
            self.load_params()
            self.get_logger().info('Initialized with active config from previous run')
        else:
            self.get_logger().warn('Previous config could not be found! You will need to initialize the appropriate parameter config')

    def param_loading_server(self, request, response):
        '''
        Service Server that loads the parameter configurations onto the Farmduino.
        
        '''
        # A standard config that works for all farmbots
        if request.standard: 
            self.load_from_yaml(path = self.default_path_, file_name = self.default_config_)
            self.load_params()
        # A configuration more specific to the model you are running
        else:
            ver = request.ver ## TODO add support for multiple configurations
            self.load_from_yaml(path = self.default_path_, file_name = self.lab_fb_config_)
            self.load_params()

        response.success = True
        return response

    def load_params(self):
        '''
        Loading all the parameters on the farmduino
        '''
        self.param_cmd_.list = False
        self.param_cmd_.write = True
        self.param_cmd_.read = False
        self.param_cmd_.update = False

        # Loading only the parameters that are not loaded to the desired value (Greatly increases upload speed)
        with open(os.path.join(self.default_path_, self.base_config_), 'r') as yaml_file:
            loaded_firmware_config = yaml.safe_load(yaml_file)
            for key, value in self.param_vals.items():
                if(loaded_firmware_config[key] != value):
                    self.param_cmd_.param = key
                    self.param_cmd_.value = value
                    self.param_cmd_pub_.publish(self.param_cmd_)
                    time.sleep(0.1)

        self.get_logger().info('Parameter loading complete!')

    def config_request_server(self, request, response):
        '''
        Service Server that receives a command and returns the appropriate response
        and executes and instructions attributed to the commands
        '''
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
        if code == 'S': # Format S PARAM_INDEX PARAM_VALUE. e.g. S 2 1
            self.__set_value(msg_split[1], msg_split[2])
            
            return response
        if code == 'G':
            response.value = self.__get_value(msg_split[1])
            return response
        if code == 'MAP':
            response.value = 0
            self.map_cmd_.sort = False
            self.map_cmd_.reindex = False
            self.map_cmd_.back_up = False
            self.map_cmd_.update = True
            self.map_cmd_.update_info = [
                'X ' + str(self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_X] / 
                                self.param_vals[self.params.MOVEMENT_STEP_PER_MM_X]),
                'Y ' + str(self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_Y] / 
                                self.param_vals[self.params.MOVEMENT_STEP_PER_MM_Y]),
                'Z ' + str(self.param_vals[self.params.MOVEMENT_AXIS_NR_STEPS_Z] / 
                                self.param_vals[self.params.MOVEMENT_STEP_PER_MM_Z]),
            ]
            
            self.map_cmd_pub_.publish(self.map_cmd_)
            return response
        if code == 'SAVE':
            response.value = 0
            os.makedirs(os.path.dirname(os.path.join(self.default_path_, self.active_config_)), exist_ok=True)
            self.save_to_yaml(self.default_path_, self.active_config_)
            return response

        # If the service gets here, the request could not be processed
        self.get_logger().warn(f'Config managing service could not process request {request.data}')
        response.success = False
        response.value = 0
        return response

    def save_to_yaml(self, path = '', file_name = ''):
        '''
        Saves a yaml file to a specified path and with a specified file name
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

        self.get_logger().info(f'Saving current parameter configuration at {os.path.join(path, file_name)}')
            
        with open(os.path.join(path, file_name), 'w') as yaml_file:
            yaml.dump(self.param_vals, yaml_file, default_flow_style = False)

    def load_from_yaml(self, path = '', file_name = ''):
        '''
        Reads the file with the specified file_name from the path as a dictionary
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
                self.param_vals = loaded_data
            else:
                self.get_logger().warn('Invalid YAML file format..')

    def __set_value(self, param, value):
        '''
        Sets the selected parameter to the parsed value
        '''
        self.get_logger().info(f'Set parameter {param} to {value}')
        self.param_vals[param] = value

    def __get_value(self, param):
        '''
        Returns a value of a selected parameter
        '''
        return self.param_vals[param]

def main(args = None):
    rclpy.init(args = args)

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