import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import ParameterList, MapCommand
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from farmbot_interfaces.msg import ParameterCommand
from farmbot_interfaces.srv import ParameterConfig, LoadParamConfig

import os
import time
import yaml

class ConfigServer(Node):
    '''
    Node handling the parameter recording and loading onto the farmduino
    '''
    # Node contructor
    def __init__(self):
        super().__init__('ConfigServer')
        # Flag waiting for initialization to be done before the config is loaded
        self.firmware_init_done_ = False

        # The dictionary containing all of the parameters for the farmbot
        self.param_vals = {
            ParameterList.PARAM_CONFIG_OK : 0,
            ParameterList.PARAM_USE_EEPROM : 0,
            ParameterList.PARAM_E_STOP_ON_MOV_ERR : 0,
            ParameterList.PARAM_MOV_NR_RETRY : 0,
            ParameterList.MOVEMENT_TIMEOUT_X : 0,
            ParameterList.MOVEMENT_TIMEOUT_Y : 0,
            ParameterList.MOVEMENT_TIMEOUT_Z : 0,
            ParameterList.MOVEMENT_KEEP_ACTIVE_X : 0,
            ParameterList.MOVEMENT_KEEP_ACTIVE_Y : 0,
            ParameterList.MOVEMENT_KEEP_ACTIVE_Z : 0,
            ParameterList.MOVEMENT_HOME_AT_BOOT_X : 0,
            ParameterList.MOVEMENT_HOME_AT_BOOT_Y : 0,
            ParameterList.MOVEMENT_HOME_AT_BOOT_Z : 0,
            ParameterList.MOVEMENT_INVERT_ENDPOINTS_X : 0,
            ParameterList.MOVEMENT_INVERT_ENDPOINTS_Y : 0,
            ParameterList.MOVEMENT_INVERT_ENDPOINTS_Z : 0,
            ParameterList.MOVEMENT_ENABLE_ENDPOINTS_X : 0,
            ParameterList.MOVEMENT_ENABLE_ENDPOINTS_Y : 0,
            ParameterList.MOVEMENT_ENABLE_ENDPOINTS_Z : 0,
            ParameterList.MOVEMENT_INVERT_MOTOR_X : 0,
            ParameterList.MOVEMENT_INVERT_MOTOR_Y : 0,
            ParameterList.MOVEMENT_INVERT_MOTOR_Z : 0,
            ParameterList.MOVEMENT_SECONDARY_MOTOR_X : 0,
            ParameterList.MOVEMENT_SECONDARY_MOTOR_INVERT_X : 0,
            ParameterList.MOVEMENT_STEPS_ACC_DEC_X : 0,
            ParameterList.MOVEMENT_STEPS_ACC_DEC_Y : 0,
            ParameterList.MOVEMENT_STEPS_ACC_DEC_Z : 0,
            ParameterList.MOVEMENT_STEPS_ACC_DEC_Z2 : 0,
            ParameterList.MOVEMENT_STOP_AT_HOME_X : 0,
            ParameterList.MOVEMENT_STOP_AT_HOME_Y : 0,
            ParameterList.MOVEMENT_STOP_AT_HOME_Z : 0,
            ParameterList.MOVEMENT_HOME_UP_X : 0,
            ParameterList.MOVEMENT_HOME_UP_Y : 0,
            ParameterList.MOVEMENT_HOME_UP_Z : 0,
            ParameterList.MOVEMENT_STEP_PER_MM_X : 0,
            ParameterList.MOVEMENT_STEP_PER_MM_Y : 0,
            ParameterList.MOVEMENT_STEP_PER_MM_Z : 0,
            ParameterList.MOVEMENT_MIN_SPD_X : 0,
            ParameterList.MOVEMENT_MIN_SPD_Y : 0,
            ParameterList.MOVEMENT_MIN_SPD_Z : 0,
            ParameterList.MOVEMENT_MIN_SPD_Z2 : 0,
            ParameterList.MOVEMENT_HOME_SPD_X : 0,
            ParameterList.MOVEMENT_HOME_SPD_Y : 0,
            ParameterList.MOVEMENT_HOME_SPD_Z : 0,
            ParameterList.MOVEMENT_MAX_SPD_X : 0,
            ParameterList.MOVEMENT_MAX_SPD_Y : 0,
            ParameterList.MOVEMENT_MAX_SPD_Z : 0,
            ParameterList.MOVEMENT_MAX_SPD_Z2 : 0,
            ParameterList.MOVEMENT_INVERT_2_ENDPOINTS_X : 0,
            ParameterList.MOVEMENT_INVERT_2_ENDPOINTS_Y : 0,
            ParameterList.MOVEMENT_INVERT_2_ENDPOINTS_Z : 0,
            ParameterList.MOVEMENT_MOTOR_CURRENT_X : 0,
            ParameterList.MOVEMENT_MOTOR_CURRENT_Y : 0,
            ParameterList.MOVEMENT_MOTOR_CURRENT_Z : 0,
            ParameterList.MOVEMENT_STALL_SENSITIVITY_X : 0,
            ParameterList.MOVEMENT_STALL_SENSITIVITY_Y : 0,
            ParameterList.MOVEMENT_STALL_SENSITIVITY_Z : 0,
            ParameterList.MOVEMENT_MICROSTEPS_X : 0,
            ParameterList.MOVEMENT_MICROSTEPS_Y : 0,
            ParameterList.MOVEMENT_MICROSTEPS_Z : 0,
            ParameterList.ENCODER_ENABLED_X : 0,
            ParameterList.ENCODER_ENABLED_Y : 0,
            ParameterList.ENCODER_ENABLED_Z : 0,
            ParameterList.ENCODER_TYPE_X : 0,
            ParameterList.ENCODER_TYPE_Y : 0,
            ParameterList.ENCODER_TYPE_Z : 0,
            ParameterList.ENCODER_MISSED_STEPS_MAX_X : 0,
            ParameterList.ENCODER_MISSED_STEPS_MAX_Y : 0,
            ParameterList.ENCODER_MISSED_STEPS_MAX_Z : 0,
            ParameterList.ENCODER_SCALING_X : 0,
            ParameterList.ENCODER_SCALING_Y : 0,
            ParameterList.ENCODER_SCALING_Z : 0,
            ParameterList.ENCODER_MISSED_STEPS_DECAY_X : 0,
            ParameterList.ENCODER_MISSED_STEPS_DECAY_Y : 0,
            ParameterList.ENCODER_MISSED_STEPS_DECAY_Z : 0,
            ParameterList.ENCODER_USE_FOR_POS_X : 0,
            ParameterList.ENCODER_USE_FOR_POS_Y : 0,
            ParameterList.ENCODER_USE_FOR_POS_Z : 0,
            ParameterList.ENCODER_INVERT_X : 0,
            ParameterList.ENCODER_INVERT_Y : 0,
            ParameterList.ENCODER_INVERT_Z : 0,
            ParameterList.MOVEMENT_AXIS_NR_STEPS_X : 0,
            ParameterList.MOVEMENT_AXIS_NR_STEPS_Y : 0,
            ParameterList.MOVEMENT_AXIS_NR_STEPS_Z : 0,
            ParameterList.MOVEMENT_STOP_AT_MAX_X : 0,
            ParameterList.MOVEMENT_STOP_AT_MAX_Y : 0,
            ParameterList.MOVEMENT_STOP_AT_MAX_Z : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_X : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_Y : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_Z : 0,
            ParameterList.MOVEMENT_AXIS_STEALTH_X : 0,
            ParameterList.MOVEMENT_AXIS_STEALTH_Y : 0,
            ParameterList.MOVEMENT_AXIS_STEALTH_Z : 0,
            ParameterList.MOVEMENT_CALIBRATION_DEADZONE_X : 0,
            ParameterList.MOVEMENT_CALIBRATION_DEADZONE_Y : 0,
            ParameterList.MOVEMENT_CALIBRATION_DEADZONE_Z : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_TOTAL_X : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_TOTAL_Y : 0,
            ParameterList.MOVEMENT_CALIBRATION_RETRY_TOTAL_Z : 0,
            ParameterList.PIN_REPORT_1_PIN_NR : 0,
            ParameterList.PIN_REPORT_2_PIN_NR : 0,
            ParameterList.PIN_GUARD_1_PIN_NR : 0,
            ParameterList.PIN_GUARD_1_TIME_OUT : 0,
            ParameterList.PIN_GUARD_1_ACTIVE_STATE : 0,
            ParameterList.PIN_GUARD_2_PIN_NR : 0,
            ParameterList.PIN_GUARD_2_TIME_OUT : 0,
            ParameterList.PIN_GUARD_2_ACTIVE_STATE : 0,
            ParameterList.PIN_GUARD_3_PIN_NR : 0,
            ParameterList.PIN_GUARD_3_TIME_OUT : 0,
            ParameterList.PIN_GUARD_3_ACTIVE_STATE : 0,
            ParameterList.PIN_GUARD_4_PIN_NR : 0,
            ParameterList.PIN_GUARD_4_TIME_OUT : 0,
            ParameterList.PIN_GUARD_4_ACTIVE_STATE : 0,
            ParameterList.PIN_GUARD_5_PIN_NR : 0,
            ParameterList.PIN_GUARD_5_TIME_OUT : 0,
            ParameterList.PIN_GUARD_5_ACTIVE_STATE : 0
        }
        
        self.default_path_ = os.path.join(
            get_package_share_directory('farmbot_controllers'),
            'config'
        )
        self.base_config_ = 'firmwareDefault.yaml'          # default config loaded by the firmware
        self.default_config_ = 'defaultParameterConfig.yaml'
        self.lab_fb_config_ = 'labFB.yaml'
        self.active_config_ = 'activeConfig.yaml'

        self.config_server_ = self.create_service(ParameterConfig, 'manage_param_config', self.config_request_server)
        self.config_loading_server_ = self.create_service(LoadParamConfig, 'load_param_config', self.param_loading_server)

        self.param_cmd_ = ParameterCommand()
        self.param_cmd_pub_ = self.create_publisher(ParameterCommand, 'parameter_command', 10)

        # UART Rx Subscriber
        self.uart_rx_sub_ = self.create_subscription(String, 'uart_receive', self.uart_rx_callback, 10)

        # Map updating publisher
        self.map_cmd_ = MapCommand()
        self.map_cmd_pub_ = self.create_publisher(MapCommand, 'map_cmd', 10)

        # Log the initialization
        self.get_logger().info('Config Server Initialized..')

    ## UART Handling Callback
    def uart_rx_callback(self, msg: String):
        msg_split = (msg.data).split(' ')
        reportCode = msg_split[0]
        if msg.data == 'R99 ARDUINO STARTUP COMPLETE' and not self.firmware_init_done_:
            self.firmware_init_done_ = True
            self.retrieve_config()
        if reportCode == 'R21' or reportCode == 'R23':
            self.set_value(int(float(msg_split[1][1:])), int(float(msg_split[2][1:])))
            self.get_logger().info(f'Updated parameter {msg_split[1]} to {msg_split[2]}')

    def retrieve_config(self):
        active_config_path = os.path.join(self.default_path_, self.active_config_)
        if os.path.exists(active_config_path):
            self.load_from_yaml(self.default_path_, self.active_config_)
            self.load_params()
            self.get_logger().info('Initialized with active config from previous run')
        else:
            self.get_logger().warn('Previous config could not be found! You will need to initialize the appropriate parameter config')

    def save_to_yaml(self, path = '', file_name = ''):
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

    def set_value(self, param, value):
        self.get_logger().info(f'Set parameter {param} to {value}')
        self.param_vals[param] = value
    
    def get_value(self, param):
        return self.param_vals[param]

    def param_loading_server(self, request, response):
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
            response.value = self.get_value(msg_split[1][1:])
            return response

        # Requests with non farmbot commands
        if code == 'S': # Format S PARAM_INDEX PARAM_VALUE. e.g. S 2 1
            self.set_value(msg_split[1], msg_split[2])
            
            return response
        if code == 'G':
            response.value = self.get_value(msg_split[1])
            return response
        if code == 'MAP':
            response.value = 0
            self.map_cmd_.sort = False
            self.map_cmd_.reindex = False
            self.map_cmd_.back_up = False
            self.map_cmd_.update = True
            self.map_cmd_.update_info = [
                'X ' + str(self.param_vals[ParameterList.MOVEMENT_AXIS_NR_STEPS_X] / 
                                self.param_vals[ParameterList.MOVEMENT_STEP_PER_MM_X]),
                'Y ' + str(self.param_vals[ParameterList.MOVEMENT_AXIS_NR_STEPS_Y] / 
                                self.param_vals[ParameterList.MOVEMENT_STEP_PER_MM_Y]),
                'Z ' + str(self.param_vals[ParameterList.MOVEMENT_AXIS_NR_STEPS_Z] / 
                                self.param_vals[ParameterList.MOVEMENT_STEP_PER_MM_Z]),
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


def main(args = None):
    rclpy.init(args = args)

    configServer = ConfigServer()

    try:
        rclpy.spin(configServer)
    except KeyboardInterrupt:
        pass
    finally:
        configServer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()