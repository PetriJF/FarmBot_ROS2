import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import ParameterList
from ament_index_python.packages import get_package_share_directory

import os
import time
import yaml

from std_msgs.msg import String
from farmbot_interfaces.msg import ParameterCommand
from farmbot_interfaces.srv import ParameterConfig, LoadParamConfig

class ConfigServer(Node):
    # Node contructor
    def __init__(self):
        super().__init__("ConfigServer")
        self.firmwareInitialized_ = False

        # The dictionary containing all of the parameters for the farmbot
        self.parameterValues = {
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
        
        self.defaultPath_ = os.path.join(
            get_package_share_directory('farmbot_controllers'),
            'config'
        )
        self.defaultFirmwareConfig_ = 'firmwareDefault.yaml'
        self.defaultConfig_ = 'defaultParameterConfig.yaml'
        self.labConfig_ = 'labFB.yaml'
        self.activeConfig_ = 'activeConfig.yaml'

        self.configServer_ = self.create_service(ParameterConfig, 'manage_param_config', self.configRequestCallback)
        self.loadingServer_ = self.create_service(LoadParamConfig, 'load_param_config', self.parameterLoadingRequest)

        self.paramCmd_ = ParameterCommand()
        self.paramCmdPub_ = self.create_publisher(ParameterCommand, 'parameter_command', 10)

        # UART Rx Subscriber
        self.uartRxSub_ = self.create_subscription(String, 'uart_receive', self.farmbotFeedbackCallback, 10)

        # Log the initialization
        self.get_logger().info("Config Server Initialized..")

    ## UART Handling Callback
    def farmbotFeedbackCallback(self, msg = String):
        msgSplit = (msg.data).split(' ')
        reportCode = msgSplit[0]
        if msg.data == 'R99 ARDUINO STARTUP COMPLETE' and not self.firmwareInitialized_:
            self.firmwareInitialized_ = True
            self.retrieveConfig()
        if reportCode == 'R21' or reportCode == 'R23':
            self.set_value(int(float(msgSplit[1][1:])), int(float(msgSplit[2][1:])))
            self.get_logger().info(f"Updated parameter {msgSplit[1]} to {msgSplit[2]}")

    def retrieveConfig(self):
        activeConfig = os.path.join(self.defaultPath_, self.activeConfig_)
        if os.path.exists(activeConfig):
            self.load_from_yaml(self.defaultPath_, self.activeConfig_)
            self.loadParameters()
            self.get_logger().info("Initialized with active config from previous run")
        else:
            self.get_logger().warn("Previous config could not be found! You will need to initialize the appropriate parameter config")

    def save_to_yaml(self, path = '', fileName = ''):
        if path == '':
            self.get_logger().warn("Path not set for retrieving the parameter config file")
            return
        if fileName == '':
            self.get_logger().warn("Parameter Config File name not set")
            return
        if not os.path.exists(path):
            self.get_logger().warn("File path is invalid")
            return

        self.get_logger().info(f"Saving current parameter configuration at {os.path.join(path, fileName)}")
            
        with open(os.path.join(path, fileName), 'w') as yaml_file:
            yaml.dump(self.parameterValues, yaml_file, default_flow_style = False)

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
                self.parameterValues = loaded_data
            else:
                self.get_logger().warn("Invalid YAML file format..")

    def set_value(self, param, value):
        self.get_logger().info(f"Set parameter {param} to {value}")
        self.parameterValues[param] = value
    
    def get_value(self, param):
        return self.parameterValues[param]

    def parameterLoadingRequest(self, request, response):
        # A standard config that works for all farmbots
        if request.standard: 
            self.load_from_yaml(path = self.defaultPath_, fileName = self.defaultConfig_)
            self.loadParameters()
        # A configuration more specific to the model you are running
        else:
            ver = request.ver ## TODO add support for multiple configurations
            self.load_from_yaml(path = self.defaultPath_, fileName = self.labConfig_)
            self.loadParameters()

        response.success = True
        return response

    def loadParameters(self):
        self.paramCmd_.list = False
        self.paramCmd_.write = True
        self.paramCmd_.read = False
        self.paramCmd_.update = False

        # Loading only the parameters that are not loaded to the desired value (Greatly increases upload speed)
        with open(os.path.join(self.defaultPath_, self.defaultFirmwareConfig_), 'r') as yaml_file:
            loaded_firmware_config = yaml.safe_load(yaml_file)
            for key, value in self.parameterValues.items():
                if(loaded_firmware_config[key] != value):
                    self.paramCmd_.param = key
                    self.paramCmd_.value = value
                    self.paramCmdPub_.publish(self.paramCmd_)
                    time.sleep(0.1)
                    #self.get_logger().info(f"F22 P{key} V{value}")

        self.get_logger().info("Parameter loading complete!")

    def configRequestCallback(self, request, response):
        response.success = True    # success until proven otherwise
        response.cmd = request.data
        msgSplit = (request.data).split(' ')
        code = msgSplit[0]
        # Using both command and report codes so that the commands themselves
        # can be just fed into it with ease
        if code == 'R21' or code == 'R23' or code == 'F22' or code == 'F23':
            self.set_value(int(msgSplit[1][1:]), int(msgSplit[2][1:]))
            response.value = 0
            return response
        if code == 'F21':
            response.value = self.get_value(msgSplit[1][1:])
            return response

        # Requests with non farmbot commands
        if code == 'S': # Format S PARAM_INDEX PARAM_VALUE. e.g. S 2 1
            self.set_value(msgSplit[1], msgSplit[2])
            response.value = 0
            return response
        if code == 'G':
            response.value = self.get_value(msgSplit[1])
            return response
        if code == 'SAVE':
            response.value = 0
            os.makedirs(os.path.dirname(os.path.join(self.defaultPath_, self.activeConfig_)), exist_ok=True)
            self.save_to_yaml(self.defaultPath_, self.activeConfig_)
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