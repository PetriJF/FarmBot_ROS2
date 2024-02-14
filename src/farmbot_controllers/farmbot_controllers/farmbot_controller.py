#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from farmbot_interfaces.msg import PlantManage
from farmbot_interfaces.srv import ParameterConfig, LoadParamConfig, StringRepReq

# Modules
from farmbot_controllers.tools import ToolCommands
from farmbot_controllers.movement import Movement
from farmbot_controllers.states import State
from farmbot_controllers.devices import DeviceControl

import time

class KeyboardTeleOp(Node):
    # Node contructor
    def __init__(self):
        super().__init__("FarmBotController")

        # Initializing movemement module
        self.mvm_ = Movement(self)
        # Initializing the state module
        self.state_ = State(self)
        # Initializing the devices and peripherals modules
        self.devices_ = DeviceControl(self)
        # Initializing the tool module
        self.tools_ = ToolCommands(self, self.mvm_, self.devices_)

        # Memory
        self.cur_x_ = 0.0
        self.cur_y_ = 0.0
        self.cur_z_ = 0.0

        # Temporary Keyboard subscriber
        self.cur_increment_ = 10.0
        self.inputSub_ = self.create_subscription(String, 'input_topic', self.commandInterpretationCallback, 10)

        # UART Rx Subscriber
        self.uartRxSub_ = self.create_subscription(String, 'uart_receive', self.farmbotFeedbackCallback, 10)

        # Map publishers
        self.plant_conf_ = PlantManage()
        self.plant_manage_pub_ = self.create_publisher(PlantManage, 'plant_mng', 10)
        self.plant_cmd_pub_ = self.create_publisher(String, 'plant_cmd', 10)

        # Log the initialization
        self.get_logger().info("Farmbot Controller Initialized..")

    def commandInterpretationCallback(self, cmd = String):
        code = cmd.data.split(' ')
        match code[0]:
            case 'e':
                self.state_.electronicStop()
            case 'E':
                self.state_.resetElectronicStop()
            case 'w':
                self.cur_x_ += self.cur_increment_
                self.mvm_.moveGantryAbsolute(x_coord = self.cur_x_ + self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'a':
                self.cur_y_ -= self.cur_increment_
                self.mvm_.moveGantryAbsolute(x_coord = self.cur_x_, y_coord = self.cur_y_ - self.cur_increment_, z_coord = self.cur_z_)
            case 's':
                self.cur_x_ -= self.cur_increment_
                self.mvm_.moveGantryAbsolute(x_coord = self.cur_x_ - self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'd':
                self.cur_y_ += self.cur_increment_
                self.mvm_.moveGantryAbsolute(x_coord = self.cur_x_, y_coord = self.cur_y_ + self.cur_increment_, z_coord = self.cur_z_)
            case '1':
                self.cur_increment_ = 10.0
            case '2':
                self.cur_increment_ = 100.0
            case '3':
                self.cur_increment_ = 500.0
            case 'i':
                self.configLoaderClient(standard = False, ver = "labFB")
            case 'h': 
                self.mvm_.goHome()
            case 'j':
                self.mvm_.findAxisHome(x = True)
            case 'k':
                self.mvm_.findAxisHome(y = True)
            case 'l':
                self.mvm_.findAxisHome(z = True)
            case 'fh':
                self.mvm_.findAllHomes()
            case 'c':
                self.mvm_.calibrateAllLens()
            case 'v':
                self.mvm_.calibrateAxisLen(x = True)
            case 'b':
                self.mvm_.calibrateAxisLen(y = True)
            case 'n':
                self.mvm_.calibrateAxisLen(z = True)
            case 'o':
                self.parameterConfigClient(cmd = 'SAVE')
            case 'p':
                self.parameterConfigClient(cmd = 'MAP')
            case 'T_1_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'T_1_0\nSeeder\n1198.0 332.4 -240.0 1')
            case 'T_2_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'T_2_0\nSoil\n1198.0 432.2 -240.0 1')
            case 'T_3_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'T_3_0\nWater\n1198.0 532.2 -240.0 1')
            case 'T_1_1' | 'T_1_2' | 'T_2_1' | 'T_2_2' | 'T_3_1' | 'T_3_2':
                self.tools_.map_cmd_client(cmd = cmd.data)
            case 'LED1':
                self.tools_.led_strip_on()
            case 'LED2':
                self.tools_.led_strip_off()
            case 'VAC1':
                self.tools_.vacuum_pump_on()
            case 'VAC2':
                self.tools_.vacuum_pump_off()
            case 'WAT1':
                self.tools_.water_pump_on()
            case 'WAT2':
                self.tools_.water_pump_off()
            case 'P_1':
                self.plant_conf_.add = True
                self.plant_conf_.autopos = False
                self.plant_conf_.x = float(code[1])
                self.plant_conf_.y = float(code[2])
                self.plant_conf_.z = float(code[3])
                self.plant_conf_.exclusion_radius = float(code[4])
                self.plant_conf_.canopy_radius = float(code[5])
                self.plant_conf_.water_quantity = float(code[6])
                self.plant_conf_.max_z = float(code[7])
                self.plant_conf_.plant_name = code[8]
                self.plant_conf_.growth_stage = code[9]
                self.plant_conf_.remove = False
                self.plant_conf_.index = -1

                self.plant_manage_pub_.publish(self.plant_conf_)
            case 'P_3' | 'P_4': # Seed/water all plants in Planning stage
                self.tools_.map_cmd_client(cmd = cmd.data)
            
            case 'S_1_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'S_1_0_0\nTray1\nRadish\n1198.0 332.4 -240.0')
            case 'S_2_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'S_2_0_0\nTray2\nRadish\n1198.0 432.2 -240.0')
            case 'S_3_0': # new tool marked
                self.tools_.map_cmd_client(cmd = 'S_3_0_0\nTray3\nRadish\n1198.0 532.2 -240.0')


    ## UART Handling Callback
    def farmbotFeedbackCallback(self, msg = String):
        msgSplit = (msg.data).split(' ')
        reportCode = msgSplit[0]
        #if reportCode == 'R21' or reportCode == 'R23':
        #    self.parameterConfigClient(cmd = msg.data)
        if reportCode == 'R82':
            self.cur_x_ = float(msgSplit[1][1:])
            self.cur_y_ = float(msgSplit[2][1:])
            self.cur_z_ = float(msgSplit[3][1:])
        
    
    ## Parameter Loading Service Client
    def configLoaderClient(self, standard = True, ver = ""):
        if not standard and ver == "":
            self.get_logger().warn("Can't set non standard configuration if the version of the farmbot is not set!")
        
        client = self.create_client(LoadParamConfig, 'load_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Parameter Loading Server...")
        
        request = LoadParamConfig.Request()
        request.standard = standard
        request.ver = ver

        future = client.call_async(request = request)
        future.add_done_callback(self.callbackConfigLoading)

    def callbackConfigLoading(self, future):
        try:
            response = future.result()
            if not response:
                self.get_logger().warn("Failure in Parameter Config Loading!")
            
            ## TODO: Add a response to the future
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e, ))

    ## Parameter Config Service Client
    def parameterConfigClient(self, cmd = String):
        client = self.create_client(ParameterConfig, 'manage_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Parameter Config Server...")

        request = ParameterConfig.Request()
        request.data = cmd

        future = client.call_async(request = request)
        future.add_done_callback(self.callbackParamConfig)

    def callbackParamConfig(self, future):
        try:
            response = future.result()
            if not response:
                self.get_logger().warn("Failure in Parameter Config Handling!")
            
            ## TODO: Add a response to the future

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e, ))
    

def main(args = None):
    rclpy.init(args = args)

    keyboardTeleOpNode = KeyboardTeleOp()
    
    try:
        rclpy.spin(keyboardTeleOpNode)
    except KeyboardInterrupt:
        keyboardTeleOpNode.destroy_node()

    keyboardTeleOpNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()