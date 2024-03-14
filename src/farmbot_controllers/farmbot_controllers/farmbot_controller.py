#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from farmbot_interfaces.msg import PlantManage
from farmbot_interfaces.srv import ParameterConfig, StringRepReq

# Modules
from farmbot_controllers.tools import ToolCommands
from farmbot_controllers.movement import Movement
from farmbot_controllers.states import State
from farmbot_controllers.devices import DeviceControl

class FarmbotControl(Node):
    # Node contructor
    def __init__(self):
        super().__init__('FarmbotController')

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
        self.input_sub_ = self.create_subscription(String, 'input_topic', self.cmd_interp_callback, 10)

        # UART Rx Subscriber
        self.uart_rx_sub_ = self.create_subscription(String, 'uart_receive', self.uart_feedback_callback, 10)

        # Map publishers
        self.plant_conf_ = PlantManage()
        self.plant_manage_pub_ = self.create_publisher(PlantManage, 'plant_mng', 10)
        self.plant_cmd_pub_ = self.create_publisher(String, 'plant_cmd', 10)

        # Log the initialization
        self.get_logger().info('Farmbot Controller Initialized..')

    def cmd_interp_callback(self, cmd = String):
        code = cmd.data.split(' ')
        match code[0]:
            case 'e':
                self.get_logger().info('CLEARING SEQUENCE')
                self.tools_.clear_sequence()
            case 'w':
                self.cur_x_ += self.cur_increment_
                self.mvm_.move_gantry_abs(x_coord = self.cur_x_ + self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'a':
                self.cur_y_ -= self.cur_increment_
                self.mvm_.move_gantry_abs(x_coord = self.cur_x_, y_coord = self.cur_y_ - self.cur_increment_, z_coord = self.cur_z_)
            case 's':
                self.cur_x_ -= self.cur_increment_
                self.mvm_.move_gantry_abs(x_coord = self.cur_x_ - self.cur_increment_, y_coord = self.cur_y_, z_coord = self.cur_z_)
            case 'd':
                self.cur_y_ += self.cur_increment_
                self.mvm_.move_gantry_abs(x_coord = self.cur_x_, y_coord = self.cur_y_ + self.cur_increment_, z_coord = self.cur_z_)
            case '1':
                self.cur_increment_ = 10.0
            case '2':
                self.cur_increment_ = 100.0
            case '3':
                self.cur_increment_ = 500.0
            case 'C_1': # Load config
                if code[1]:
                    self.config_loader_client(ver = code[1])
                else:
                    self.get_logger().warn('No parameter config set')
            case 'h': 
                self.mvm_.go_home()
            case 'j':
                self.mvm_.find_axis_home(x = True)
            case 'k':
                self.mvm_.find_axis_home(y = True)
            case 'l':
                self.mvm_.find_axis_home(z = True)
            case 'fh':
                self.mvm_.find_all_homes()
            case 'c':
                self.mvm_.calibrate_all_axis()
            case 'v':
                self.mvm_.calibrate_axis(x = True)
            case 'b':
                self.mvm_.calibrate_axis(y = True)
            case 'n':
                self.mvm_.calibrate_axis(z = True)
            case 'o':
                self.param_config_client(cmd = 'SAVE')
            case 'p':
                self.param_config_client(cmd = 'MAP')
            ## Tool commands
            case 'T_1_0' | 'T_2_0' | 'T_3_0': # e.g. T_1_0 Seeder 1198.0 332.4 -240.0 1
                tool = code[0] + '\n' + code[1] + '\n' + code[2] + ' ' + code[3] + ' ' + code[4] + ' ' + code[5]
                self.tools_.map_cmd_client(cmd = tool)
            case 'T_1_1' | 'T_1_2' | 'T_2_1' | 'T_2_2' | 'T_3_1' | 'T_3_2':
                self.tools_.map_cmd_client(cmd = cmd.data)
            ## Plant commands
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
            case 'P_2':
                self.plant_conf_.add = False
                self.plant_conf_.autopos = False
                self.plant_conf_.x = -1.0
                self.plant_conf_.y = -1.0
                self.plant_conf_.z = -1.0
                self.plant_conf_.exclusion_radius = -1.0
                self.plant_conf_.canopy_radius = -1.0
                self.plant_conf_.water_quantity = -1.0
                self.plant_conf_.max_z = -1.0
                self.plant_conf_.plant_name = ''
                self.plant_conf_.growth_stage = ''
                self.plant_conf_.remove = True
                self.plant_conf_.index = int(code[1])

                self.plant_manage_pub_.publish(self.plant_conf_)
            case 'P_3' | 'P_4': # Seed/water all plants in Planning stage
                self.tools_.map_cmd_client(cmd = cmd.data)
            ## Seed Tray commands
            case 'S_1_0' | 'S_2_0' | 'S_3_0': # e.g. S_1_0 0 Tray1 Radish 1198.0 332.4 -240.0
                tray = (code[0] + (('_' + code[1]) if code[1] in ['0', '1'] else '_0') 
                        + '\n' + code[2] + '\n' + code[3] + '\n' 
                        + code[4] + ' ' + code[5] + ' ' + code[6])
                self.tools_.map_cmd_client(cmd = tray)
            case 'I_0': # Calibrate Camera
                self.tools_.cam_calib_client(cmd = 'GET')
            case 'I_1': # Panorama Sequencing
                self.tools_.stitch_panorama_client(calib = False, update_map = False,
                                                    x = self.cur_x_, y = self.cur_y_,
                                                    z = self.cur_z_)
            ## Device commands
            case 'D_L_1' | 'D_L_0':
                self.tools_.led_strip(state = int(code[0][4]))
            case 'D_V_0' | 'D_V_1':
                self.tools_.vacuum_pump(state = int(code[0][4]))
            case 'D_W_0' | 'D_W_1':
                self.tools_.water_pump(state = int(code[0][4]))
            case 'D_C':
                self.devices_.read_pin(63, False)

    ## UART Handling Callback
    
    def uart_feedback_callback(self, msg = String):
        '''
        Takes the feedback from the Serial Receiver and updates
        information accordingly 
        '''
        msgSplit = (msg.data).split(' ')
        reportCode = msgSplit[0]

        if reportCode == 'R82':
            self.cur_x_ = float(msgSplit[1][1:])
            self.cur_y_ = float(msgSplit[2][1:])
            self.cur_z_ = float(msgSplit[3][1:])
            
            # Update the position reference within the sequencing module
            self.tools_.x = self.cur_x_
            self.tools_.y = self.cur_y_
            self.tools_.z = self.cur_z_
        elif reportCode == 'R41':
            self.tools_.uart_message(msg.data)
    
    ## Parameter Manager Clients and Future Callbacks

    def config_loader_client(self, ver: str):
        '''
        Client for the Parameter Loading Server
        Requests the server to load the parameter configuration appropriate for
        the model of the farmbot used.
        '''
        if ver == '':
            self.get_logger().warn('IGNORED. Cannot set configuration if the version of the farmbot is not set!')
            return
        if ver not in ['Genesis', 'genesis', 'Gen', 'gen',
                       'Express', 'express', 'Exp', 'exp',
                       'Custom', 'custom']:
            self.get_logger().warn('IGNORED. Config type unrecognized')
            return
        

        client = self.create_client(StringRepReq, 'load_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Parameter Loading Server...')
        
        request = StringRepReq.Request()
        request.data = ver

        future = client.call_async(request = request)
        future.add_done_callback(self.config_loading_callback)

    def config_loading_callback(self, future):
        '''
        Future Callback from the parameter loading server
        Checks if the parameter loading server executed correctly
        '''
        try:
            response = future.result().data
            if response == 'FAILED':
                self.get_logger().warn('Failure in Parameter Config Loading!')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    def param_config_client(self, cmd: String):
        '''
        Parameter Configuration Client
        Requests a response from the Parameter Manager Server
        '''
        client = self.create_client(ParameterConfig, 'manage_param_config')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for Parameter Config Server...')

        request = ParameterConfig.Request()
        request.data = cmd

        future = client.call_async(request = request)
        future.add_done_callback(self.param_config_callback)

    def param_config_callback(self, future):
        '''
        Parameter information request future callback
        Gets the response from the request sent to the parameter config manager
        '''
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Failure in Parameter Config Handling!')
                return
            
            if future.result().cmd:
                info = future.result().cmd.split(' ')
                if info[0] == 'MAP':
                    self.tools_.stitch_panorama_client(calib = False, update_map = True,
                                                       x = float(info[2]), y = float(info[4]),
                                                       z = float(info[6]))
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))
    

def main(args = None):
    rclpy.init(args = args)

    main_ctrl_node = FarmbotControl()
    
    try:
        rclpy.spin(main_ctrl_node)
    except KeyboardInterrupt:
        main_ctrl_node.destroy_node()

    main_ctrl_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()