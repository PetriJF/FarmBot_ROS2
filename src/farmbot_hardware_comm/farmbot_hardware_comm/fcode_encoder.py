from rclpy.node import Node
from farmbot_interfaces.msg import StatusCommand

class DeviceCmdHandler :
    '''
    Module handling device and peripheral commands.
    '''
    def __init__(self, node: Node):
        self.node = node
        self.uart_cmd = str()

        # Log the initialization
        self.node.get_logger().info('Device Command Handler Initialized..')

    def water_cmd(self, command : list):
        '''
        Watering style command. Used to set the watering to be either time based (1) or
        measured using a flow meter (2)
        '''
        
        # Valid watering commands
        valid_commands = (1, 2)

        if int(command[0]) in valid_commands:
            if int(command[1]) <= 0:
                self.node.get_logger().warning('The time constraint/volume constraint was not set!')
            else:
                self.uart_cmd = 'F0' + command[0] + (' T' if command[0] == '1' else ' N') + command[1]

                self.node.get_logger().info(self.uart_cmd)
                return self.uart_cmd
        else:
            self.node.get_logger().error('Wrong watering command type! First element should be 1 (timed pulses msec) or 2 (volume pulses)!')

    def i2c_cmd(self, command : list):
        '''
        I2C Command Handler. Used to read or write to the I2C devices connected
        to the farmduino
        '''

        if command[0] == 'True':    # I2C SET
            self.uart_cmd = 'F51 E' + command[1] + ' P' + command[2] + ' V' + command[3]
        else:           # I2C READ
            self.uart_cmd = 'F52 E' + command[1] + ' P' + command[2]

        self.node.get_logger().info(self.uart_cmd)
        return self.uart_cmd

    def pin_cmd(self, command : list):
        '''
        Pin Command Handler. Handles reading (F42) and setting (F41, F43, F44)
        pins. Note that this covers both analog and digital pins, and the pin
        mode needs to be set accordingly!
        '''

        # Check if in set mode, but no set command selected
        if command[0] == 'True' and command[1] == 'False' and command[2] == 'False' and command[3] == 'False':
            self.node.get_logger().error('Pin is in SET mode, but no SET CASE was selected. Use set_io, set_value or set_value2 to indicate\
                                        what set mode you want!')
        else:
            # SET mode for the pin
            if command[0] == 'True':
                if command[1] == 'True':      # NOTE: pin_mode should be 0 for input and 1 for output
                    self.uart_cmd = 'F43 P' + command[4] + ' M' + command[8]
                elif command[2] == 'True': # NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd = 'F41 P' + command[4] + ' V' + command[5] + ' M' + command[8]
                elif command[3] == 'True':# NOTE: pin_mode should be 0 for digital and 1 for analog
                    self.uart_cmd = 'F44 P' + command[4] + ' V' + command[5] + ' W' + command[6] + ' T' + command[7] + ' M' + command[8]
            # READ mode for the pin
            else:
                self.uart_cmd = 'F42 P' + command[5] + ' M' + command[8]

            self.node.get_logger().info(self.uart_cmd)
            return self.uart_cmd
        

class MotorCmdHandler :
    '''
    Module handling motor and actuator commands.
    '''
    def __init__(self, node: Node):
        self.node = node
        self.uart_cmd = str()

        # Log the initialization
        self.node.get_logger().info('Motor Command Handler Initialized..')

    def gantry_cmd(self, command : list):
        '''
        Handling gantry commands. Note that homing and calibration must be done through the
        home_handler topic
        '''

        mode = command[0]
        # Getting the target location and the speed
        x, y, z = command[1], command[2], command[3]
        max_x, max_y, max_z = command[4], command[5], command[6]

        # Form the GCode command for the "move at location" action
        self.uart_cmd = ('G01 ' if mode == 'True' else 'G00 ') + 'X' + x + ' Y' + y + ' Z' + z \
                            + ('' if mode == 'True' else (' A' + max_x + ' B' + max_y + ' C' + max_z))

        self.node.get_logger().info(self.uart_cmd)
        return self.uart_cmd

    def home_cmd(self, command : list):
        '''
        Handling homing and calibration commands
        '''

        if command[0] == 'True':            # Home all axis
            self.uart_cmd = 'G28'
        elif command[1] == 'True': # Set the current position as home
            self.uart_cmd = 'F84 ' + 'X' + ("1 " if command[3]=='True' else '0 ')\
                                         + 'Y' + ("1 " if command[4]=='True' else '0 ')\
                                         + 'Z' + ("1" if command[5]=='True' else '0')
        else:
            if command[3] == 'False' and  command[4] == 'False' and command[5] == 'False':
                self.node.get_logger().error('No axis selected for homing/calibration!')
            else:
                if command[3] == 'True' :              # Find home or calibrate x axis
                    self.uart_cmd = 'F11' if command[2]=='False' else 'F14'
                if command[4] == 'True':            # Find home or calibrate y axis
                    self.uart_cmd = 'F12' if command[2]=='False' else 'F15'
                if command[5] == 'True':            # Find home or calibrate z axis
                    self.uart_cmd = 'F13' if command[2]=='False' else 'F16'

        self.node.get_logger().info(self.uart_cmd)
        return self.uart_cmd

    def servo_cmd(self, command : list):
        '''
        Handles servo command interpretation. Note that servos can be attached only on pins
        4, 5, 6 and 11 on the Farmduino
        '''

        # The valid servo pins
        valid_servo_pins = (4, 5, 6, 11)
        
        # If the servo pin is within the valid pins:
        if (int(command[0]) in valid_servo_pins):
            # Request servo attached to PIN to be rotated to ANGLE
            self.uart_cmd = 'F61 P' + command[0] + ' V' + command[1]

            self.node.get_logger().info(self.uart_cmd)
            return self.uart_cmd
        else:
            self.node.get_logger().error(command[0] + ' is not within the valid servo pins(4, 5, 6, 11)!')
        

class StateCmdHandler :
    '''
    Module handling commands for manipulating the farmbot's parameters, state and status
    '''
    def __init__(self, node: Node):
        self.node = node
        self.uart_cmd = str()

        # Log the initialization
        self.node.get_logger().info('State Command Handler Initialized..')

    def param_cmd(self, command : list):
        '''
        Parameter Command Handler. Used to manipulate the Farmduino
        parameters by manipulating code friendly commands into the
        GCode (FCode) commands utilized in the UART communication
        scheme
        '''
        
        if command[0] == 'True':        # List all parameters 
            self.uart_cmd = 'F20'
        else:
            if command[2] == 'True':    # Read a parameter
                self.uart_cmd = 'F21 P' + command[4]
            elif command[1] == 'True': # Write to a parameter
                self.uart_cmd = 'F22 P' + command[4] + ' V' + command[5]
            elif command[3] == 'True': # Update a parameter (in the calibration state)
                self.uart_cmd = 'F23 P' + command[4] + ' V' + command[5]
                
        self.node.get_logger().info(self.uart_cmd)
        return self.uart_cmd

    def state_cmd(self, command : list):
        '''
        State Command Handler. Used as the main pipeline for specific commands.
        E - End Stop, @ - Abort Movement, F09 - Report Endstop, F81 - Report
        Current Position and F83 - Report Software Version
        '''
        state_counter = command.count('True')
        
        # Check that only one start command is in effect
        if state_counter != 1:
            self.node.get_logger().error('Make sure to include only 1 state handler command! Your input has ' + str(state_counter) + ' commands')
        else:
            if command[0] == 'True':               # Electronic stop
                self.uart_cmd = 'E'
            elif command[1] == 'True':    # Abort current movement command
                self.uart_cmd = '@'
            elif command[2] == 'True':       # Reset the Electronic stop
                self.uart_cmd = 'F09'
            elif command[3] == 'True':      # Report End Stop
                self.uart_cmd = 'F81'
            elif command[4] == 'True':      # Report Current Position
                self.uart_cmd = 'F82'
            elif command[5] == 'True':        # Report Software Version
                self.uart_cmd = 'F83'

            self.node.get_logger().info(self.uart_cmd)
            return self.uart_cmd

    def status_cmd(self, cmd: StatusCommand):
        '''
        Function that allows the read and write of a status command for the
        Farmbot. Check documentation for more information on status commands
        '''
        if cmd.mode:    # Write mode
            self.uart_cmd = 'F32 P' + str(cmd.p) + ' V' + str(cmd.v)
        else:           # Read mode
            self.uart_cmd = 'F31 P' + str(cmd.p)

        self.node.get_logger().info(self.uart_cmd)
        return self.uart_cmd