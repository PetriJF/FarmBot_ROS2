from rclpy.node import Node
from farmbot_interfaces.msg import PinCommand, I2CCommand
from std_msgs.msg import Int64MultiArray

class DeviceControl:
    '''
    ROS2 Python Module that enables the control of the different devices
    that are connected to the Farmbot
    '''
    def __init__(self, node: Node):
        self.node_ = node
        # Variables used to store the device commands
        self.pin_cmd_ = PinCommand()
        self.water_cmd_ = Int64MultiArray()
        self.i2c_cmd_ = I2CCommand()


        # Publishers for the command types
        self.pin_pub_ = self.node_.create_publisher(PinCommand, 'pin_command', 10)
        self.i2c_pub_ = self.node_.create_publisher(I2CCommand, 'i2c_command', 10)
        self.water_pub_ = self.node_.create_publisher(Int64MultiArray, 'water_command', 10)

    ## I2C Control Handlers
    
    def i2c_read(self, pin: int, element: int):
        '''
        Reading from an I2C device
        '''
        self.i2c_handler(self, mode = False, pin = pin, element = element, value = 0)

    def i2c_set(self, pin: int, element: int, value: int):
        '''
        Setting a value to an I2C device
        '''
        self.i2c_handler(self, mode = True, pin = pin, element = element, value = value)

    def i2c_handler(self, mode: bool, element: int, pin: int, value: int):
        '''
        Handler that enables the formation of the i2c command interface

        Args:
            mode {bool}: False for READ, True for SET
            p {int}: Pin number
            e {int}: Element in tool mount
            v {int}: Value number
        '''
        
        self.i2c_cmd_.mode = mode
        self.i2c_cmd_.e = element
        self.i2c_cmd_.p = pin
        self.i2c_cmd_.v = value

        self.i2c_pub_.publish(self.i2c_cmd_)

    ## Water Control Handlers
    ## NOTE: The documentation mentions that the commands are not implemented. Need to invesigate
    
    def water_command(self, mode: bool, unit: int):
        '''
        The water control commands. NOTE: The documentation mentions that the commands
        are not implemented! Did not check if this is true, but added the command handling
        anyway if support will be added to the arduino firmware.

        Args:
            mode {bool}: False for time based watering, True for valume flow based watering
            unit {int}: The amount of time (in time based watering) in millisec. or the pulse
                        count of the flow meter.
        '''

        self.water_cmd_.data = [int(mode), unit]
        self.water_pub_.publish(self.water_cmd_)

    ## Pin Control Handlers

    def set_pin_value(self, pin: int, value: int, pin_mode: bool):
        '''
        Setting a farmduino pin to a selected value and pin_mode

        Args:
            pin {int}: The pin the value is set on
            value {int}: Value to set
            pin_mode {bool}: 0 for digital, 1 for analog
        '''
        self.manipulate_pin(mode = True, set_value1 = True, pin = pin, value1 = value, pin_mode = pin_mode)
        
    def set_pin_value_2(self, pin: int, value1: int, delay: int, value2: int, pin_mode: bool):
        '''
        Setting value1 to pin and waiting for delay milliseconds. After the delay,
        value2 is set. Both sets are done in the selected pin_mode (0 for digital
        and 1 for analog)

        Args:
            pin {int}: The pin the value is set on
            value1 {int}: First value to set
            delay {int}: delay between the 2 values in milliseconds
            value2 {int}: Second value to set
            pin_mode {bool}: 0 for digital, 1 for analog
        '''
        self.manipulate_pin(mode = True, set_value2 = True, pin = pin, value1 = value1,
                            delay = delay, value2 = value2, pin_mode = pin_mode)

    def read_pin(self, pin: int, pin_mode: bool):
        '''
        Read the value from the selected pin in the parsed pin_mode

        Args:
            pin {int}: The pin to read the value from
            pin_mode{bool}: 0 for digital, 1 for analog
        '''
        self.manipulate_pin(mode = False, pin = pin, pin_mode = pin_mode)

    def set_pin_io(self, pin: int, io_mode: bool):
        '''
        Setting the IO of the selected pin

        Args:
            pin {int}: the pin to set the IO for
            io_mode(bool): 0 for input, 1 for output
        '''
        self.manipulate_pin(mode = True, set_io = True, pin = pin, pin_mode = io_mode)

    def manipulate_pin(self, mode: bool = False, set_io: bool = False, set_value1: bool = False, set_value2: bool = False,
                       pin: int = 0, value1: int = 0, value2: int = 0, delay: int = 0, pin_mode: bool = False):
        '''
        Creates the pin command used to handle and translate to the specific FarmBot
        commands.

        Args:
            mode{bool}: 1 for SET, 0 for READ
            set_io{bool}: If mode = True, and set_IO, the IO state of the pin will be set
            set_value1{bool}: If mode = True, and set)value, the value on the pin will be set
            set_value2{bool}: If mode and set_value2 are true, the double value set command will be used
            pin{int}: PIN worked on
            value1{int}: value to write on pin
            value2{int}: a second value to set on the pin after a delay
            delay{int}: time delay in millis
            pin_mode{bool}: (0-digital / 1-analog) OR (0-input / 1-output)
        '''
        
        self.pin_cmd_.mode = mode
        self.pin_cmd_.set_io = set_io
        self.pin_cmd_.set_value = set_value1
        self.pin_cmd_.set_value2 = set_value2
        self.pin_cmd_.pin = pin
        self.pin_cmd_.value = value1
        self.pin_cmd_.value2 = value2
        self.pin_cmd_.delay = delay
        self.pin_cmd_.pin_mode = pin_mode

        self.pin_pub_.publish(self.pin_cmd_)


