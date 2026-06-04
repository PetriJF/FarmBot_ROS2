from rclpy.node import Node
from std_msgs.msg import String

class Parameters:
    def __init__(self, node: Node):
        self.node = node
        
        # Used for reading and writing to the FarmBot Parameters
        self.paramHandler = String() 
        # Parameter Command Publisher
        self.paramCmdPub = self.node.create_publisher(String, 'farmbot_command', 10)

    ## Parameter Handling Commands

    def readParam(self, param = int):
        '''
        Read the value on parameter {param}.

        Args:
            param {Int}: Parameter in question
        '''
        self.parameterHandler(list = False, write = False, read = True, update = False, param = param)

    def listAllParams(self):
        '''
        List all the parameters and their values
        '''
        self.parameterHandler(list = True, write = False, read = False, update = False)

    def writeParam(self, param = int, value = int):
        '''
        Write {value} to parameter {param}

        Args:
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        '''
        self.parameterHandler(list = False, write = True, read = False, update = False, param = param, value = value)
    
    def updateParam(self, param = int, value = int):
        '''
        Update parameter {param} with {value}.

        Args:
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        '''
        self.parameterHandler(list = False, write = False, read = False, update = True, param = param, value = value)

    def parameterHandler(self, list = bool, write = bool, read = bool, update = bool, param = int, value = int):
        '''
        Function for parameter handling commands.

        Args:
            list {bool}: If true, all the parameters will be listed
            write {bool}: If true, value V will be written to parameter P
            read {bool}: If true, parameter P will be listed
            update {bool}: If true, parameter P will be updated with value V (e.g. during calib.)
            param {Int}: Parameter in question
            value {Int}: Value written to param if write or update modes are active
        '''
        
        self.paramHandler.data = 'parameter_command ' + str(list) + ' ' + str(write) + ' ' + str(read) + ' ' + str(update) + ' ' + str(param) + ' ' + str(value)

        self.paramCmdPub.publish(self.paramHandler)