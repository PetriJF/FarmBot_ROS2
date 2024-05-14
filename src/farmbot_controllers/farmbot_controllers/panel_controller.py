#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from farmbot_interfaces.srv import LedPanelHandler
from farmbot_interfaces.msg import FBPanel

import RPi.GPIO as GPIO

class PanelController(Node):
    
    # Node contructor
    def __init__(self):
        super().__init__('PanelController')

        GPIO.setmode(GPIO.BCM)

        # LED Initializing
        GPIO.setup(FBPanel.ESTOP_LED, GPIO.OUT)
        GPIO.setup(FBPanel.UNLOCK_LED, GPIO.OUT)
        GPIO.setup(FBPanel.BUTTON_LED_A, GPIO.OUT)
        GPIO.setup(FBPanel.BUTTON_LED_B, GPIO.OUT)
        GPIO.setup(FBPanel.BUTTON_LED_C, GPIO.OUT)
        GPIO.setup(FBPanel.LED1, GPIO.OUT)
        GPIO.setup(FBPanel.LED2, GPIO.OUT)
        GPIO.setup(FBPanel.LED3, GPIO.OUT)
        GPIO.setup(FBPanel.LED4, GPIO.OUT)

        # Button Initialization
        GPIO.setup(FBPanel.BUTTON_ESTOP, GPIO.IN)
        GPIO.setup(FBPanel.BUTTON_UNLOCK, GPIO.IN)
        GPIO.setup(FBPanel.BUTTON_A, GPIO.IN)
        GPIO.setup(FBPanel.BUTTON_B, GPIO.IN)
        GPIO.setup(FBPanel.BUTTON_C, GPIO.IN)

        
        self.cmd_ = String()
        self.priority_pub_ = self.create_publisher(String, 'uart_transmit', 10)
        self.input_sub_ = self.create_subscription(String, 'keyboard_topic', self.command_callback, 10)
        self.input_pub_ = self.create_publisher(String, 'input_topic', 10)

        # LED Flasher Button
        self.flash_state_ = False
        flashing_frequency = 2.0
        self.leds_to_flash_ = []
        
        self.led_flasher_timer_ = self.create_timer(1.0 / flashing_frequency, self.LED_flasher)

        self.led_pin_list_ = [
            FBPanel.LED1,
            FBPanel.LED2,
            FBPanel.LED3,
            FBPanel.LED4,
            FBPanel.ESTOP_LED,
            FBPanel.UNLOCK_LED,
            FBPanel.BUTTON_LED_A,
            FBPanel.BUTTON_LED_B,
            FBPanel.BUTTON_LED_C
        ]
        self.led_panel_server_ = self.create_service(LedPanelHandler, 'set_led', self.LED_server)

        self.LED_client(FBPanel.ESTOP_LED, FBPanel.ON)
        self.LED_client(FBPanel.UNLOCK_LED, FBPanel.ON)

        # Log the initialization
        self.get_logger().info('Panel Controller Initialized..')

    ### Verifying user input for E_STOPS and RESETS

    def command_callback(self, cmd: String):
        if cmd.data == 'e':
            self.LED_client(FBPanel.ESTOP_LED, FBPanel.OFF)
            self.LED_client(FBPanel.UNLOCK_LED, FBPanel.FLASHING)
            self.cmd_.data = 'E'
            self.priority_pub_.publish(self.cmd_)
            self.get_logger().info('ESTOP button pressed')
        elif cmd.data == 'E':
            self.LED_client(FBPanel.ESTOP_LED, FBPanel.ON)
            self.LED_client(FBPanel.UNLOCK_LED, FBPanel.ON)
            self.cmd_.data = 'F09'
            self.priority_pub_.publish(self.cmd_)
            self.get_logger().info('RESET button pressed')
        self.input_pub_.publish(cmd)

        
    ### Service Server

    def LED_server(self, request, response):
        '''
        Service Server handling LED State manipulation.
        It sets the LEDS to ON, OFF or FLASHING.
        '''
        # Check if the LED Pin is correct
        if request.led_pin not in self.led_pin_list_:
            self.get_logger().warn('Selected LED pin is not recorded as having an LED attached')
            response.success = False
            return response
        # Check if an existing state was selected
        if request.state not in [FBPanel.ON, FBPanel.OFF, FBPanel.FLASHING]:
            self.get_logger().warn('Selected LED status is not recognized')
            response.success = False
            return response
        
        if request.state == FBPanel.ON:
            GPIO.output(request.led_pin, GPIO.HIGH)
            self.remove_flashing_led(request.led_pin)
        elif request.state == FBPanel.OFF:
            GPIO.output(request.led_pin, GPIO.LOW)
            self.remove_flashing_led(request.led_pin)
        elif request.state == FBPanel.FLASHING:
            self.add_flashing_led(request.led_pin)

        response.success = True
        return response

    def add_flashing_led(self, led_pin):
        '''
        Add a LED from the flashing LED list
        '''
        if led_pin not in self.leds_to_flash_:
            self.leds_to_flash_.append(led_pin)
    
    def remove_flashing_led(self, led_pin):
        '''
        Remove a LED from the flashing LED list
        '''
        if led_pin in self.leds_to_flash_:
            self.leds_to_flash_.remove(led_pin)

    ### Service Client

    def LED_client(self, led_pin, state):
        '''
        Service client for switching an LED on or off
        '''
        client = self.create_client(LedPanelHandler, 'set_led')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for LED Handling Server...')
        
        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = client.call_async(request = request)
        future.add_done_callback(self.LED_panel_callback)

    def LED_panel_callback(self, future):
        '''
        Service client callback once the LED switching server ends.
        '''
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Failure in LED Panel Handling!')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    ### LED states for the panel

    def LED_flasher(self):
        '''
        Timer Callback that flashes the LEDs in the self.leds_to_flash_ list.
        '''
        for led_pin in self.leds_to_flash_:
            GPIO.output(led_pin, GPIO.HIGH if self.flash_state_ else GPIO.LOW)
        self.flash_state_ = not self.flash_state_
    
    def estop_button_handler(self, channel):
        '''
        EStop Button Event Handler for the panel controller
        '''
        current_state = GPIO.input(FBPanel.BUTTON_ESTOP)
        if current_state == GPIO.LOW:
            self.LED_client(FBPanel.ESTOP_LED, FBPanel.OFF)
            self.LED_client(FBPanel.UNLOCK_LED, FBPanel.FLASHING)
            self.cmd_.data = 'E'
            self.priority_pub_.publish(self.cmd_)
            self.get_logger().info('ESTOP button pressed')

    def reset_button_handler(self, channel):
        '''
        Reset Button Event Handler for the panel controller
        '''
        current_state = GPIO.input(FBPanel.BUTTON_UNLOCK)
        if current_state == GPIO.LOW:
            self.LED_client(FBPanel.ESTOP_LED, FBPanel.ON)
            self.LED_client(FBPanel.UNLOCK_LED, FBPanel.ON)
            self.cmd_.data = 'F09'
            self.priority_pub_.publish(self.cmd_)
            self.get_logger().info('RESET button pressed')
    
    # Example of mapping Button A to something (E.g. Homing)
    def buttonAHandler(self, channel):
        current_state = GPIO.input(FBPanel.BUTTON_A)
        if current_state == GPIO.LOW:
             self.cmd_.data = 'H_0'
             self.input_pub_.publish(self.cmd_)
    
    # Example of mapping Button B to something (E.g. Calibrating)
    def buttonBHandler(self, channel):
        current_state = GPIO.input(FBPanel.BUTTON_B)
        if current_state == GPIO.LOW:
             self.cmd_.data = 'C_0'
             self.input_pub_.publish(self.cmd_)

    # Example of mapping Button C to something (E.g. Watering)
    def buttonCHandler(self, channel):
        current_state = GPIO.input(FBPanel.BUTTON_B)
        if current_state == GPIO.LOW:
             self.cmd_.data = 'P_4'
             self.input_pub_.publish(self.cmd_)


    def destroy_node(self):
        '''
        destroy_node overloading for cleaning up the GPIO
        '''
        # gpio cleanup
        GPIO.cleanup()


def main(args = None):
    rclpy.init(args = args)

    panel_node = PanelController()
    
    # GPIO Button
    GPIO.add_event_detect(FBPanel.BUTTON_ESTOP, GPIO.FALLING, callback=panel_node.estop_button_handler, bouncetime=200)
    GPIO.add_event_detect(FBPanel.BUTTON_UNLOCK, GPIO.FALLING, callback=panel_node.reset_button_handler, bouncetime=200)
    GPIO.add_event_detect(FBPanel.BUTTON_A, GPIO.FALLING, callback=panel_node.buttonAHandler, bouncetime=1000)
    GPIO.add_event_detect(FBPanel.BUTTON_B, GPIO.FALLING, callback=panel_node.buttonBHandler, bouncetime=1000)
    GPIO.add_event_detect(FBPanel.BUTTON_C, GPIO.FALLING, callback=panel_node.buttonCHandler, bouncetime=1000)
    

    try:
        rclpy.spin(panel_node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.remove_event_detect(FBPanel.BUTTON_ESTOP)
        GPIO.remove_event_detect(FBPanel.BUTTON_UNLOCK)
        GPIO.remove_event_detect(FBPanel.BUTTON_A)
        GPIO.remove_event_detect(FBPanel.BUTTON_B)
        panel_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()