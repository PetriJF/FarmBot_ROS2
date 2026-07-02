#!/usr/bin/env python3
"""
GPIO controller module for FarmBot panel interface.

Manages hardware GPIO pins for LEDs, buttons, and emergency stop functionality.
Handles button press events and LED state control via ROS2 services and callbacks.
"""
import os

import RPi.GPIO as GPIO

from ament_index_python.packages import get_package_share_directory

from farmbot_interfaces.srv import LedPanelHandler

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import yaml


class GPIOController(Node):
    """
    Manage GPIO pins for FarmBot panel LEDs and buttons.

    Initializes and controls hardware GPIO pins for status LEDs, button LEDs,
    and button inputs. Handles button press events and provides LED control
    via ROS2 services.
    """

    # Node contructor
    def __init__(self):
        """Initialize the GPIO controller node and configure hardware pins."""
        super().__init__('GPIOController')

        GPIO.setmode(GPIO.BCM)

        self.directory = os.path.join(
            get_package_share_directory('farmbot_hardware_comm'),
            'config'
        )
        self.button = yaml.safe_load(open(os.path.join(self.directory, 'ButtonCommand.yaml'), 'r'))
        self.fb_panel = yaml.safe_load(open(os.path.join(self.directory, 'FarmbotPanel.yaml'), 'r'))

        # LED Initializing
        GPIO.setup(self.fb_panel['ESTOP_LED'], GPIO.OUT)
        GPIO.setup(self.fb_panel['UNLOCK_LED'], GPIO.OUT)
        GPIO.setup(self.fb_panel['BUTTON_LED_A'], GPIO.OUT)
        GPIO.setup(self.fb_panel['BUTTON_LED_B'], GPIO.OUT)
        GPIO.setup(self.fb_panel['BUTTON_LED_C'], GPIO.OUT)
        GPIO.setup(self.fb_panel['LED1'], GPIO.OUT)
        GPIO.setup(self.fb_panel['LED2'], GPIO.OUT)
        GPIO.setup(self.fb_panel['LED3'], GPIO.OUT)
        GPIO.setup(self.fb_panel['LED4'], GPIO.OUT)

        # Button Initialization
        GPIO.setup(self.fb_panel['BUTTON_ESTOP'], GPIO.IN)
        GPIO.setup(self.fb_panel['BUTTON_UNLOCK'], GPIO.IN)
        GPIO.setup(self.fb_panel['BUTTON_A'], GPIO.IN)
        GPIO.setup(self.fb_panel['BUTTON_B'], GPIO.IN)
        GPIO.setup(self.fb_panel['BUTTON_C'], GPIO.IN)

        self.cmd = String()
        self.lowlevel_command_pub = self.create_publisher(String, 'farmbot_command', 10)

        self.highlevel_command_pub = self.create_publisher(String, 'input_topic', 10)

        # LED Flasher Button
        self.flash_state = False
        flashing_frequency = 2.0
        self.leds_to_flash = []

        self.led_flasher_timer = self.create_timer(1.0 / flashing_frequency, self.LED_flasher)

        self.led_pin_list = [
            self.fb_panel['LED1'],
            self.fb_panel['LED2'],
            self.fb_panel['LED3'],
            self.fb_panel['LED4'],
            self.fb_panel['ESTOP_LED'],
            self.fb_panel['UNLOCK_LED'],
            self.fb_panel['BUTTON_LED_A'],
            self.fb_panel['BUTTON_LED_B'],
            self.fb_panel['BUTTON_LED_C']
        ]

        self.led_panel_server = self.create_service(LedPanelHandler, 'set_led', self.LED_server)

        # Log the initialization
        self.get_logger().info('GPIO Controller Initialized..')

    # Service Server

    def LED_server(self, request, response):
        """
        Service Server handling LED State manipulation.

        It sets the LEDS to ON, OFF or FLASHING.
        """
        # Check if the LED Pin is correct
        if request.led_pin not in self.led_pin_list:
            self.get_logger().warn('Selected LED pin is not recorded as having an LED attached')
            response.success = False
            return response
        # Check if an existing state was selected
        if request.state not in [self.fb_panel['LED_ON'], self.fb_panel['LED_OFF'],
                                 self.fb_panel['LED_FLASHING']]:
            self.get_logger().warn('Selected LED status is not recognized')
            response.success = False
            return response

        if request.state == self.fb_panel['LED_ON']:
            GPIO.output(request.led_pin, GPIO.HIGH)
            self.remove_flashing_led(request.led_pin)
        elif request.state == self.fb_panel['LED_OFF']:
            GPIO.output(request.led_pin, GPIO.LOW)
            self.remove_flashing_led(request.led_pin)
        elif request.state == self.fb_panel['LED_FLASHING']:
            self.add_flashing_led(request.led_pin)

        response.success = True
        return response

    def add_flashing_led(self, led_pin):
        """Add a LED from the flashing LED list."""
        if led_pin not in self.leds_to_flash:
            self.leds_to_flash.append(led_pin)

    def remove_flashing_led(self, led_pin):
        """Remove a LED from the flashing LED list."""
        if led_pin in self.leds_to_flash:
            self.leds_to_flash.remove(led_pin)

    # Service Client

    def _LED_client(self, led_pin, state):
        """Service client for switching an LED on or off."""
        client = self.create_client(LedPanelHandler, 'set_led')
        while not client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for LED Handling Server...')

        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = client.call_async(request=request)
        future.add_done_callback(self.LED_panel_callback)

    def LED_panel_callback(self, future):
        """Service client callback once the LED switching server ends."""
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Failure in LED Panel Handling!')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    # LED states for the panel

    def LED_flasher(self):
        """Timer Callback that flashes the LEDs in the self.leds_to_flash list."""
        for led_pin in self.leds_to_flash:
            GPIO.output(led_pin, GPIO.HIGH if self.flash_state else GPIO.LOW)
        self.flash_state = not self.flash_state

    def estop_button_handler(self, channel):
        """Estop Button Event Handler for the panel controller."""
        current_state = GPIO.input(self.fb_panel['BUTTON_ESTOP'])
        if current_state == GPIO.LOW:
            self.cmd.data = 'E'
            self.lowlevel_command_pub.publish(self.cmd)
            self.highlevel_command_pub.publish(self.cmd)
            self.get_logger().info('ESTOP button pressed')

    def reset_button_handler(self, channel):
        """Reset Button Event Handler for the panel controller."""
        current_state = GPIO.input(self.fb_panel['BUTTON_UNLOCK'])
        if current_state == GPIO.LOW:
            self.cmd.data = 'F09'
            self.lowlevel_command_pub.publish(self.cmd)
            self.cmd.data = 'R'
            self.highlevel_command_pub.publish(self.cmd)
            self.get_logger().info('RESET button pressed')

    def buttonHandler(self, channel):
        """
        Handle button press events for buttons A, B, and C.

        Reads the GPIO channel state and triggers the corresponding command
        for the pressed button.
        """
        current_state = GPIO.input(channel)
        if current_state == GPIO.LOW:
            if channel == self.fb_panel['BUTTON_A']:
                self.get_logger().info('BUTTON A pressed : ' +
                                       self.button['button_A']['command_name'] + ' is triggered')
                self.process_button_command(self.button['button_A']['command_type'],
                                            self.button['button_A']['command'])
            elif channel == self.fb_panel['BUTTON_B']:
                self.get_logger().info('BUTTON B pressed : ' +
                                       self.button['button_B']['command_name'] + ' is triggered')
                self.process_button_command(self.button['button_B']['command_type'],
                                            self.button['button_B']['command'])
            elif channel == self.fb_panel['BUTTON_C']:
                self.get_logger().info('BUTTON C pressed : ' +
                                       self.button['button_C']['command_name'] + ' is triggered')
                self.process_button_command(self.button['button_C']['command_type'],
                                            self.button['button_C']['command'])

    def process_button_command(self, level, cmd):
        """Determine the topic to which the command should be sent based on the command level."""
        self.cmd.data = cmd
        if level == 'LOW_LEVEL':
            self.lowlevel_command_pub.publish(self.cmd)
        elif level == 'HIGH_LEVEL':
            self.highlevel_command_pub.publish(self.cmd)

    def destroy_node(self):
        """Destroy_node overloading for cleaning up the GPIO."""
        # gpio cleanup
        GPIO.cleanup()


def main(args=None):
    """Initialize and run the GPIO controller node."""
    rclpy.init(args=args)

    gpio_node = GPIOController()

    # GPIO Button
    GPIO.add_event_detect(gpio_node.fb_panel['BUTTON_ESTOP'], GPIO.FALLING,
                          callback=gpio_node.estop_button_handler, bouncetime=200)
    GPIO.add_event_detect(gpio_node.fb_panel['BUTTON_UNLOCK'], GPIO.FALLING,
                          callback=gpio_node.reset_button_handler, bouncetime=200)
    GPIO.add_event_detect(gpio_node.fb_panel['BUTTON_A'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['BUTTON_B'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['BUTTON_C'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)

    try:
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.remove_event_detect(gpio_node.fb_panel['BUTTON_ESTOP'])
        GPIO.remove_event_detect(gpio_node.fb_panel['BUTTON_UNLOCK'])
        GPIO.remove_event_detect(gpio_node.fb_panel['BUTTON_A'])
        GPIO.remove_event_detect(gpio_node.fb_panel['BUTTON_B'])
        GPIO.remove_event_detect(gpio_node.fb_panel['BUTTON_C'])
        gpio_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
