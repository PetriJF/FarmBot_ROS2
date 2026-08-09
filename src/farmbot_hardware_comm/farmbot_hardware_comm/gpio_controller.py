#!/usr/bin/env python3
"""
GPIO controller module for the FarmBot panel interface.

Manages GPIO hardware components including LEDs, buttons, and the emergency
stop input. The emergency stop and unlock buttons call the bridge's state
triggers directly, while the configurable panel buttons publish the high-level
command they are mapped to in ButtonCommand.yaml.
"""
import RPi.GPIO as GPIO

from farmbot_hardware_comm.modules.exceptions import ServerError, YAMLError
from farmbot_hardware_comm.modules.yaml_handler import YAMLHandler

from farmbot_interfaces.srv import LedPanelHandler

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger


class GPIOController(Node):
    """
    Manage GPIO pins for FarmBot panel LEDs and buttons.

    Initialises and controls hardware GPIO pins for status LEDs, button LEDs,
    and button inputs. Handles button press events and provides LED control
    via ROS2 services.
    """

    # Node constructor
    def __init__(self):
        """Initialise the GPIO controller node and configure hardware pins."""
        super().__init__('GPIOController')

        GPIO.setmode(GPIO.BCM)

        self.directory = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')

        try:
            self.button = YAMLHandler.load_yaml(self.directory, 'ButtonCommand.yaml')
            self.fb_panel = YAMLHandler.load_yaml(self.directory, 'FarmbotPanel.yaml')
        except YAMLError as e:
            self.get_logger().warn(f'yaml error: {e}')
            return

        # LED Initialising
        GPIO.setup(self.fb_panel['estop_led'], GPIO.OUT)
        GPIO.setup(self.fb_panel['unlock_led'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_a'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_b'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_c'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_1'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_2'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_3'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_4'], GPIO.OUT)

        # Button Initialisation
        GPIO.setup(self.fb_panel['button_estop'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_unlock'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_a'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_b'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_c'], GPIO.IN)

        self.estop_client = self.create_client(Trigger, 'estop')
        self.resume_client = self.create_client(Trigger, 'resume')

        # TODO: migrate high-level commands to language-agnostic service calls
        self.request_command_pub = self.create_publisher(String, 'request_command', 10)

        # Maps a button's GPIO channel to its ButtonCommand.yaml entry
        self.button_channels = {self.fb_panel[button]: button
                                for button in ('button_a', 'button_b', 'button_c')}

        # LED Flasher Button
        self.flash_state = False

        self.declare_parameter('flashing_frequency', rclpy.Parameter.Type.DOUBLE)
        flashing_frequency = self.get_parameter('flashing_frequency'
                                                ).get_parameter_value().double_value

        self.leds_to_flash = []

        self.led_flasher_timer = self.create_timer(1.0 / flashing_frequency, self.led_flasher)

        self.led_pin_list = [
            self.fb_panel['led_1'],
            self.fb_panel['led_2'],
            self.fb_panel['led_3'],
            self.fb_panel['led_4'],
            self.fb_panel['estop_led'],
            self.fb_panel['unlock_led'],
            self.fb_panel['button_led_a'],
            self.fb_panel['button_led_b'],
            self.fb_panel['button_led_c']
        ]

        self.led_panel_server = self.create_service(LedPanelHandler, 'set_led', self.led_server)

        # Log the Initialisation
        self.get_logger().info('GPIO Controller Initialised..')

    # Service Server

    def led_server(self, request: LedPanelHandler.Request,
                   response: LedPanelHandler.Response) -> LedPanelHandler.Response:
        """
        Handle an LED panel service request.

        Validates the requested LED pin and state, then updates the LED to be
        turned on, turned off, or set to flash.

        Args:
            request {LedPanelHandler.Request}: LED control request.
            response {LedPanelHandler.Response}: Service response object.
        """
        # Check if the LED pin is correct
        if request.led_pin not in self.led_pin_list:
            self.get_logger().warn('Selected LED pin is not recorded as having an LED attached')
            response.success = False
            return response
        # Check if an existing state was selected
        if request.state not in [self.fb_panel['led_on'], self.fb_panel['led_off'],
                                 self.fb_panel['led_flashing']]:
            self.get_logger().warn('Selected LED status is not recognised')
            response.success = False
            return response

        if request.state == self.fb_panel['led_on']:
            GPIO.output(request.led_pin, GPIO.HIGH)
            self.remove_flashing_led(request.led_pin)
        elif request.state == self.fb_panel['led_off']:
            GPIO.output(request.led_pin, GPIO.LOW)
            self.remove_flashing_led(request.led_pin)
        elif request.state == self.fb_panel['led_flashing']:
            self.add_flashing_led(request.led_pin)

        response.success = True
        return response

    def add_flashing_led(self, led_pin: int):
        """
        Add an LED to the flashing LED list.

        Args:
            led_pin {int}: GPIO pin of the LED to flash.
        """
        if led_pin not in self.leds_to_flash:
            self.leds_to_flash.append(led_pin)

    def remove_flashing_led(self, led_pin: int):
        """
        Remove an LED from the flashing LED list.

        Args:
            led_pin {int}: GPIO pin to stop flashing.
        """
        if led_pin in self.leds_to_flash:
            self.leds_to_flash.remove(led_pin)

    # LED states for the panel

    def led_flasher(self):
        """
        Toggle the state of all flashing LEDs.

        This timer callback alternates the output state of every LED registered
        in the flashing LED list to produce a blinking effect.
        """
        for led_pin in self.leds_to_flash:
            GPIO.output(led_pin, GPIO.HIGH if self.flash_state else GPIO.LOW)
        self.flash_state = not self.flash_state

    def estop_button_handler(self, channel):
        """
        Handle the emergency stop button event.

        Checks the state of the emergency stop button and calls the estop service
        when the button is pressed.

        Args:
            channel {int}: GPIO channel that triggered the event.
        """
        current_state = GPIO.input(self.fb_panel['button_estop'])
        if current_state == GPIO.LOW:
            if not self.estop_client.wait_for_service(1.0):
                self.get_logger().fatal('Estop Server not available!')
                raise ServerError('GPIO controller failed: server unavailable')
            request = Trigger.Request()
            future = self.estop_client.call_async(request=request)
            future.add_done_callback(self.client_callback)

            self.get_logger().info('ESTOP button pressed')

    def reset_button_handler(self, channel):
        """
        Handle the reset emergency stop button event.

        Checks the state of the reset button and calls the resume service
        when the button is pressed.

        Args:
            channel {int}: GPIO channel that triggered the event.
        """
        current_state = GPIO.input(self.fb_panel['button_unlock'])
        if current_state == GPIO.LOW:
            if not self.resume_client.wait_for_service(1.0):
                self.get_logger().fatal('Resume Server not available!')
                raise ServerError('GPIO controller failed: server unavailable')
            request = Trigger.Request()
            future = self.resume_client.call_async(request=request)
            future.add_done_callback(self.client_callback)

            self.get_logger().info('RESET button pressed')

    def client_callback(self, future):
        """
        Handle the response of a service client request.

        Processes the service response and logs the command status depending
        on the result of the request.
        """
        try:
            response = future.result()
            if not response:
                self.get_logger().warn('Command Failure!')

            elif not response.success:
                self.get_logger().warn(f'Command {response.message}!')

            elif response.message:
                self.get_logger().info(response.message)

            else:
                self.get_logger().info('Command successful')

        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e, ))

    def button_handler(self, channel):
        """
        Handle button press events for the configurable panel buttons.

        Reads the GPIO channel state and publishes the high-level command that
        ButtonCommand.yaml maps to the pressed button.

        Args:
            channel {int}: GPIO channel that triggered the callback.
        """
        if GPIO.input(channel) != GPIO.LOW:
            return

        button = self.button_channels.get(channel)
        if button is None:
            self.get_logger().warn(f'GPIO channel {channel} is not a mapped button. Ignored')
            return

        self.get_logger().info(f"{button} pressed : {self.button[button]['command_name']} "
                               'is triggered')

        cmd = String()
        cmd.data = self.button[button]['command']
        self.request_command_pub.publish(cmd)

    def destroy_node(self):
        """Destroy_node overloading for cleaning up the GPIO."""
        # gpio cleanup
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    """Initialise and run the GPIO controller node."""
    rclpy.init(args=args)

    gpio_node = GPIOController()

    # GPIO button
    GPIO.add_event_detect(gpio_node.fb_panel['button_estop'], GPIO.FALLING,
                          callback=gpio_node.estop_button_handler, bouncetime=200)
    GPIO.add_event_detect(gpio_node.fb_panel['button_unlock'], GPIO.FALLING,
                          callback=gpio_node.reset_button_handler, bouncetime=200)
    GPIO.add_event_detect(gpio_node.fb_panel['button_a'], GPIO.FALLING,
                          callback=gpio_node.button_handler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['button_b'], GPIO.FALLING,
                          callback=gpio_node.button_handler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['button_c'], GPIO.FALLING,
                          callback=gpio_node.button_handler, bouncetime=1000)

    try:
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.remove_event_detect(gpio_node.fb_panel['button_estop'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_unlock'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_a'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_b'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_c'])
        gpio_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
