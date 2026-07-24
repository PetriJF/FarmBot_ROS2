#!/usr/bin/env python3
"""
GPIO controller module for the FarmBot panel interface.

Manages GPIO hardware components including leds, buttons, and the emergency
stop input. Provides ROS 2 services and callbacks to handle button events,
control led states, and publish panel commands.
"""
import RPi.GPIO as GPIO

from farmbot_hardware_comm.modules.yaml_loader import YAMLLoader

from farmbot_interfaces.srv import LedPanelHandler

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class GPIOController(Node):
    """
    Manage GPIO pins for FarmBot panel leds and buttons.

    Initialises and controls hardware GPIO pins for status leds, button leds,
    and button inputs. Handles button press events and provides led control
    via ROS2 services.
    """

    # Node contructor
    def __init__(self):
        """Initialise the GPIO controller node and configure hardware pins."""
        super().__init__('GPIOController')

        GPIO.setmode(GPIO.BCM)

        self.directory = YAMLLoader.get_directory_package('farmbot_hardware_comm', 'config')

        self.button = YAMLLoader.load_yaml(self.directory, 'ButtonCommand.yaml')
        self.fb_panel = YAMLLoader.load_yaml(self.directory, 'FarmbotPanel.yaml')

        # led Initialising
        GPIO.setup(self.fb_panel['estop_led'], GPIO.OUT)
        GPIO.setup(self.fb_panel['unlock_led'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_A'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_B'], GPIO.OUT)
        GPIO.setup(self.fb_panel['button_led_C'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_1'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_2'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_3'], GPIO.OUT)
        GPIO.setup(self.fb_panel['led_4'], GPIO.OUT)

        # Button Initialisation
        GPIO.setup(self.fb_panel['button_estop'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_unlock'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_A'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_B'], GPIO.IN)
        GPIO.setup(self.fb_panel['button_C'], GPIO.IN)

        self.cmd = String()
        self.estop_client = self.node.create_client(Trigger, 'estop')
        self.resume_client = self.node.create_client(Trigger, 'resume')

        self.highlevel_command_pub = self.create_publisher(String, 'input_topic', 10)    # TODO

        # Led Flasher Button
        self.flash_state = False

        self.declare_parameter('flashing_frequency', rclpy.Parameter.Type.DOUBLE)
        flashing_frequency = self.get_parameter('flashing_frequency'
                                                ).get_parameter_value().double_value

        self.leds_to_flash = []

        self.led_flasher_timer = self.create_timer(1.0 / flashing_frequency, self.led_flasher)

        self.led_pin_list = [
            self.fb_panel['led_1'],
            self.fb_panel['led2'],
            self.fb_panel['led3'],
            self.fb_panel['led4'],
            self.fb_panel['estop_led'],
            self.fb_panel['unlock_led'],
            self.fb_panel['button_led_A'],
            self.fb_panel['button_led_B'],
            self.fb_panel['button_led_C']
        ]

        self.led_panel_server = self.create_service(LedPanelHandler, 'set_led', self.led_server)

        # Log the Initialisation
        self.get_logger().info('GPIO Controller Initialised..')

    # Service Server

    def led_server(self, request: LedPanelHandler.Request,
                   response: LedPanelHandler.Response) -> LedPanelHandler.Response:
        """
        Handle an led panel service request.

        Validates the requested led pin and state, then updates the led to be
        turned on, turned off, or set to flash.

        Args:
            request {LedPanelHandler.Request}: led control request.
            response {LedPanelHandler.Response}: Service response object.
        """
        # Check if the led Pin is correct
        if request.led_pin not in self.led_pin_list:
            self.get_logger().warn('Selected led pin is not recorded as having an led attached')
            response.success = False
            return response
        # Check if an existing state was selected
        if request.state not in [self.fb_panel['led_on'], self.fb_panel['led_off'],
                                 self.fb_panel['led_flashing']]:
            self.get_logger().warn('Selected led status is not recognized')
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
        Add an led to the flashing led list.

        Args:
            led_pin {int}: GPIO pin of the led to flash.
        """
        if led_pin not in self.leds_to_flash:
            self.leds_to_flash.append(led_pin)

    def remove_flashing_led(self, led_pin: int):
        """
        Remove an led from the flashing led list.

        Args:
            led_pin {int}: GPIO pin to stop flashing.
        """
        if led_pin in self.leds_to_flash:
            self.leds_to_flash.remove(led_pin)

    # Led states for the panel

    def led_flasher(self):
        """
        Toggle the state of all flashing leds.

        This timer callback alternates the output state of every led registered
        in the flashing led list to produce a blinking effect.
        """
        for led_pin in self.leds_to_flash:
            GPIO.output(led_pin, GPIO.HIGH if self.flash_state else GPIO.LOW)
        self.flash_state = not self.flash_state

    def estop_button_handler(self, channel):
        """
        Handle the emergency stop button event.

        Checks the state of the emergency stop button and publishes an emergency
        stop command when the button is pressed.

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

            self.cmd.data = 'E'
            self.highlevel_command_pub.publish(self.cmd)
            self.get_logger().info('estop button pressed')

    def reset_button_handler(self, channel):
        """
        Handle the reset emergency stop button event.

        Checks the state of the reset emergency stop  button and publishes an emergency
        stop command when the button is pressed.

        Args:
            channel {int}: GPIO channel that triggered the event.
        """
        current_state = GPIO.input(self.fb_panel['button_unlock'])
        if current_state == GPIO.LOW:
            if not self.resume_client.wait_for_service(1.0):
                self.get_logger().fatal('Resume Server not available!')
                raise ServerError('GPIO controller failed: server unavailable')
            request = Trigger.Request()
            future = self.estop_client.call_async(request=request)
            future.add_done_callback(self.client_callback)

            self.cmd.data = 'R'
            self.highlevel_command_pub.publish(self.cmd)
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
                self.node.get_logger().warn('Command Failure!')

            elif not response.success:
                self.node.get_logger().warn(f'Command {response.message}!')

            elif response.message:
                self.node.get_logger().info(response.message)

            else:
                self.node.get_logger().info('Command succesful')

        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))

    def button_handler(self, channel):
        """
        Handle button press events for the panel buttons.

        Reads the GPIO channel state and triggers the command associated with
        the pressed button.

        Args:
            channel {int}: GPIO channel that triggered the callback.
        """
        current_state = GPIO.input(channel)
        if current_state == GPIO.LOW:
            if channel == self.fb_panel['button_A']:
                self.get_logger().info('button A pressed : ' +
                                       self.button['button_A']['command_name'] + ' is triggered')
                self.cmd.data = self.button['button_A']['command']
            elif channel == self.fb_panel['button_B']:
                self.get_logger().info('button B pressed : ' +
                                       self.button['button_B']['command_name'] + ' is triggered')
                self.cmd.data = self.button['button_B']['command']
            elif channel == self.fb_panel['button_C']:
                self.get_logger().info('button C pressed : ' +
                                       self.button['button_C']['command_name'] + ' is triggered')
                self.cmd.data = self.button['button_C']['command']
            self.highlevel_command_pub.publish(self.cmd)

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
    GPIO.add_event_detect(gpio_node.fb_panel['button_A'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['button_B'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)
    GPIO.add_event_detect(gpio_node.fb_panel['button_C'], GPIO.FALLING,
                          callback=gpio_node.buttonHandler, bouncetime=1000)

    try:
        rclpy.spin(gpio_node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.remove_event_detect(gpio_node.fb_panel['button_estop'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_unlock'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_A'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_B'])
        GPIO.remove_event_detect(gpio_node.fb_panel['button_C'])
        gpio_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
