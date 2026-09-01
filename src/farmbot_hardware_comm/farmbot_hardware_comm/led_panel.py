"""
Panel LED control module for the FarmBot SerialController.

Contains the gpio_controller's set_led service clients for use in the serial controller.
"""
from farmbot_interfaces.srv import LedPanelHandler

from rclpy.node import Node


class LedPanel:
    """Call the farmbot panel LEDs through the gpio_controller."""

    def __init__(self, node: Node, panel: dict):
        """Set up the LED client for the pins listed in the panel configuration."""
        self.node = node
        self.panel = panel
        self.led_client = self.node.create_client(LedPanelHandler, 'hardware_comm/set_led')

    def show_ready(self):
        """Light the panel for a farmbot that is running and not e-stopped."""
        self.switch_led(self.panel['estop_led'], self.panel['led_on'])
        self.switch_led(self.panel['unlock_led'], self.panel['led_on'])

    def show_estopped(self):
        """Light the panel for an active e-stop, flashing the unlock button."""
        self.switch_led(self.panel['estop_led'], self.panel['led_off'])
        self.switch_led(self.panel['unlock_led'], self.panel['led_flashing'])

    def switch_led(self, led_pin: int, state: int):
        """
        Send a request to set an LED state.

        The request is sent asynchronously. If the LED handling service is not
        available, the request is ignored.

        Args:
            led_pin {int}: GPIO pin of the LED to control.
            state {int}: led_off, led_on or led_flashing, from the panel config.
        """
        if not self.led_client.service_is_ready():
            self.node.get_logger().warn('LED Handling Server not available!')
            return

        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = self.led_client.call_async(request=request)
        future.add_done_callback(self.led_panel_callback)

    def led_panel_callback(self, future):
        """
        Handle the response from the LED handling service.

        Logs a warning if the service reports a failure or an error if the service
        call itself fails.

        Args:
            future: Future containing the service response.
        """
        try:
            response = future.result()
            if not response:
                self.node.get_logger().warn('Failure in LED Panel Handling!')
        except Exception as e:
            self.node.get_logger().error('Service call failed %r' % (e, ))
