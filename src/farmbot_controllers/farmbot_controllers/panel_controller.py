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
        super().__init__("PanelController")

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
        self.inputPub_ = self.create_publisher(String, 'input_topic', 10)
        self.inputSub_ = self.create_subscription(String, 'keyboard_topic', self.cmdCallback, 10)

        # LED Flasher Button
        self.flashState_ = False
        flashingFrequency = 2.0
        self.ledsToFlash_ = [FBPanel.ESTOP_LED, FBPanel.UNLOCK_LED]
        
        self.ledFlashingTimer_ = self.create_timer(1.0 / flashingFrequency, self.ledFlasher)

        self.ledPinList_ = [
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
        self.ledPanelServer_ = self.create_service(LedPanelHandler, 'set_led', self.ledHandlerCallback)

        # Log the initialization
        self.get_logger().info("Panel Controller Initialized..")

    ### Verifying user input for E_STOPS and RESETS

    def cmdCallback(self, cmd = String):
        if cmd.data == 'e':
            self.callLEDPanel(FBPanel.ESTOP_LED, FBPanel.OFF)
            self.callLEDPanel(FBPanel.UNLOCK_LED, FBPanel.FLASHING)
            self.cmd_.data = 'e'
            self.get_logger().info("ESTOP button pressed")
        elif cmd.data == 'E' or cmd.data == 'i':
            self.callLEDPanel(FBPanel.ESTOP_LED, FBPanel.ON)
            self.callLEDPanel(FBPanel.UNLOCK_LED, FBPanel.ON)
            self.cmd_.data = 'E'
            self.get_logger().info("RESET button pressed")

        self.inputPub_.publish(cmd)

    ### Service Server

    def ledHandlerCallback(self, request, response):
        # Check if the LED Pin is correct
        if request.led_pin not in self.ledPinList_:
            self.get_logger().warn("Selected LED pin is not recorded as having an LED attached")
            response.success = False
            return response
        # Check if an existing state was selected
        if request.state not in [FBPanel.ON, FBPanel.OFF, FBPanel.FLASHING]:
            self.get_logger().warn("Selected LED status is not recognized")
            response.success = False
            return response
        
        if request.state == FBPanel.ON:
            GPIO.output(request.led_pin, GPIO.HIGH)
            self.removeFromFlashingLeds(request.led_pin)
        elif request.state == FBPanel.OFF:
            GPIO.output(request.led_pin, GPIO.LOW)
            self.removeFromFlashingLeds(request.led_pin)
        elif request.state == FBPanel.FLASHING:
            self.addToFlashingLeds(request.led_pin)

        response.success = True
        return response

    def addToFlashingLeds(self, led_pin):
        if led_pin not in self.ledsToFlash_:
            self.ledsToFlash_.append(led_pin)
    
    def removeFromFlashingLeds(self, led_pin):
        if led_pin in self.ledsToFlash_:
            self.ledsToFlash_.remove(led_pin)

    ### Service Client

    def callLEDPanel(self, led_pin, state):
        client = self.create_client(LedPanelHandler, 'set_led')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for LED Handling Server...")
        
        request = LedPanelHandler.Request()
        request.led_pin = led_pin
        request.state = state

        future = client.call_async(request = request)
        future.add_done_callback(self.callbackLEDPanel)

    def callbackLEDPanel(self, future):
        try:
            response = future.result()
            if not response:
                self.get_logger().warn("Failure in LED Panel Handling!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e, ))

    ### LED states for the panel

    def ledFlasher(self):
        for led_pin in self.ledsToFlash_:
            GPIO.output(led_pin, GPIO.HIGH if self.flashState_ else GPIO.LOW)
        self.flashState_ = not self.flashState_
    
    def estopButtonHandler(self, channel):
        current_state = GPIO.input(FBPanel.BUTTON_ESTOP)
        if current_state == GPIO.LOW:
            self.callLEDPanel(FBPanel.ESTOP_LED, FBPanel.OFF)
            self.callLEDPanel(FBPanel.UNLOCK_LED, FBPanel.FLASHING)
            self.cmd_.data = 'e'
            self.inputPub_.publish(self.cmd_)
            self.get_logger().info("ESTOP button pressed")

    def resetButtonHandler(self, channel):
        current_state = GPIO.input(FBPanel.BUTTON_UNLOCK)
        if current_state == GPIO.LOW:
            self.callLEDPanel(FBPanel.ESTOP_LED, FBPanel.ON)
            self.callLEDPanel(FBPanel.UNLOCK_LED, FBPanel.ON)
            self.cmd_.data = 'E'
            self.inputPub_.publish(self.cmd_)
            self.get_logger().info("RESET button pressed")
    
    # Just for demonstration purposes
    # def buttonAHandler(self, channel):
    #     current_state = GPIO.input(FBPanel.BUTTON_A)
    #     if current_state == GPIO.LOW:
    #         self.callLEDPanel(FBPanel.LED1, FBPanel.FLASHING)
    #         self.callLEDPanel(FBPanel.LED2, FBPanel.FLASHING)
    #         self.callLEDPanel(FBPanel.LED3, FBPanel.FLASHING)
    #         self.callLEDPanel(FBPanel.LED4, FBPanel.FLASHING)
    #         #self.cmd_.data = 'A'
    #         #self.inputPub_.publish(self.cmd_)
    #         self.get_logger().info("Button A pressed")

    def destroy_node(self):
        # gpio cleanup
        GPIO.cleanup()


def main(args = None):
    rclpy.init(args = args)

    panelControllerNode = PanelController()
    
    # GPIO Button
    GPIO.add_event_detect(FBPanel.BUTTON_ESTOP, GPIO.FALLING, callback=panelControllerNode.estopButtonHandler, bouncetime=1000)
    GPIO.add_event_detect(FBPanel.BUTTON_UNLOCK, GPIO.FALLING, callback=panelControllerNode.resetButtonHandler, bouncetime=1000)
    #GPIO.add_event_detect(FBPanel.BUTTON_A, GPIO.FALLING, callback=panelControllerNode.buttonAHandler, bouncetime=1000)

    try:
        #rclpy.spin(keyboardTeleOpNode)
        rclpy.spin(panelControllerNode)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.remove_event_detect(FBPanel.BUTTON_ESTOP)
        GPIO.remove_event_detect(FBPanel.BUTTON_UNLOCK)
        #GPIO.remove_event_detect(FBPanel.BUTTON_A)
        panelControllerNode.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()