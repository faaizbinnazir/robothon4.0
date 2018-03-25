from rpi_devices import cleanup
from rpi_devices import Servo as ServoBackend

class Servo:
    def __init__(self, gpio_pin):
        self.servo_backend = ServoBackend(gpio_pin)
        self.gpio_pin = gpio_pin
        pass

    def turn_on_forward(self):
        self.servo_backend.turn(180)
        print("Turning on servo (fw).")

    def turn_on_backward(self):
        self.servo_backend.turn(0)
        print("Turning on servo (bw).")

    def turn_off(self):
        self.servo_backend.turn(90)
        print("Turning off servo.")
