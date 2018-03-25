import usonic

class DistanceSensor:

    def __init__(self, gpio_trig, gpio_echo):
        self.usonic = usonic()
        self.usonic.config(gpio_echo, gpio_trig)

    def get_distance_cm(self):
        return self.usonic.read()
