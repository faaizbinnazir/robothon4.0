from time import sleep


class ServoHelper:
    def __init__(self, servo, speed_cm_s):
        self._servo = servo
        self._speed_cm_s = speed_cm_s

    def move_cm(self, distance_cm):
        time_to_work_s = distance_cm/self.speed_cm_s
        self.servo.turn_on_forward()
        sleep(time_to_work_s)
        self.servo.turn_off()

    def get_servo(self):
        return self.servo
