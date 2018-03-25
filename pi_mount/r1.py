import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)

p.start(7.5)


class DistanceSensor:


class servoMotion:
    """docstring for servoMotion."""
    def __init__(self,angle,distance):
        self.angle=0
        self.distance=0

    theta=0

    def wallFinder():
        servoMotion.goFront()

    def goFront(degree):
        if front!=minDist:
            self.distance=degree*radius

    def turnRight():
        leftServo.write(-270)
        rightServo.write(270)
        self.theta=theta-90

    def turnLeft():
        leftServo.write(270)
        rightServo.write(-270)
        self.theta=theta+90

    def goBack():
        leftServo.write(360)
        rightServo.write(360)
        sleep(3000)

    def keepRight():
        if !goFront():
            servoMotionturnRight()
    def keepLeft():
        if !keepRight():
            servoMotionturnLeft()

class points():
    """docstring for points."""
    def __init__(self, x,y):
        self.x=0
        self.y=0
        self.angle=servoMotion.angle

    def markCorner():
        if angle==0 :
            x=x+servoMotion.goFront
        elif angle==90
            y=y+servoMotion.goFront
        elif angle=-90:
            y=y-servoMotion.goFront
        elif angle=180 or angle=-180:
            x=x-servoMotion.goFront
        elif angle=270:
            y=y-servoMotion.goFront
        elif angle=-270:
            y=y+servoMotion.goFront
        elif angle=360 or angle=-360
            x=x+servoMotion.goFront



    def cornerFind():
        wallFinder()
        if servoMotion.turnRight or servoMotion.turnLeft:
            points.markCorner()
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)

p.start(7.5)

class servo:
    """docstring for servo."""
    minDist=50 """mm"""
    radius=37  """mm"""
    x=0
    y=0
    angle=0
    distance=0
    def __init__(self, degree):
        self.currentPos=0
    def rotateServo(self,degree):
        self.currentPos=degree
        print("curent servo position"+self.currentPos)
    def wallFinder():
        servo.goFront()
    def goFront(degree):
        if front!=minDist:
            distance=degree*radius
    def turnRight():
        leftServo.write(-270)
        rightServo.write(270)
    def turnLeft():
        leftServo.write(270)
        rightServo.write(-270)
    def goBack():
        leftServo.write(360)
        rightServo.write(360)
        sleep(3000)
    def keepRight():
        if !goFront():
            turnRight()
    def keepLeft():
        if !keepRight():
            turnLeft()
    def cornerFind():
        wallFinder()
        if condition:
            pass
