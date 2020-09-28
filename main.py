from . import ServoHelper
from . import Servo
from . import DistanceSensor
from .Point import Point

from math import pi as PI
from time import sleep
from time import time


def run_servos_for_seconds(servos_list, run_time_s):
    for servo in servos_list:
        servo.turn_on_forward()
    sleep(run_time_s)
    for servo in servos_last:
        servo.turn_off()

def is_there_obstacle(distance_sensor, threshold_cm):
    return distance_sensor.get_distance_cm()<threshold_cm

def calculate_servo_speed(servo_L, servo_R, distance_sensor):
    calibration_time_s = 5
    distance_initial_cm = distance_sensor.get_distance_cm()
    run_servos_for_seconds([servo_L, servo_R], calibration_time_s)
    distance_final_cm = distance_sensor.get_distance_cm()
    distance_moved_cm = distance_final_cm - distance_initial_cm
    speed_cm_s = distance_moved_cm/calibration_time_s
    return speed_cm_s


def distance_cm_to_next_wall_infront(servo_L: ServoHelper,
                                     servo_R: ServoHelper,
                                     distance_sensor: DistanceSensor,
                                     min_distance_cm):
    distance_initial_cm = distance_sensor.get_distance_cm()
    servo_L.turn_on_forward()
    servo_R.turn_on_forward()
    while distance_sensor.get_distance_cm() > min_distance_cm:
        sleep(0.05)
    servo_L.turn_off()
    servo_R.turn_off()
    return distance_sensor.get_distance_cm() - distance_initial_cm


def distance_cm_along_wall_till_broken(servo_L: ServoHelper,
                                     servo_R: ServoHelper,
                                     distance_sensor_side: DistanceSensor,
                                     distance_sensor_front: DistanceSensor,
                                     min_distance_cm):
    distance_initial_cm = distance_sensor_front.get_distance_cm()
    servo_L.turn_on_forward()
    servo_R.turn_on_forward()
    while distance_sensor_front.get_distance_cm() > min_distance_cm and\
          distance_sensor_side.get_distance_cm() <= min_distance_cm:
        sleep(0.05)
    servo_L.turn_off()
    servo_R.turn_off()
    return distance_sensor_front.get_distance_cm() - distance_initial_cm


def align_parallel_to_wall_and_get_angle_turned_ccw(servo_L, servo_R,
                                                    wheel_distance_cm,
                                                    speed_cm_s,
                                                    distance_sensor):
    time_initial_secs = time()
    distance_1 = distance_sensor.get_distance_cm()
    servo_R.turn_on_forward()
    servo_L.turn_on_backward()
    distance_2 = distance_sensor.get_distance_cm()
    while distance_2 < distance_1:
        sleep(0.01)
        distance_1 = distance_2
        distance_2 = distance_sensor.get_distance_cm()
    servo_L.turn_off()
    servo_R.turn_off()
    time_final_secs = time()
    time_turned_secs = time_final_secs-time_initial_secs
    distance_moved_along_circle = speed_cm_s*time_turned_secs
    radius_cm = wheel_distance_cm/2
    angle_turned_rads = distance_moved_along_circle/radius_cm
    angle_turned_degs = 360/PI * angle_turned_rads
    return angle_turned_degs

def turn_90_deg_ccw(servo_L, servo_R, wheel_distance_cm, speed_cm_s):
    radius_cm = wheel_distance_cm/2
    distance_to_turn_cm = radius_cm*(PI/2)
    time_to_work_s = distance_to_turn_cm/speed_cm_s
    servo_L.turn_on_backward()
    servo_R.turn_on_forward()
    sleep(time_to_work_s)
    servo_L.turn_off()
    servo_R.turn_off()

def turn_90_deg_cw(servo_L, servo_R, wheel_distance_cm, speed_cm_s):
    radius_cm = wheel_distance_cm/2
    distance_to_turn_cm = radius_cm*(PI/2)
    time_to_work_s = distance_to_turn_cm/speed_cm_s
    servo_R.turn_on_backward()
    servo_L.turn_on_forward()
    sleep(time_to_work_s)
    servo_R.turn_off()
    servo_L.turn_off()

###### MAIN #######

MIN_DISTANCE_TO_WALL_CM = 10
WHEEL_DISTANCE_CM = 10

# Setup
servo_L = Servo()
servo_R = Servo()
distance_sensor_F = DistanceSensor()
distance_sensor_L = DistanceSensor()
distance_sensor_R = DistanceSensor()

# Calibrate
SPEED_CM_S = calculate_servo_speed(servo_L, servo_R, distance_sensor_F)

# Find first wall
distance_cm_to_next_wall_infront(servo_L,servo_R, distance_sensor_F)
turn_90_deg_ccw(servo_L, servo_R, WHEEL_DISTANCE_CM)

#find first corner
distance_cm_to_next_wall_infront(servo_L,servo_R, distance_sensor_F)
# this is our origin


# all coordinates are in cm
DISTANCE_THRESHOLD_CM = 5
origin = Point(0,0)
curr_pos = origin
curr_orientation = Point(0,1)
corners = []
while True:
    if is_there_obstacle(distance_sensor_R, DISTANCE_THRESHOLD_CM):
        turn_90_deg_ccw(servo_L, servo_R, WHEEL_DISTANCE_CM)
        curr_orientation = curr_orientation.rotate(PI/2) # Counter-clockwise
    else:
        turn_90_deg_cw(servo_L, servo_R, WHEEL_DISTANCE_CM)
        curr_orientation = curr_orientation.rotate(-PI/2) # Counter-clockwise
    x = distance_cm_along_wall_till_broken(servo_L,servo_R,
                                           distance_sensor_R,
                                           distance_sensor_F)
    curr_pos += curr_orientation*x
    corners  += [curr_pos]
    if curr_pos.distance_to(origin) < DISTANCE_THRESHOLD_CM:
        break

print("The detected corners are:")
for p in corners:
    print(p)
