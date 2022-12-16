#!/usr/bin/env pybricks-micropython
from bbot import *

# Create your objects here.
ev3 = EV3Brick()
bot = BehavioralBot()

# CONSTS
right_port = Port.C
left_port = Port.D
color_port = Port.B
speed = 100
time = 10000

# Film
right_motor = Motor(right_port, positive_direction=Direction.CLOCKWISE, gears=None)
left_motor = Motor(left_port, positive_direction=Direction.CLOCKWISE, gears=None)
color_sensor = ColorSensor(color_port)
#rightMotor.run_angle(speed=200, rotation_angle=forward_angle, then=Stop.COAST, wait=False)
#leftMotor.run_angle(speed=200, rotation_angle=forward_angle, then=Stop.COAST, wait=True)
rightMotor.run_time(speed=speed, time=time, then=Stop.COAST, wait=False)
leftMotor.run_time(speed=speed, time=time, then=Stop.COAST, wait=True)
