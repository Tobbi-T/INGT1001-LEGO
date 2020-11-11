#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Practice description:
#   Build a LEGO car and make a Python program for it.
#   The program should do the following:
#       1. Print "Hello World" using the onboard screen.
#       2. Drive in a rectangle once, then stop.
#       3. Say "Have a nice day" using the onboard speaker.

# Robot information
EV3 = EV3Brick()
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.D
WHEEL_DIAMETER = 100  # The diameter of the robot wheels [mm].
AXLE_TRACK = 165  # The distance between the robot wheels [mm].
ROBOT = DriveBase(Motor(LEFT_MOTOR_PORT), Motor(RIGHT_MOTOR_PORT), WHEEL_DIAMETER, AXLE_TRACK)

# Settings
ANGLES = 4  # The number of angles in the shape the robot drives.
ANGLE_TURN_CORRECTION = 10  # Correction to compensate for turn inaccuracy [deg].
DRIVE_LENGTH = 1000  # The distance the robot drives for every side in the shape [mm].

EV3.screen.print('Hello World')

for i in range(0, ANGLES):
    ROBOT.straight(DRIVE_LENGTH)
    ROBOT.turn(360/ANGLES + ANGLE_TURN_CORRECTION)

EV3.speaker.say('Have a nice day')
