#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Practice description:
#   Make NTNU's new lawn mower robot.
#   The program should do the following:
#       1. Wait for a button to be pressed, then say "Exercise 2" and start driving.
#       2. Drive around and use an ultrasonic sensor to avoid obstacles.
#       3. When the button is pressed again, say "Exercise done" and exit.

# Robot information
EV3 = EV3Brick()
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.D
TOUCH_SENSOR = TouchSensor(Port.S1)  # The touch sensor to use to detect start/stop signals.
DISTANCE_SENSOR = UltrasonicSensor(Port.S4)  # The ultrasonic sensor to use to avoid obstacles.
WHEEL_DIAMETER = 100  # The diameter of the robot wheels [mm].
AXLE_TRACK = 165  # The distance between the robot wheels [mm].
ROBOT = DriveBase(Motor(LEFT_MOTOR_PORT), Motor(RIGHT_MOTOR_PORT), WHEEL_DIAMETER, AXLE_TRACK)

# Stay in this loop and check for button press every 300 ms.
while not TOUCH_SENSOR.pressed():
    wait(300)

# Once button is pressed, say "Exercise 2" and wait 2000 ms.
EV3.speaker.say('Exercise 2')
wait(2000)

# Stay in this loop until button is pressed again.
while not TOUCH_SENSOR.pressed():
    distance = DISTANCE_SENSOR.distance()  # Distance acquired from the ultrasonic sensor [mm].

    EV3.screen.clear()
    EV3.screen.print("Distance:\n" + distance)

    wait(300)

    # If distance less than 40 cm or too close to measure, turn 40 degrees.
    if distance < 400 or distance == 2550:
        ROBOT.turn(40)

    ROBOT.straight(100)  # Drive straight for 100 mm.

# When the button is pressed for the second time, say "Exercise done" and exit.
EV3.speaker.say('Exercise done')
