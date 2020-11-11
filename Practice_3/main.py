#!/usr/bin/env pybricks-micropython
import random

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Practice description:
#   Make NTNU's new entertainment robot.
#   The program should do the following:
#       1. Drive around a table, following a black tape.
#       2. Stop every 10 seconds to perform entertainment.
#       3. After 40 seconds, an obstacle will magically appear, and your robot should stop 5-20 cm away from it.
#       4. Play the sound "Fanfare" or "Cheering", then exit.

# Robot information
EV3 = EV3Brick()
COLOR_SENSOR = ColorSensor(Port.S1)  # The color sensor that determines if the robot is on the black tape.
DISTANCE_SENSOR = UltrasonicSensor(Port.S4)  # The ultrasonic sensor to use to avoid obstacles.
LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.D
WHEEL_DIAMETER = 100  # The diameter of the robot wheels [mm].
AXLE_TRACK = 165  # The distance between the robot wheels [mm].
ROBOT = DriveBase(Motor(LEFT_MOTOR_PORT), Motor(RIGHT_MOTOR_PORT), WHEEL_DIAMETER, AXLE_TRACK)

# Settings
TOTAL_RUN_TIME = 10000  # The total run time before stopping to perform [ms].
INTERVAL = 500  # The intervals to update sensor data at [ms].


def play_dragonborn():
    """Plays some notes from Dragonborn using the onboard speaker."""
    NOTES = ['E3/4', 'E3/4', 'E3/4', 'C3/4', 'C3/4', 'C3/4', 'D3/4', 'D3/4', 'D3/4', 'A2/4', 'A2/4', 'F#3/8', 'G3/8']
    EV3.speaker.play_notes(NOTES, 220)
    EV3.speaker.play_notes(NOTES, 220)


def play_smash():
    """Plays a sad few notes from All Star using the onboard speaker."""
    NOTES = ['A2/2', 'E3/2', 'C#3/4', 'C#3/2', 'B2/4', 'A2/4', 'A2/2', 'D3/2', 'C#3/4', 'C#3/4', 'B2/4', 'B2/4', 'A2/4']
    EV3.speaker.play_notes(NOTES, 200)


def print_joke():
    """Prints a joke on screen, and then laughs at it"""
    EV3.screen.clear()
    EV3.screen.print("Why did the Java\nprogrammer go to the\n eye doctor?\n ... Because (s)he\n couldn't C#.")
    wait(3000)
    EV3.speaker.play_file(SoundFile.LAUGHING_1)


def perform_activity():
    """Performs a random activity unless the robot is stopping."""
    if stop:
        EV3.speaker.play_file(SoundFile.FANFARE)
    else:
        r = random.randint(1, 4)
        if r == 1:
            play_smash()
        elif r == 2:
            print_joke()
        elif r == 3:
            play_dragonborn()
        else:
            EV3.speaker.play_file(SoundFile.DOG_BARK_2)


stop = False

# While the robot does not have a signal to stop.
while not stop:
    for i in range(0, TOTAL_RUN_TIME / INTERVAL):
        distance = DISTANCE_SENSOR.distance()
        reflection = COLOR_SENSOR.reflection()

        EV3.screen.clear()
        EV3.screen.print("Distance: %s\nReflection: %s" % (distance, reflection))

        # If the distance from an obstacle is less than 200 mm, stop the program.
        if distance <= 200:
            stop = True
            break

        # While the robot is not on the line, turn.
        while COLOR_SENSOR.reflection() > 20:
            COLOR_SENSOR.reflection()
            ROBOT.turn(-3)

        # Drive until the next interval.
        ROBOT.drive(100, 0)
        wait(INTERVAL)
        ROBOT.stop()

    # For every time the total run time finishes, perform an activity.
    # If stop is true, play the "Fanfare".
    perform_activity()
