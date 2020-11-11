#!/usr/bin/env pybricks-micropython
from threading import Thread

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Practice description:
#   Make a race car robot.
#   The program should do the following:
#       1. Drive around a field as fast as possible while following a black tape.

class RallyRobot:
    def __init__(self):
        """Initialize the RallyRobot with a DriveBase and two color-sensors."""
        # Robot information
        self.EV3 = EV3Brick()
        self.LEFT_COLOR_SENSOR = ColorSensor(Port.S1)
        self.RIGHT_COLOR_SENSOR = ColorSensor(Port.S4)
        LEFT_MOTOR_PORT = Port.A
        RIGHT_MOTOR_PORT = Port.D
        WHEEL_DIAMETER = 56  # The diameter of the robot wheels [mm].
        AXLE_TRACK = 200  # The distance between the robot wheels [mm].
        self.ROBOT = DriveBase(Motor(LEFT_MOTOR_PORT), Motor(RIGHT_MOTOR_PORT), WHEEL_DIAMETER, AXLE_TRACK)

        # Settings
        self.TURN_ANGLE = 35  # Turn angle when robot is not on the line [deg].
        self.TURN_SPEED = 70  # Turn speed when robot is not on the line [mm/s].
        self.DRIVE_SPEED = 200  # Drive speed when robot is on the line [mm/s].
        self.BLACK_THRESHOLD = 11  # Highest possible reflection where the robot assumes it is still on color "black".

    def update_sensor_data(self):
        """Gets sensor data and prints it to the screen."""
        reflection_left = self.LEFT_COLOR_SENSOR.reflection()
        reflection_right = self.RIGHT_COLOR_SENSOR.reflection()
        turning = self.needs_turn()

        self.EV3.screen.clear()
        self.EV3.screen.print(
            "Reflection L: %s\nReflection R: %s\nTurning: %s" % (reflection_left, reflection_right, turning))

    def needs_turn(self):
        """Returns true if robot needs to turn, and false if not."""
        LEFT_BLACK = self.LEFT_COLOR_SENSOR.reflection() < self.BLACK_THRESHOLD
        return not LEFT_BLACK

    def get_turn_direction(self, last_turn_direction):
        """Get what direction needs to be turned."""
        RIGHT_BLACK = self.RIGHT_COLOR_SENSOR.reflection() < self.BLACK_THRESHOLD

        if RIGHT_BLACK or last_turn_direction == 1:  # If this is true, turn right.
            return 1
        else:  # If not, turn left.
            return -1

    def run(self):
        """Runs the rally robot."""
        while True:
            turn_direction = None

            while self.needs_turn():
                turn_direction = self.get_turn_direction(turn_direction)
                self.ROBOT.drive(self.TURN_SPEED, turn_direction * self.TURN_ANGLE)  # Get angle to turn

            self.ROBOT.drive(self.DRIVE_SPEED, 0)  # Drive and turn given angle. Turns while driving.


rally_robot = RallyRobot()  # Make new instance of RallyRobot.


# This function is supposed to run in a separate thread to keep updating sensor info.
def update_sensor_data():
    while True:
        rally_robot.update_sensor_data()
        wait(500)


# Initialize thread.
st = Thread(target=update_sensor_data)
st.start()

# Run the robot.
rally_robot.run()
