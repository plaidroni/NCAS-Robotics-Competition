#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
from robot import Robot
# Initialize the EV3 Brick.
ev3 = EV3Brick()

#constants
WHEEL_DIAMETER = 35  # Wheel diameter in millimeters.
AXLE_TRACK = 230       # Distance between the centers of the two wheels.
LENGTH_ARENA = 3500  # Arena dimensions in millimeters.
WIDTH_ARENA = 1740  # Arena dimensions in millimeters.
#we should find the numbers of rows and columns during our walkthrough of the arena
ROWS = 3  # Number of rows of tiles.
COLUMNS = 6  # Number of columns of tiles.
TILE_WIDTH = LENGTH_ARENA // COLUMNS  # Size of each tile in millimeters.
HOME_BASE_X = 0
HOME_BASE_Y = 0


#mineral dictionary
MINERALS = []

# changed the commented out stuff here because when we init the robot object, we pass those sensors ykyk
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S2)
touch_sensor = TouchSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)

# Initialize the motors.
front_left_motor = Motor(Port.D)
front_right_motor = Motor(Port.C)
grabber_motor = Motor(Port.B)



# Initialize the drive base.
drive_base = DriveBase(front_left_motor, front_right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

#initialize the robot
robot = Robot(
        ev3=ev3,
        front_drive_base=drive_base,
        color_sensor=color_sensor,
        touch_sensor=touch_sensor,
        ultrasonic_sensor=ultrasonic_sensor,
        gyro_sensor=gyro_sensor,
        claw_motor=grabber_motor
    )

# Set constants for the robot
robot.ROWS = ROWS
robot.COLUMNS = COLUMNS
robot.TILE_WIDTH = TILE_WIDTH
robot.HOME_BASE_X = HOME_BASE_X
robot.HOME_BASE_Y = HOME_BASE_Y

#TODO: put in intialize function here
robot.Start()