#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math


#constants
WHEEL_DIAMETER = 55.5  # Wheel diameter in millimeters.
AXLE_TRACK = 104       # Distance between the centers of the two wheels.
LENGTH_ARENA = 1800  # Arena dimensions in millimeters.
WIDTH_ARENA = 1800  # Arena dimensions in millimeters.
#we should find the numbers of rows and columns during our walkthrough of the arena
ROWS = 3  # Number of rows of tiles.
COLUMNS = 3  # Number of columns of tiles.
TILE_WIDTH = LENGTH_ARENA // COLUMNS  # Size of each tile in millimeters.
HOME_BASE_X = 0
HOME_BASE_Y = 0

#mineral dictionary
MINERALS = []


# Initialize the EV3 Brick.
ev3 = EV3Brick()

#Initialize the sensors
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S2)
touch_sensor = TouchSensor(Port.S3)

# Initialize the motors.
front_left_motor = Motor(Port.D)
front_right_motor = Motor(Port.C)
back_left_motor = Motor(Port.B)
back_right_motor = Motor(Port.A)

# Initialize the drive base.
front_drive_base = DriveBase(front_left_motor, front_right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
back_drive_base = DriveBase(back_left_motor, back_right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

#initialize the robot
robot = Robot(front_drive_base=front_drive_base, back_drive_base=back_drive_base, color_sensor=color_sensor, ultrasonic_sensor=ultrasonic_sensor, touch_sensor=touch_sensor)


#starts in bottom left corner of the grid, facing upwards
def StartGridMovement():
    for row in range(ROWS):
        for col in range(COLUMNS - 1):
            robot.straight(TILE_WIDTH)  # move forward one tile
            ev3.speaker.beep()
        if row < ROWS - 1:
            if row % 2 == 0:
                robot.turn(90)
                robot.straight(TILE_WIDTH)  # move down to the next row.
                robot.turn(90)  # turn right to face the next row.
            else:
                robot.turn(-90)  # turn left at the end of the row.
                robot.straight(TILE_WIDTH)  # move down to the next row.
                robot.turn(-90)  # turn left to face the next row.
                if(CheckIfOutOfBounds()):
                    return
    # after completing the grid, turn around to face the starting grid position
    ReturnToHomeBase(robot.x, robot.y)

def CheckIfOutOfBounds():
    distance = ultrasonic_sensor.distance()
    #temp distance
    if distance < 50:
        ev3.speaker.beep()
        robot.stop()
        return True
    return False

def FindAngleBetweenPoints(x1, y1, x2, y2):
    deltaY = y2 - y1
    deltaX = x2 - x1
    angleInRadians = math.atan2(deltaY, deltaX)
    angleInDegrees = math.degrees(angleInRadians)
    return angleInDegrees

#pythag theorem to find dist between 2 points
def FindDistanceBetweenPoints(x1, y1, x2, y2):
    deltaY = y2 - y1
    deltaX = x2 - x1
    distance = math.sqrt(deltaX**2 + deltaY**2)
    return distance

def ReturnToHomeBase(currentX, currentY):
    angleToHome = FindAngleBetweenPoints(currentX, currentY, HOME_BASE_X, HOME_BASE_Y)
    distanceToHome = FindDistanceBetweenPoints(currentX, currentY, HOME_BASE_X, HOME_BASE_Y)
    robot.turn(angleToHome)
    robot.straight(distanceToHome)

def StartCollectItem():
    # TODO: Implement item collection logic / motor control to start collection
    ev3.speaker.beep()
#raise collection motor
def EndCollectItem():
