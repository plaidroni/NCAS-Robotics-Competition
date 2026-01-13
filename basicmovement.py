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


# Initialize the EV3 Brick.
ev3 = EV3Brick()

#Initialize the sensors
color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S2)
# touch_sensor = TouchSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S4)

# Initialize the motors.
front_left_motor = Motor(Port.D)
front_right_motor = Motor(Port.C)
grabber_motor = Motor(Port.B)
# back_left_motor = Motor(Port.B)
# back_right_motor = Motor(Port.A)

# Initialize the drive base.
front_drive_base = DriveBase(front_left_motor, front_right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
# back_drive_base = DriveBase(back_left_motor, back_right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

#initialize the robot
robot = Robot(
        front_drive_base=front_drive_base,
        # back_drive_base=back_drive_base,
        color_sensor=color_sensor,
        ultrasonic_sensor=ultrasonic_sensor,
        gyro_sensor=gyro_sensor,
        claw_motor=grabber_motor,
    )
#drive straight until obstacle is detected, then drive up and collect the object then stop
def DriveUntilObstacleAndCollect():
    while True:
        distance = robot.ultrasonic_sensor.distance()

        robot.checkMaterials()

        if distance < 50: #percentage distance to obstacle
            robot.StartCollectItem()
            robot.drive(-200)  # go back the total distance traveled
            robot.ev3.speaker.beep()
            break

        # just because ultasonic sensors can be a bit iffy sometimes
        elif distance > 500:
            distance = 500

        robot.straightSimple(distance)

# DriveUntilObstacleAndCollect()

# while True:
#     checkMaterials()