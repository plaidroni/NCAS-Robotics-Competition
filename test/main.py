#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

# Initialize the EV3 Brick
ev3 = EV3Brick()

# ============================================================================
# CONFIGURE PORTS HERE
# ============================================================================
TOUCH_SENSOR_PORT = Port.S3      # Change to desired port
COLOR_SENSOR_PORT = Port.S2      # Change to desired port
GYRO_SENSOR_PORT = Port.S1       # Change to desired port
MOTOR_1_PORT = Port.D            # Change to desired port
MOTOR_2_PORT = Port.A            # Change to desired port

# ============================================================================
# Initialize Sensors
# ============================================================================
touch_sensor = TouchSensor(TOUCH_SENSOR_PORT)
color_sensor = ColorSensor(COLOR_SENSOR_PORT)
gyro_sensor = GyroSensor(GYRO_SENSOR_PORT)

# Initialize Motors
motor_1 = Motor(MOTOR_1_PORT)
motor_2 = Motor(MOTOR_2_PORT)

# ============================================================================
# Test Functions
# ============================================================================


gyro_sensor.reset_angle(0)
counter = 1
while True:
    if touch_sensor.pressed():
        ev3.speaker.beep(frequency=2000, duration=200)

    if colorsensor.color() == Color.GREEN:
        ev3.speaker.say("Green detected")
    elif colorsensor.color() == Color.BLUE:
        ev3.speaker.say("Blue detected")
    elif colorsensor.color() == Color.YELLOW:
        ev3.speaker.say("Yellow detected")

    ev3.screen.print(str(gyro_sensor.angle()))
    counter += 1
    if counter % 2 == 0:
        drive_base.drive(200)
    else:
        drive_base.drive(-200)
    wait(100)