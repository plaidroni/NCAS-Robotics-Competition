from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math

class Robot:
    x = 0
    y = 0
    theta = 0  # orientation angle in degrees
    front_drive_base = None
    back_drive_base = None
    color_sensor = None
    ultrasonic_sensor = None
    touch_sensor = None
    gyro_sensor = None
    isCollecting = False
    sensorIndex = 0
    distanceToEndOfPath = 0
    distanceTraveled = 0
    thresholdAngle = 5
    pathOK = True
    obstacleDetectedLocations = []


    def __init__(
    self,
     front_drive_base,
    #   back_drive_base=None,
       color_sensor=None,
        ultrasonic_sensor=None,
         touch_sensor=None,
          gyro_sensor=None
          ):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.front_drive_base = front_drive_base
        # self.back_drive_base = back_drive_base
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor
        self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
    def get_position(self):
        return (self.x, self.y)
    def set_position(self, x, y):
        self.x = x
        self.y = y

    def 

    def turn(self, angle):
        self.front_drive_base.turn(angle)
        self.theta = (self.theta + angle) % 360
        # if self.back_drive_base:
        #     self.back_drive_base.turn(angle)
    #if robot finds an obstacle on the ultrasonic sensor, it should stop
    def straight(self, distance):
        self.front_drive_base.drive(distance)

        while self.pathOK:
            distanceToObstacle = self.ultrasonic_sensor.distance()
            #return to 90, 0, 180, 270 heading
            if self.gyro_sensor.angle() > self.theta + self.thresholdAngle or self.gyro_sensor.angle() < self.theta - self.thresholdAngle:
                self.turn(self.theta - self.gyro_sensor.angle())
            identifyMaterial()
                # if object is detected, stop and collect them. we should then return to path
            if distanceToObstacle < 10:
                self.front_drive_base.stop()
                if self.back_drive_base:
                    self.back_drive_base.stop()
                self.obstacleDetectedLocations.append((self.x, self.y))
                self.StartCollectItem()
                self.ReturnToHomeBase(self.x, self.y)
                self.pathOK = False
                break

        # if self.back_drive_base:
        #     self.back_drive_base.straight(distance)

        #make distance calculation based on heading
        self.x += distance * math.cos(math.radians(self.theta))
        self.y += distance * math.sin(math.radians(self.theta))
# from docs
#     color()
# Measures the color of a surface.

# ambient()
# Measures the ambient light intensity.

# rgb()
# Measures the reflection of a surface using a red, green, and then a blue light.

#TODO: implement identify material function and beep based on material
    def identify_material(self):
        if self.color_sensor:
            color = self.color_sensor.color()
            return color
        return None

    def StartCollectItem(p):
        # TODO: Implement item collection logic / motor control to start collection
        ev3.speaker.beep()
    #raise collection motor
    def EndCollectItem():

    def UpdateAndCheckAngle():
        if self.gyro_sensor:
            self.theta = self.gyro_sensor.angle()
            return self.theta
        return None

    def TestSensors():
        if(self.touch_sensor.pressed()):
            ev3.speaker.beep()
            #flip
            self.sensorIndex = (self.sensorIndex + 1) % 4
        
        while self.testingSensors:
            switch self.sensorIndex:
                case 0:
                    if self.touch_sensor:
                        if self.touch_sensor.pressed():
                            ev3.speaker.beep()
                case 1:
                    if self.ultrasonic_sensor:
                        distance = self.ultrasonic_sensor.distance()
                        print(distance)
                        ev3.speaker.beep(frequency=440 + distance, duration=100)
                case 2:
                    if self.color_sensor:
                        color = self.color_sensor.color()
                        print(color)
                        ev3.speaker.beep(frequency=440 + color * 100, duration=100)
                case 3:
                    if self.gyro_sensor:

            wait(50)
          
    def ReturnToAngle(targetAngle):
        #logic to return to path after ReturnToHomeBase
        currentAngle = self.gyro_sensor.angle()
        angleDiff = targetAngle - currentAngle
        self.turn(angleDiff)

    #starts in bottom left corner of the grid, facing upwards
    def StartGridMovement():
        for row in range(ROWS):
            for col in range(COLUMNS - 1):
                self.straight(TILE_WIDTH)  # move forward one tile
                ev3.speaker.beep()
            if row < ROWS - 1:
                if row % 2 == 0:
                    self.turn(90)
                    self.straight(TILE_WIDTH)  # move down to the next row.
                    self.turn(90)  # turn right to face the next row.
                else:
                    self.turn(-90)  # turn left at the end of the row.
                    self.straight(TILE_WIDTH)  # move down to the next row.
                    self.turn(-90)  # turn left to face the next row.
                    if(CheckIfOutOfBounds()):
                        return
        # after completing the grid, turn around to face the starting grid position and drive
        self.ReturnToHomeBase(self.x, self.y)
        self.ReturnToAngle(0)
        self.StartGridMovement()