from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
import time

class Robot:
    # Constants - will be set from main

    
    x = 0
    y = 0
    theta = 0  # orientation angle in degrees
    front_drive_base = None
    color_sensor = None
    ultrasonic_sensor = None
    touch_sensor = None
    gyro_sensor = None
    claw_motor = None
    isCollecting = False
    sensorIndex = 0
    distanceToEndOfPath = 0
    distanceTraveled = 0
    thresholdAngle = 5
    pathOK = True
    obstacleDetectedLocations = []
    testingSensors = False
    ev3 = None

    def __init__(
        self,
        ev3,
        front_drive_base,
        color_sensor=None,
        ultrasonic_sensor=None,
        # touch_sensor=None,
        gyro_sensor=None,
        claw_motor=None
    ):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.ev3 = ev3
        self.front_drive_base = front_drive_base
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor
        # self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
        
        
        if self.gyro_sensor:
            self.gyro_sensor.reset_angle(0)
            self.ev3.speaker.beep()
            wait(10)
    
    def get_position(self):
        return (self.x, self.y)
    
    def set_position(self, x, y):
        self.x = x
        self.y = y

    def stop(self):
        self.front_drive_base.stop()

    def turn(self, angle):
        # First perform the turn
        self.front_drive_base.turn(angle)
        self.theta = (self.theta + angle) % 360
        
        # If gyro sensor available, snap to nearest 90-degree increment
        if self.gyro_sensor:
            current_gyro = self.gyro_sensor.angle()
            # Find the nearest 90-degree increment
            snapped_angle = round(current_gyro / 90) * 90
            # Calculate correction needed
            correction = snapped_angle - current_gyro
            
            # Apply correction if it's significant
            if abs(correction) > 2:
                self.front_drive_base.turn(correction)
                self.theta = snapped_angle % 360
            else:
                self.theta = snapped_angle % 360


    # Move straight while checking for obstacles
    def straight(self, distance):
        start_distance = self.front_drive_base.distance()
        target_distance = start_distance + distance
        
        self.front_drive_base.drive(200, 0) #drive forward at 200mm/s
        
        while abs(self.front_drive_base.distance() - target_distance) > 10:
            # Check for obstacles if ultrasonic sensor is available
            if self.ultrasonic_sensor:
                distanceToObstacle = self.ultrasonic_sensor.distance()
                
                # If object is detected, stop and collect it
                if distanceToObstacle < 50:  # 50mm threshold
                    self.front_drive_base.stop()
                    self.obstacleDetectedLocations.append((self.x, self.y))
                    
                    # Keep identifying material
                    material = self.identify_material()
                    #TODO: do something with material info

                    # ideally it will say whatever it found
                    self.ev3.speaker.say("Object detected: " + str(material))
                    
                    self.StartCollectItem()
                    
                    # Save position and heading before moving
                    start_x, start_y = self.x, self.y
                    saved_theta = self.theta
                    
                    self.ReturnToHomeBase()
                    
                    # Return to where we found the object and continue
                    self.GoToPosition(start_x, start_y)
                    self.ReturnToAngle(saved_theta)
                    
                    # Restart driving
                    self.front_drive_base.drive(200, 0)
            
            # Maintain heading if gyro sensor is available
            # if self.gyro_sensor:
            #     current_angle = self.gyro_sensor.angle()
            #     if abs(current_angle - self.theta) > self.thresholdAngle:
            #         self.front_drive_base.stop()
            #         self.turn(self.theta - current_angle)
            #         self.front_drive_base.drive(200, 0)
            
            # wait(50)
        
        self.front_drive_base.stop()
        
        # Update position based on heading
        self.x += distance * math.cos(math.radians(self.theta))
        self.y += distance * math.sin(math.radians(self.theta))

    def straightSimple(self, distance, forwards=1):
        self.front_drive_base.drive((distance * forwards), 0)
        # assuming default speed of 200mm/s
        time.sleep(distance / 200)
    
    def identify_material(self):
        if self.color_sensor:
            color = self.color_sensor.color()
            return color
        return None

    def clawOpen(self):
        if self.claw_motor:
            self.claw_motor.run_angle(200, 90)  # Open claw by 90 degrees

    def clawClose(self):
        if self.claw_motor:
            self.claw_motor.run_angle(200, -90)  # Close claw by 90 degrees

    def StartCollectItem(self):
        self.clawClose()
        self.isCollecting = True
        self.ev3.speaker.beep()
        # Add motor control for collection mechanism here
        
        self.EndCollectItem()

    def EndCollectItem(self):
        self.clawClose()
        self.isCollecting = False
        self.ev3.speaker.beep()

    def UpdateAndCheckAngle(self):
        if self.gyro_sensor:
            self.theta = self.gyro_sensor.angle()
            return self.theta
        return None

    def TestSensors(self):
        if self.touch_sensor and self.touch_sensor.pressed():
            self.ev3.speaker.beep()
            # Flip sensor index
            self.sensorIndex = (self.sensorIndex + 1) % 4
        
        while self.testingSensors:
            if self.sensorIndex == 0:
                if self.touch_sensor:
                    if self.touch_sensor.pressed():
                        self.ev3.speaker.beep()
            elif self.sensorIndex == 1:
                if self.ultrasonic_sensor:
                    distance = self.ultrasonic_sensor.distance()
                    print(distance)
                    
            elif self.sensorIndex == 2:
                if self.color_sensor:
                    color = self.color_sensor.color()
                    print(color)
                    
            elif self.sensorIndex == 3:
                if self.gyro_sensor:
                    angle = self.gyro_sensor.angle()
                    print(angle)
                   
            
            wait(50)
    
    def ReturnToAngle(self, targetAngle):
        # Logic to return to path after ReturnToHomeBase
        if self.gyro_sensor:
            currentAngle = self.gyro_sensor.angle()
            angleDiff = targetAngle - currentAngle
            self.turn(angleDiff)
        else:
            # If no gyro, turn to target angle based on current theta
            angleDiff = targetAngle - self.theta
            self.turn(angleDiff)

    def GoToPosition(self, target_x, target_y):
        # Calculate angle and distance to target
        deltaX = target_x - self.x
        deltaY = target_y - self.y
        distance = math.sqrt(deltaX**2 + deltaY**2)
        # already at target, no need to move
        if distance < 10: 
            return
            
        targetAngle = math.degrees(math.atan2(deltaY, deltaX))
        
        # Turn to face target
        angleDiff = targetAngle - self.theta
        self.turn(angleDiff)
        
        # Drive to target WITHOUT obstacle checking from internal drivebase function
        self.front_drive_base.straight(distance)
        
        # Update robot's internal position. Let's hope that the movement was accurate!
        self.x = target_x
        self.y = target_y

    def ReturnToHomeBase(self):
        # Return to home base at (0, 0)
        self.GoToPosition(self.HOME_BASE_X, self.HOME_BASE_Y)
    # Check with color sensor to see if near boundary (BLUE TAPE ON GROUND)
    def CheckIfOutOfBounds(self):
        if self.color_sensor:
            if self.color_sensor.color() == Color.BLUE:
                self.ev3.speaker.beep()
                return True
        return False

    # Starts in bottom left corner of the grid, facing upwards (theta=90), but absolute gyro is 0
    def StartGridMovement(self):
        for row in range(self.ROWS):
            for col in range(self.COLUMNS - 1):
                self.straight(self.TILE_WIDTH)  # Move forward one tile
                self.ev3.speaker.beep()
            
            if row < self.ROWS - 1:
                if row % 2 == 0:
                    self.turn(70)
                    self.straight(self.TILE_WIDTH)  # Move down to the next row
                    self.turn(70)  # Turn right to face the next row
                    self.ev3.speaker.beep()
                else:
                    self.turn(-70)  # Turn left at the end of the row
                    self.straight(self.TILE_WIDTH)  # Move down to the next row
                    self.turn(-70)  # Turn left to face the next row
                    self.ev3.speaker.beep()
                
                if self.CheckIfOutOfBounds():
                    self.ReturnToHomeBase()
                    self.ev3.speaker.say("Out of bounds detected. Returning to home base.")
        
        # After completing the grid, return to home base
        self.ReturnToHomeBase()
        self.ReturnToAngle(0)
        # Could restart grid movement here if needed
        # self.StartGridMovement()

    def FindAngleBetweenPoints(self, x1, y1, x2, y2):
        deltaY = y2 - y1
        deltaX = x2 - x1
        angleInRadians = math.atan2(deltaY, deltaX)
        angleInDegrees = math.degrees(angleInRadians)
        return angleInDegrees

    # Pythag theorem to find distance between 2 points
    def FindDistanceBetweenPoints(self, x1, y1, x2, y2):
        deltaY = y2 - y1
        deltaX = x2 - x1
        distance = math.sqrt(deltaX**2 + deltaY**2)
        return distance
