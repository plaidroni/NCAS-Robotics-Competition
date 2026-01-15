from pybricks.parameters import Stop, Color
from pybricks.tools import wait
import math


class Robot:
    # Constants - will be set from main
    drive_base = None
    color_sensor = None
    ultrasonic_sensor = None
    touch_sensor = None
    gyro_sensor = None
    claw_motor = None
    ev3 = None
    isCollecting = False
    isDriving = False
    isDrivingForward = True
    turn_rate = 0
    target_heading = 0

    def __init__(
        self,
        ev3,
        front_drive_base,
        color_sensor=None,
        ultrasonic_sensor=None,
        touch_sensor=None,
        gyro_sensor=None,
        claw_motor=None
    ):
        self.x = 0
        self.y = 0
        self.turn_rate = 0
        self.ev3 = ev3
        self.isDriving = True
        self.target_heading = 0
        self.isDrivingForward = True
        self.drive_base = front_drive_base
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor
        self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
        self.claw_motor = claw_motor

        self.ev3.volume(100)

    #TODO: This may have some threading issues, so we may need to refactor later.
    # Test with the hardware, and maybe put everything into a while true loop.
    def Start(self):
        target_heading = self.gyro_sensor.angle()

        #check for constants, always running. I'm unsure if we can multithread like this, but we'll see during tests.

        while self.isCollecting:
            self.claw_motor.run(100)  # Apply constant torque to keep claw closed slightly

        while True:
            if self.isCollecting and self.claw_motor:
                self.claw_motor.run(10)

            if(self.isDriving and self.gyro_sensor):
                error = self.target_heading - self.gyro_sensor.angle()
                print("Gyro Error: ", error)
                self.ev3.screen.print("Gyro Error: {}".format(error))
                kP = 2.5
                turn_rate = kP * error
                if(self.isDrivingForward):
                    self.drive_base.drive(500, turn_rate)
                else:
                    self.drive_base.drive(-500, turn_rate)
                

            #If we run out of bounds, stop driving and go back
            if(self.CheckIfOutOfBounds()):  
                self.drive_base.stop()
                self.ev3.speaker.beep()
                self.drive_base.drive(-500)
                #continue keyword skips the rest of the code, so that we are no longer checking for materials, and just ensuring that we make it home.
                continue

            #Stop driving for a moment, so that the judge knows that we've noticed the material.
            #TODO: Continously check for minerals
            if(self.CheckForMaterials()):
                self.isDriving = False
                wait(150)
                self.isDriving = True
                
            #TODO: Drive forward until we hit something with either the touch sensor, or ultrasonic sensor
            if(self.isObjectInFront()):
                self.ev3.screen.print("Touch Sensor Pressed")
                self.ev3.screen.print("Ultrasonic Sensor Pressed")
                self.drive_base.stop()
                self.BeginCollectionSequence()
                self.BeginExtractionSequence()

            
        
        #add timer to come back

             
        #TODO: Back up to starting position
        self.drive_base.drive(-500)
        
    def BeginCollectionSequence(self):
        self.isCollecting = True
        self.ev3.speaker.beep()
        #Drive forward slightly
        self.drive_base.straight(10)
        #TODO: experiment with the best values
        if(self.claw_motor):
            self.claw_motor.run_until_stalled(2000,Stop.COAST,30)  # Close claw

    def BeginExtractionSequence(self):
        self.ev3.speaker.beep()
        self.target_heading = (self.target_heading + 180) % 360
        #If we want to try to fix the gyro issue, then we should set isDrivingForward to true here.
        self.isDrivingForward = False
        # self.isDrivingForward = True
        #Drive forward slightly

    def CheckForMaterials(self):
        if(self.color_sensor and self.color_sensor.color() == Color.YELLOW):
            self.ev3.speaker.beep()
            return True
        if(self.color_sensor and self.color_sensor.color() == Color.GREEN):
            self.ev3.speaker.beep()
            return True

    def CheckIfOutOfBounds(self):
        if self.color_sensor and self.color_sensor.color() == Color.BLUE:
            self.ev3.speaker.beep()
            return True
        return False

    def isObjectInFront(self):
        if self.ultrasonic_sensor:
            distance = self.ultrasonic_sensor.distance()
            if distance < 65 or distance > 250:
                return True
        if self.touch_sensor and self.touch_sensor.pressed():
            return True
        return False