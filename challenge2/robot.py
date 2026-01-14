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

    def __init__(
        self,
        ev3,
        front_drive_base,
        color_sensor=None,
        # ultrasonic_sensor=None,
        # touch_sensor=None,
        gyro_sensor=None,
        claw_motor=None
    ):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.ev3 = ev3
        self.drive_base = drive_base
        self.color_sensor = color_sensor
        # self.ultrasonic_sensor = ultrasonic_sensor
        # self.touch_sensor = touch_sensor
        self.gyro_sensor = gyro_sensor
        self.claw_motor = claw_motor
        
        if self.gyro_sensor:
            self.gyro_sensor.reset_angle(0)
            self.ev3.speaker.beep()
            wait(10)
    #TODO: This may have some threading issues, so we may need to refactor later.
    # Test with the hardware, and maybe put everything into a while true loop.
    def Start(self):

        
        #check for constants, always running. I'm unsure if we can multithread like this, but we'll see during tests.

        while self.isCollecting:
            self.claw_motor.run(10)  # Apply constant torque to keep claw closed slightly

        while True:
            #If we run out of bounds, stop driving and go back
            if(self.CheckIfOutOfBounds()):  
                self.drive_base.stop()
                self.ev3.speaker.say("Returning to home base")
                self.drive_base.drive(-500)
                #continue keyword skips the rest of the code, so that we are no longer checking for materials, and just ensuring that we make it home.
                continue
            #Stop driving for a moment, so that the judge knows that we've noticed the material.
            if(self.CheckForMaterials()):
                isDriving = False
                wait(150)
                isDriving = True
                
            #TODO: Continously check for minerals
            if(self.isObjectInFront()):
                self.drive_base.stop()
                self.BeginCollectionSequence()
        
        #TODO: Drive forward until we hit something with either the touch sensor, or ultrasonic sensor
        

        #TODO: Begin collection sequence
       
        #Begin heading home
        self.BeginExtractionSequence()

             
        #TODO: Back up to starting position
        self.drive_base.drive(-500)
        
    def BeginCollectionSequence(self):
        self.ev3.speaker.say("Beginning Collection Sequence")
        #Drive forward slightly
        self.drive_base.straight(10)
        #TODO: experiment with the best values
        self.claw_motor.run_until_stopped(600,)  # Close claw

    def BeginExtractionSequence(self):
        self.ev3.speaker.say("Object secured, heading home")
        #Drive forward slightly
        self.drive_base.straight(10)

    def CheckForMaterials(self):
        if(self.color_sensor.color() == ColorSensor.COLOR_YELLOW):
            self.ev3.speaker.say("I've spotted gold!")
            return True
        if(self.color_sensor.color() == ColorSensor.COLOR_GREEN):
            self.ev3.speaker.say("Greener than a damn golf course here.")
            return True

    def CheckIfOutOfBounds(self):
        if self.color_sensor.color() == ColorSensor.COLOR_BLUE:
            self.ev3.speaker.say("Out of bounds!")
            return True
        return False

    def isObjectInFront(self):
        if self.ultrasonic_sensor.distance() < 65 or self.ultrasonic_sensor.distance() > 250 or self.touch_sensor.is_pressed():
            return True
        return False