class Robot:
    # Constants - will be set from main
    drive_base = None
    color_sensor = None
    ultrasonic_sensor = None
    touch_sensor = None
    gyro_sensor = None
    claw_motor = None
    ev3 = None

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
    
    def Start(self):
        self.drive_base.straight(1000)
        #TODO: Drive forward until we hit something with either the touch sensor, or ultrasonic sensor
        #TODO: Begin collection sequence
             #Drive forward slightly
             #Close Claw (apply constant motor torque)
        #TODO: Back up to starting position
        
