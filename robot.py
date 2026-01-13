class Robot:
    x = 0
    y = 0
    front_drive_base = None
    back_drive_base = None
    color_sensor = None
    ultrasonic_sensor = None
    touch_sensor = None

    def __init__(self, front_drive_base, back_drive_base=None, color_sensor=None, ultrasonic_sensor=None, touch_sensor=None):
        self.x = 0
        self.y = 0
        self.front_drive_base = front_drive_base
        self.back_drive_base = back_drive_base
        self.color_sensor = color_sensor
        self.ultrasonic_sensor = ultrasonic_sensor
        self.touch_sensor = touch_sensor

    def get_position(self):
        return (self.x, self.y)
    def set_position(self, x, y):
        self.x = x
        self.y = y