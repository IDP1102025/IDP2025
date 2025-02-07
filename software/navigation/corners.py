from sensors.sensor_manager import LineSensor

class CornerIdentification: 
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor):
        #Pin Assignment
        self.outer_left_sensor = outer_left_sensor
        self.inner_left_sensor = inner_left_sensor
        self.inner_right_sensor = inner_right_sensor
        self.outer_right_sensor = outer_right_sensor

    def find_turn(self,state_pattern):
        if state_pattern[0] == 1 or state_pattern[3] == 1:
            return True



