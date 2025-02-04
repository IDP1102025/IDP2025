import time
from sensors.sensor_manager import LineSensor

class NotPidFollower:
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor):
        # Pin Assignment
        self.outer_left_sensor = outer_left_sensor
        self.inner_left_sensor = inner_left_sensor
        self.inner_right_sensor = inner_right_sensor
        self.outer_right_sensor = outer_right_sensor

        self.error_loop = 0

    def follow_the_line(self, current_left_speed, current_right_speed):
        # 1) Read sensors
        outer_left_detect  = self.outer_left_sensor.read_sensor()   # 0 or 1
        inner_left_detect  = self.inner_left_sensor.read_sensor()
        inner_right_detect = self.inner_right_sensor.read_sensor()
        outer_right_detect = self.outer_right_sensor.read_sensor()

        # 2) Determine sensor pattern
        state_pattern = [outer_left_detect,
                         inner_left_detect,
                         inner_right_detect,
                         outer_right_detect]
        
        if state_pattern == [0,1,1,0]:
            self.error_loop += 1
            current_left_speed, current_right_speed = max(current_right_speed,current_left_speed)
            if self.error_loop >= 10:
                current_left_speed += 5
                current_right_speed += 5
            return current_left_speed, current_right_speed
        elif state_pattern in ([0,0,1,0], [0,1,0,0], [0,1,0,1], [1,0,1,0]):
            return current_left_speed, current_right_speed
        else:
            self.error_loop = 0
            if state_pattern == [0,0,1,1]:  # slightly left
                current_left_speed += 5
                current_right_speed -= 5
            elif state_pattern == [1,1,0,0]:   # slightly right
                current_left_speed -= 5
                current_right_speed += 5
            elif state_pattern == [0,0,0,1]: # bigger correction
                current_left_speed += 10
                current_right_speed -= 10        
            elif state_pattern == [1,0,0,0]:
                current_left_speed -= 10
                current_right_speed += 10 
            return current_left_speed, current_right_speed
