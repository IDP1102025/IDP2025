import time
from sensors.sensor_manager import LineSensor

class CornerIdentification: 
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor):
        #Pin Assignment
        self.outer_left_sensor = LineSensor(outer_left_sensor)
        self.inner_left_sensor = LineSensor(inner_left_sensor)
        self.inner_right_sensor = LineSensor(inner_right_sensor)
        self.outer_right_sensor = LineSensor(outer_right_sensor)

    def find_turn(self):
        while True:
            
            # how many times the sensors must detect a node consecutively before confirming its presence
            detection_factor = 0
            
            while detection_factor < 3: 
                outer_left_detect, inner_left_detect = self.outer_left_sensor.read_sensor(), self.inner_left_sensor.read_sensor()
                inner_right_detect, outer_right_detect = self.inner_right_sensor.read_sensor(), self.outer_right_sensor.read_sensor()

                state_pattern = [outer_left_detect, inner_left_detect, inner_right_detect, outer_right_detect]

                if state_pattern == [0, 1, 1, 1] or state_pattern == [1, 1, 1, 0] or state_pattern == [1, 1, 1, 1]:
                    detection_factor += 1

                if detection_factor == 3: 
                    return True
                
                #if the method returns True, need to pop one item from the path list in robot class and effectuate turn for the node it knows it is at
            
                time.sleep(0.1)

            


