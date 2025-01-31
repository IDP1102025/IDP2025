import time
from sensors.sensor_manager import LineSensor

#Sensors output:
#   1 for White
#   0 for Black

#PID CONTROL

#sensor_pins list = [left left, inner left, inner right, right right]
    
class LineFollower: 
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor, kp_high, kp_low, ki, kd):
        #Pin Assignment
        self.outer_left_sensor = LineSensor(outer_left_sensor)
        self.inner_left_sensor = LineSensor(inner_left_sensor)
        self.inner_right_sensor = LineSensor(inner_right_sensor)
        self.outer_right_sensor = LineSensor(outer_right_sensor)

        #PID Constants
        self.kp_low = kp_low
        self.kp_high = kp_high
        self.ki = ki
        self.kd = kd

        #PID Variables
        self.previous_error = 0
        self.integral = 0

        #if no error detected by sensors, this is incremented until 10, where the vehicle will accelerate due to linear drive
        self.error_loop = 0

    #calculate_pid returns the adjustment value to the wheel speeds of the robot
    def calculate_pid(self, current, target, kp):
        error = target - current
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        derivative = error - self.previous_error
        self.previous_error = error

        correction = (kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        return correction
    
    #correction is determined from the calculate_pid function
    def adjust_motors(self, correction, base_speed):
        left_speed = base_speed - correction
        right_speed = base_speed + correction

        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        
        #return the corrected/adjusted speeds, 0 indicates forwards direction
        return (left_speed, right_speed, 0)

    def follow_line(self, target_position = 0, base_speed = 50):
        while True: 
            outer_left_detect, inner_left_detect = self.outer_left_sensor.read_sensor(), self.inner_left_sensor.read_sensor()
            inner_right_detect, outer_right_detect = self.inner_right_sensor.read_sensor(), self.outer_right_sensor.read_sensor()

            state_pattern = [outer_left_detect, inner_left_detect, inner_right_detect, outer_right_detect]

            current_position = 0

            if state_pattern == [0, 1, 1, 0]:
                current_position = 0
                self.error_loop += 1
            else:
                self.error_loop = 0

                if state_pattern == [0, 0, 1, 1]:
                    current_position = -1
                    kp = self.kp_low
                elif state_pattern == [1, 1, 0, 0]:
                    current_position = 1
                    kp = self.kp_low
                elif state_pattern == [0, 0, 0, 1]:
                    current_position = -2
                    kp = self.kp_high
                elif state_pattern == [1, 0, 0, 0]:
                    current_position = 2
                    kp = self.kp_high
                elif state_pattern == [0, 0, 0, 0]:
                    current_position = 3
                
            # this code returns left_speed, right_speed, and direction = 0 unless the robot strays from the line, in which it returns backwards speed and direction = 1
            if self.error_loop == 10 and abs(current_position) == 0:
                self.adjust_motors(0, base_speed*1.5)
            if abs(current_position) == 1 or abs(current_position) == 2:
                correction = self.calculate_pid(current_position, target_position, kp)
                self.adjust_motors(correction, base_speed)
            elif current_position == 3: 
                self.recover_off_the_line()

            time.sleep(0.1)

    def recover_off_the_line(self):
        backwards_speed = 20
        # speed and direction - returns values, does not change motor speed directly
        return (backwards_speed, 1)