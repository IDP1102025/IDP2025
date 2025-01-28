from machine import Pin, PWM
import time
from sensors.sensor_manager import LineSensor
from controllers.motor_driver import DualMotor

#Sensors output:
#   1 for White
#   0 for Black

#Setting up GPIO pins

left_sensor_pin = 
right_sensor_pin = 

left_sensor = Pin(left_sensor_pin, Pin.IN)
right_sensor = Pin(right_sensor_pin, Pin.IN)


#PID CONTROL

#sensor_pins list = [left left, inner left, inner right, right right]
    
class LineFollower: 
    def __init__(self, sensor_pins, left_motor_pins, right_motor_pins, kp_list, ki, kd):
        #Pin Assignment
        self.outer_left_sensor = LineSensor(sensor_pins[0])
        self.inner_left_sensor = LineSensor(sensor_pins[1])
        self.inner_right_sensor = LineSensor(sensor_pins[2])
        self.outer_right_sensor = LineSensor(sensor_pins[3])
       
        self.dual_motors = DualMotor(left_motor_pins[0],left_motor_pins[1],right_motor_pins[0],right_motor_pins[1])

        #PID Constants
        self.kp_low = kp_list[0]
        self.kp_high = kp_list[1]
        self.ki = ki
        self.kd = kd

        #PID Variables
        self.previous_error = 0
        self.integral = 0
    
    def calculate_pid(self, current, target, kp):
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        correction = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        return correction
    
    def adjust_motors(self, correction, base_speed):
        left_speed = base_speed - correction
        right_speed = base_speed + correction

        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        self.dual_motors.left_pwm.duty_u16(left_speed / 100 * 65535)
        self.dual_motors.right_pwm.duty_u16(right_speed / 100 * 65535)

    def follow_line(self, target_position = 0, base_speed = 50):
        while True: 
            outer_left_detect, inner_left_detect = self.outer_left_sensor.read_sensor(), self.inner_left_sensor.read_sensor()
            inner_right_detect, outer_right_detect = self.inner_right_sensor.read_sensor(), self.outer_right_sensor.read_sensor()

            state_pattern = [outer_left_detect, inner_left_detect, inner_right_detect, outer_right_detect]

            current_position = 0

            if state_pattern == [0, 1, 1, 0]:
                current_position = 0
            elif state_pattern == [0, 0, 1, 1]:
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
                
            if current_position ==  1 or abs(current_position) == 1 or abs(current_position) == 2:
                correction = self.calculate_pid(target_position, current_position, kp)
                self.adjust_motors(base_speed, correction)
            elif current_position == 3: 
                self.dual_motors.move_backward(50)

            time.sleep(0.1)