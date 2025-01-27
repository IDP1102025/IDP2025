from machine import Pin, PWM
import time

#Sensors output:
#   1 for White
#   0 for Black

#Setting up GPIO pins

left_sensor_pin = 
right_sensor_pin = 

left_sensor = Pin(left_sensor_pin, Pin.IN)
right_sensor = Pin(right_sensor_pin, Pin.IN)

#PID CONTROL
    
class LineFollower: 
    def __init__(self, left_sensor_pin, right_sensor_pin, left_motor_pins, right_motor_pins, kp, ki, kd):
        #Pin Assignment
        self.left_sensor_pin = left_sensor_pin
        self.right_sensor_pin = right_sensor_pin
        self.left_motor_pins = [Pin(pin, Pin.OUT) for pin in left_motor_pins]
        self.right_motor_pins = [Pin(pin, Pin.OUT) for pin in right_motor_pins]
        #PID Constants
        self.kp = kp
        self.ki = ki
        self.kd = kd

        #PID Variables
        self.previous_error = 0
        self.integral = 0

        left_sensor = Pin(left_sensor_pin, Pin.IN)
        right_sensor = Pin(right_sensor_pin, Pin.IN)

        #Initialise PWM for motors

        self.left_pwm = PWM(self.left_motor_pins[0])
        self.right_pwm = PWM(self.right_motor_pins[0])
        self.left_pwm.freq(1000)
        self.right_pwm.freq(1000)
    
    def read_sensors(self):
        left_reading = self.left_sensor.value()
        right_reading = self.right_sensor.value()
        return left_reading, right_reading
    
    def calculate_pid(self, current, target):
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

        self.left_pwm.duty_u16(left_speed / 100 * 65535)
        self.right_pwm.duty_u16(right_speed / 100 * 65535)

    def follow_line(self, target_position = 0, base_speed = 50):
        while True: 
            left_detect, right_detect = self.read_sensors()

            current_position = 0

            if left_detect == 0 and right_detect == 0:
                current_position = 0
            elif left_detect == 1 and right_detect == 0:
                current_position = 0
            elif left_detect == 0 and right_detect == 1:
                current_position = -1
            
            correction = self.calculate_pid(target_position, current_position)

            self.adjust_motors(base_speed, correction)

            time.sleep(0.1)

    def stop(self):
        self.left_pwm.duty_u16(0)
        self.right_pwm.duty_u16(0)