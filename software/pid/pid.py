import RPi.GPIO as GPIO
import time

#Sensors output:
#   1 for White
#   0 for Black

#Setting up GPIO pins

left_sensor_pin = 
right_sensor_pin = 

GPIO.setmode(GPIO.BCM)

GPIO.setup(left_sensor_pin, GPIO.IN)
GPIO.setup(right_sensor_pin, GPIO.IN)

#For testing

try: 
    while True: 
        left_sensor = GPIO.input(left_sensor_pin)
        right_sensor = GPIO.input(right_sensor_pin)

        if left_sensor == 0 and right_sensor == 0:
            print("Straight")

        if left_sensor == 1 and right_sensor == 0:
            print("Turning Right")

        if left_sensor == 0 and right_sensor == 1:
            print("Turn Left")

        time.sleep(0.1)

except KeyboardInterrupt: 
    print("Test over.")
    GPIO.cleanup()

#PID CONTROL
    
class LineFollower: 
    def __init__(self, left_sensor_pin, right_sensor_pin, left_motor_pins, right_motor_pins, kp, ki, kd):
        #Pin Assignment
        self.left_sensor_pin = left_sensor_pin
        self.right_sensor_pin = right_sensor_pin
        self.left_motor_pins = left_motor_pins
        self.right_motor_pis = right_motor_pins

        #PID Constants
        self.kp = kp
        self.ki = ki
        self.kd = kd

        #PID Variables
        self.previous_error = 0
        self.integral = 0

        #Setup GPIO inputs
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(left_sensor_pin, GPIO.IN)
        GPIO.setup(right_sensor_pin, GPIO.IN)
        
        for pin in self.left_motor_pins + self.right_motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        #Initialise PWM for motors

        self.left_pwm = GPIO.PWM(self.left_motor_pins[0], 100)
        self.right_pwm = GPIO.PWM(self.right_motor_pin[0], 100)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
    
    def read_sensors(self):
        left_reading = GPIO.input(self.left_sensor_pin)
        right_reading = GPIO.input(self.right_sensor_pin)
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

        self.left_pwm.ChangeDutyCycle(left_speed)
        self.right_pwm.ChangeDutyCycle(right_speed)

    def follow_line(self, target_position = 0, base_speed = 50):
        while True: 
            left_detect, right_detect = self.read_sensors()

            current_position = 1 if left_detect == 1 else -1 if right_detect == 1 else 0

            correction = self.calculate_pid(target_position, current_position)

            self.adjust_motors(base_speed, correction)

            time.sleep(0.1)

    def stop(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()