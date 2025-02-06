from machine import Pin, PWM
from time import sleep

# Note: Check pins are set to the correct GPIO pins

class DualMotor:
    def __init__(self, left_dir_pin, left_pwm_pin, right_dir_pin, right_pwm_pin):
        # Left motor setup
        self.left_dir = Pin(left_dir_pin, Pin.OUT)  # Direction pin for left motor
        self.left_pwm = PWM(Pin(left_pwm_pin))     # PWM pin for left motor
        self.left_pwm.freq(1000)                   # Set PWM frequency for left motor
        self.left_pwm.duty_u16(0)                  # Initialize duty cycle to 0
        
        # Right motor setup
        self.right_dir = Pin(right_dir_pin, Pin.OUT)  # Direction pin for right motor
        self.right_pwm = PWM(Pin(right_pwm_pin))     # PWM pin for right motor
        self.right_pwm.freq(1000)                    # Set PWM frequency for right motor
        self.right_pwm.duty_u16(0)                   # Initialize duty cycle to 0

    def stop(self):
        # Stop both motors
        self.left_pwm.duty_u16(0)
        self.right_pwm.duty_u16(0)

    def move_forward(self, left_speed=100, right_speed=100):
        # Move both motors forward at a given speed
        self.left_dir.value(0)  # 0 for forward
        self.right_dir.value(0)  # 0 for forward
        left_duty = int(65535 * left_speed / 100)  # Convert speed percentage to duty cycle
        right_duty = int(65535 * right_speed / 100)
        self.left_pwm.duty_u16(left_duty)
        self.right_pwm.duty_u16(right_duty)

    def move_backward(self, speed=100):
        # Move both motors backward at a given speed
        self.left_dir.value(1)  # 1 for backward
        self.right_dir.value(1)  # 1 for backward
        duty = int(65535 * speed / 100)  # Convert speed percentage to duty cycle
        self.left_pwm.duty_u16(duty)
        self.right_pwm.duty_u16(duty)

    def turn_left(self, speed=70):
        # Turn left by reducing or reversing the speed of the left motor
        self.left_dir.value(1)  # Reverse left motor
        self.right_dir.value(0)  # Forward right motor
        left_duty = int(65535 * speed / 100)  # Set speed for left motor
        right_duty = int(65535 * speed / 100)  # Set speed for right motor
        self.left_pwm.duty_u16(left_duty)
        self.right_pwm.duty_u16(right_duty)

    def turn_right(self, speed=70):
        # Turn right by reducing or reversing the speed of the right motor
        self.left_dir.value(0)  # Forward left motor
        self.right_dir.value(1)  # Reverse right motor
        left_duty = int(65535 * speed / 100)  # Set speed for left motor
        right_duty = int(65535 * speed / 100)  # Set speed for right motor
        self.left_pwm.duty_u16(left_duty)
        self.right_pwm.duty_u16(right_duty)

# Example usage:
#motor = DualMotor(left_dir_pin=4, left_pwm_pin=5, right_dir_pin=7, right_pwm_pin=6) # Initialize motor driver

#motor.move_forward(100,100)  # Move forward at 50% speed
# sleep(2)  # Wait for 2 seconds
# motor.turn_left(30)  # Turn left at 30% speed
# sleep(1)  # Wait for 1 second
# motor.turn_right(30)  # Turn right at 30% speed
# sleep(1)  # Wait for 1 second
# motor.move_backward(50)  # Move backward at 70% speed
#sleep(2)  # Wait for 2 seconds
#motor.stop()  # Stop both motors



