from machine import Pin, PWM
import time
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from pid.pid import LineFollower
from sensors.sensor_manager import LineSensor, CrashSensor, UltraSound
from sensors.code_reader import CodeReader
class Robot :
    def __init__(self):
        '''
        Inititalise all sensors, motors, linefollower class
        - 4 line sensors (inside PID class anyway)
        - 1 dual motor driver done
        - 1 ultrasonic sensor done
        - 1 qr code scanner done
        - 1 linear actuator
        - 1 led 
        - 1 push bottom (start/stop/reset)

        Robot variables + states:
        - Current task (idle,line following forward, reversing, turning left/right, picking up box, dropping box, scanning qr code) 
        - Direction facing (Front, right, back, left) (represented as 1,2,3,4)
        - Current position 
        - Next target position
        - Goal position
        - Time elapsed
        - Current speed % (0-100)
        - Boxes in each depot (4,3,2,1,0)
        - Robot running (True/False)
        '''

        # Initialising Motors and actuators
        self.dual_motors = DualMotor(left_dir_pin=6, left_pwm_pin=7, right_dir_pin=5, right_pwm_pin=4)
        self.linear_actuator = LinearActuatorDriver(None,None) # add pins for linear actuator

        # Init Sensors
        self.ultrasonic_sensor = UltraSound("""add pins""")

        self.code_scanner = CodeReader("""add pins""")
        self.outer_left_sensor = LineSensor("""add pins""")
        self.inner_left_sensor = LineSensor("""add pins""")
        self.inner_right_sensor = LineSensor("""add pins""")
        self.outer_right_sensor = LineSensor("""add pins""")

        # Init line follower
        



