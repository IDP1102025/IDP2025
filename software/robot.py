from machine import Pin, PWM
import time
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from pid.pid import LineFollower
from sensors.sensor_manager import LineSensor, CrashSensor, UltraSound
from sensors.code_reader import CodeReader
from collections import deque
from time import time , sleep

class Robot :
    def __init__(self):
        '''
        Inititalise all sensors, motors, linefollower class done
        - 4 line sensors (inside PID class anyway)
        - 1 dual motor driver done`
        - 1 ultrasonic sensor done
        - 1 qr code scanner done
        - 1 linear actuator done
        - 1 led done
        - 1 push bottom (start/stop/reset) done
        - initialise navigation and graph

        Robot variables + states:
        - Current task (idle,line following forward, reversing, turning left/right, picking up box, dropping box, scanning qr code) 
        - Direction facing (Front, right, back, left) (represented as 1,2,3,4)
        - Current position 
        - Next target position
        - Goal position
        - Time elapsed
        - Current speed % (0-100)
        - Boxes in each depot (4,3,2,1,0)

        robot path should be a list of tuples 
        '''
        # Initialise LED and button
        self.led = Pin(14, Pin.OUT)
        self.button = Pin(12, Pin.IN, Pin.PULL_DOWN)

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
        self.line_follower = LineFollower(None, self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor,"""add kp,ki ,kd""")

        # Init navigaton
        # TODO

        # Robot variables and states
        self.current_task = "idle"
        self.direction_facing = 1
        self.current_position = "Start"
        self.goal_position = "Depot 1"
        self.next_target_position = "Depot 1"
        self.time_elapsed = 0
        self.current_speed = 0
        self.boxes_in_depot = {"Depot 1":4, "Depot 2":4}
        self.depot_loop_running = False
        self.robot_path = [] # deque of tuples containing directions of travel and number of nodes to pass (1,1)

    def robot_standby(self):
        while True:
            if self.button.value() == 1:
                self.start()
                break
    
    def start(self):
        raise NotImplementedError
    
    def return_to_start(self):
        raise NotImplementedError
    
    def goto_node(self,target_node):

        raise NotImplementedError

        



