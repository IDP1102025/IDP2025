from machine import Pin, PWM
import time
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from pid.pid import LineFollower
from sensors.sensor_manager import LineSensor, CrashSensor, UltraSound
from sensors.code_reader import CodeReader
from navigation.corners import CornerIdentification
from collections import deque
from time import time , sleep

class TestRobot :
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

        self.corner_identification = CornerIdentification(self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor)

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
        junction_status = self.corner_identification.find_turn()
        while junction_status != True: 
            self.dual_motors.move_forward(30, 30)
        if junction_status == True: 
            self.move(1, 30)
    
    def return_to_start(self):
        raise NotImplementedError
    
    def goto_node(self,target_node):
        raise NotImplementedError

    def depot(self):
        raise NotImplementedError
    
    def face_direction(self, desired_direction):
        net_turn = desired_direction - self.direction_facing
        
        if self.direction_facing != desired_direction:
            if abs(net_turn) == 2:
                streets_passed = 0
                while streets_passed < 2:
                    if self.inner_right_sensor.read_sensor() == 1 and self.inner_left_sensor.read_sensor() == 1:
                        streets_passed += 1
                    self.dual_motors.turn_right(30)
                self.dual_motors.stop()

            elif net_turn == 1 or net_turn == -3:
                streets_passed = 0
                while streets_passed < 1:
                    if self.inner_right_sensor.read_sensor() == 1 and self.inner_left_sensor.read_sensor() == 1:
                        streets_passed += 1
                    self.dual_motors.turn_right(30)
                self.dual_motors.stop()

            elif net_turn == -1 or net_turn == 3: 
                streets_passed = 0
                while streets_passed < 1:
                    if self.inner_right_sensor.read_sensor() == 1 and self.inner_left_sensor.read_sensor() == 1:
                        streets_passed += 1
                    self.dual_motors.turn_left(30)
                self.dual_motors.stop()
    
    def move(self, number_of_junctions, base_speed):
        junctions_passed = 0
        self.dual_motors.move_forward(base_speed, base_speed)
        while True: 
            while junctions_passed < number_of_junctions:
                left_speed, right_speed, direction = self.line_follower.follow_line(0, base_speed)
                self.dual_motors.move_forward(left_speed, right_speed)
                if self.corner_identification.find_turn() == True: 
                    junctions_passed += 1
            self.dual_motors.stop()

    def reverse(self):
        raise NotImplementedError
    
    def stop(self):
        raise NotImplementedError

