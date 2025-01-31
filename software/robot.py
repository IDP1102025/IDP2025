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
from navigation.navigation import Navigation
from navigation.graph import Graph
from navigation.node import Node

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

        self.corner_identification = CornerIdentification(self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor)

        # Init line follower
        self.line_follower = LineFollower(None, self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor,"""add kp,ki ,kd""")

        # Init navigaton
        self.navigation = Navigation()

        # Robot variables and states
        self.direction_facing = 1
        self.current_node = self.navigation.graph.get_node("Start") # Inititalise at the start node
        self.time_elapsed = 0
        self.base_speed = 30
        self.boxes_in_depot = {"Depot 1":4, "Depot 2":4}
        self.robot_node_path = deque([]) # Deque of node objects
        self.robot_direction_path = deque([]) # Deque of directions of travel and number of junctions to travel in that direction
        

        # Robot states
        self._current_task = "idle"  # Use an underscore to define a private variable

    @property
    def current_task(self):
        return self._current_task

    @current_task.setter
    def current_task(self, task):
        """Set the robot's current task and update the LED accordingly."""
        self._current_task = task

        # LED ON if task is not idle, else OFF
        if task != "idle":
            self.led.value(1)  # Turn LED ON
        else:
            self.led.value(0)  # Turn LED OFF


    def robot_standby(self):
        while True:
            if self.button.value() == 1:
                self.start()
                break
    
    def start(self):
        junction_status = self.corner_identification.find_turn()
        timeout = time() + 5  # 5-second timeout to prevent infinite loop
        while junction_status != True and time() < timeout: 
            self.dual_motors.move_forward(30, 30) # Start moving forward to find the first junction

        if junction_status: # Once junction is found, proceed towards the start node
            self.move(1, 30)
        else:
            self.dual_motors.stop()

    def return_to_start(self):
        '''
        Return automatically to the start node from the current node
        '''
        
        self.goto_node(self.navigation.graph.get_node("Start"))

    def goto_node(self,target_node):
        '''
        Args:
            target_node (Node): Node object to navigate to
        '''
        # Performs navigation and pathing to a specific node from the current node
        # Update status
        self.current_task = "moving"
    
        # Clear current path of node objects
        self.robot_node_path.clear()

        # Clear current direction path
        self.robot_direction_path.clear()

        # Get the path to the target node
        # Algo function returns compressed paths already
        distance_to_node, self.robot_node_path, self.robot_direction_path = self.navigation.dijkstra_with_directions(self.current_node, target_node) 

        # Execute path
        self.execute_pathing()

        # Execute action at current node, either depot or goal
        if self.current_node.node_type == "depot":
            self.depot()
        
        elif self.current_node.node_type == "goal":
            self.target_node()
        
        # Update status back to idle
        self.current_task = "idle"
    
    def execute_pathing(self):
        # Execute the path to the target node usings the robot's inbuilt queue
        self.current_node = self.robot_node_path.popleft() # first node is the node we are starting at so pop it off first
        while self.robot_direction_path: # While direction queue is not empty
            next_node = self.robot_node_path.popleft()
            next_direction , number_of_junctions_to_pass = self.robot_direction_path.popleft()

            # Face the direction of the next node
            self.face_direction(next_direction)
            
            # Move to the next node, travelling for n junctions
            self.move(number_of_junctions_to_pass,base_speed=self.base_speed)
            self.current_node = next_node # Once new node is reached, update current node
        
    
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

            # set new direction
            self.direction_facing = desired_direction
        
    
    def move(self, number_of_junctions, base_speed):
        detected_junctions = 0
        while detected_junctions < number_of_junctions:
            left_speed, right_speed, direction = self.line_follower.follow_line(0, base_speed)
            self.dual_motors.move_forward(left_speed, right_speed)

            if self.corner_identification.find_turn():
                sleep(0.1)  # Debounce delay
                if self.corner_identification.find_turn():  # Confirm junction
                    detected_junctions += 1

        self.stop()

    def reverse(self):
        self.dual_motors.move_backward(20)
    
    def stop(self):
        self.dual_motors.stop()

    # Block to perform at depot nodes
    def depot(self):
        # Face the direction of the depot (south)
        self.face_direction(3)
        self.qr = CodeReader()
        # TODO: Implement depot logic
        while self.qr.poll_for_code(1) == None: 
            self.dual_motors.move_forward(10, 10)
        self.stop()
        if type(self.qr.poll_for_code(1)) == str:
            # destination = letter from message returned by poll_for_code
            # obtain route to the destination
            # pick up box
            # move backward to junction
            raise NotImplementedError

    
    # Block to perform at goal node
    def target_node(self):
        # face direction (hard code these in for each destination)
        # move forward (set distance)
        # drop box
        # back up to return to junction
        raise NotImplementedError
    

    def begin_test(self):
        # command to test the robot by moving to a specific node and then returning to the start
        raise NotImplementedError
    
    