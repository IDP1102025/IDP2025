from machine import Pin, PWM
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from line_follow.follow_line import NotPidFollower
from sensors.sensor_manager import LineSensor, CrashSensor, UltraSound
from sensors.code_reader import CodeReader
from navigation.corners import CornerIdentification
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
        self.led = Pin(16, Pin.OUT)
        self.button = Pin(22, Pin.IN, Pin.PULL_DOWN)
        
        
        # Initialising Motors and actuators
        self.dual_motors = DualMotor(left_dir_pin=4, left_pwm_pin=5, right_dir_pin=7, right_pwm_pin=6)
        self.linear_actuator = LinearActuatorDriver(0,1) # add pins for linear actuator

        # Init Sensors
        self.ultrasonic_sensor = UltraSound(26)

        self.qr = CodeReader(15,14)
        self.outer_left_sensor = LineSensor(8)
        self.inner_left_sensor = LineSensor(9)
        self.inner_right_sensor = LineSensor(10)
        self.outer_right_sensor = LineSensor(11)

        self.corner_identification = CornerIdentification(self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor)

        # Init line follower
        self.line_follower = NotPidFollower(self.outer_left_sensor, self.inner_left_sensor, self.inner_right_sensor, self.outer_right_sensor)

        # Init navigaton
        self.navigation = Navigation()

        # Robot variables and states
        self.direction_facing = 1
        self.current_node = self.navigation.graph.get_node("Start") # Inititalise at the start node
        self.boxes_in_depot = {1:4, 2:4}
        self.robot_node_path = []  #list of node objects
        self.robot_direction_path = [] # list of directions of travel and number of junctions to travel in that direction: (1,2) corresponds to (front, 2 junctions)
        
        # Inititalise robot speeds
        self.right_speed , self.left_speed = 70 , 70

        # Robot states
        self._current_task = "idle"  # Use an underscore to define a private variable

    @property
    def current_task(self):
        return self._current_task

    # Sets the status of the LED if robot is not idle
    @current_task.setter
    def current_task(self, task):
        self._current_task = task

    # Starts the robot if the button is pressed - calls the self.start() code
    def robot_standby(self):
        while True:
            button_state = self.button.value()
            if button_state == 1:
                break
        print("Button pressed! Starting the robot...")  # Confirm action
        self.start()
    
    def start(self):
        # Check if the robot is at the start node
        self.dual_motors.move_forward(100, 100)
        while True:
            current_pattern = self.line_follower.scan_state_patterns()
            if current_pattern == [1,1,1,1]:
                self._current_task = "moving"
                self.led.value(1)
                break
        # Continues moving forward until it reaches the first node
        sleep(1)
        self.move(1)

    def return_to_start(self):
        '''
        Return automatically to the start node from the current node
        '''
        
        self.goto_node(self.navigation.graph.get_node("Start Node"))
        self.face_direction(3)
        self.move(1)
        self.dual_motors.move_forward(80, 80)
        sleep(1.87)
        self.led.value(0)
        self.stop()
        

    def goto_node(self,target_node):
        '''
        Args:
            target_node (Node): Node object to navigate to
        '''
        # Performs navigation and pathing to a specific node from the current node
    
        # Clear current path of node objects
        self.robot_node_path  = []

        # Clear current direction path
        self.robot_direction_path = []

        # Get the path to the target node
        # Algo function returns compressed paths already
        print(self.current_node, target_node)
        distance_to_node, self.robot_node_path, self.robot_direction_path = self.navigation.dijkstra_with_directions(self.current_node, target_node) 

        # Execute path
        self.execute_pathing()

        # Execute action at current node, either depot or goal
#         if self.current_node.node_type == "depot":
#             self.depot()
#         `elif self.current_node.node_type == "goal":
#             self.target_node()
        
    
    def execute_pathing(self):
        # Execute the path to the target node usings the robot's inbuilt queue
        print(list(self.robot_node_path))
        print(list(self.robot_direction_path))
        self.current_node = self.robot_node_path.pop() # first node is the node we are starting at so pop it off first
        while self.robot_direction_path: # While direction queue is not empty
            next_node = self.robot_node_path.pop()
            next_direction , number_of_junctions_to_pass = self.robot_direction_path.pop()

            # Face the direction of the next node
            self.face_direction(next_direction)
            
            # Move to the next node, travelling for n junctions
            self.move(number_of_junctions_to_pass)
            self.current_node = next_node # Once new node is reached, update current node
        print(self.current_node.name)
        if len(self.robot_direction_path) != 0:
            print("Error in pathing")
            print(self.robot_direction_path)
        
    
    def face_direction(self, desired_direction,depot_number=1):
        '''
        Turn the robot to face a specific direction
        Args:
            desired_direction (int): Direction to face (1,2,3,4) = (Front, Right, Back, Left)

        '''
        # Add movement offset to account for wheel and sensor offset
        
        net_turn = desired_direction - self.direction_facing
        
        if self.direction_facing != desired_direction:
            self.dual_motors.move_forward(50,50)
            
            
            if abs(net_turn) == 2: # 180 degree turn
                if depot_number ==1:
                    self.dual_motors.u_turn(80)
                    sleep(1)
                    while True:
                        current_pattern = self.line_follower.scan_state_patterns()
                        # If the desired pattern is detected
                        if current_pattern == [0, 1, 1, 0]:
                            print(f"[INFO] Detected pattern {current_pattern}. Stopping turn.")
                            self.dual_motors.stop()
                            break
                else:
                    self.dual_motors.left_u_turn(80)
                    sleep(1)
                    while True:
                        current_pattern = self.line_follower.scan_state_patterns()
                        # If the desired pattern is detected
                        if current_pattern == [0, 1, 1, 0]:
                            print(f"[INFO] Detected pattern {current_pattern}. Stopping turn.")
                            self.dual_motors.stop()
                            break

            elif net_turn == 1 or net_turn == -3: # 90 degree turn right
                sleep(0.2)
                self.dual_motors.turn_right(100)
                sleep(1)
                while True:
                    self.line_follower.scan_state_patterns()
                    if self.inner_right_sensor.read_sensor() == 1 and self.inner_left_sensor.read_sensor() == 1:
                        print("[INFO] Detected inner sensors = 1. Stopping turn.")
                        self.dual_motors.stop()
                        break
                    
            elif net_turn == -1 or net_turn == 3: # 90 degree turn left
                sleep(0.2)
                self.dual_motors.turn_left(100)
                sleep(1)
                while True:
                    self.line_follower.scan_state_patterns()
                    if self.inner_right_sensor.read_sensor() == 1 and self.inner_left_sensor.read_sensor() == 1:
                        print("[INFO] Detected inner sensors = 1. Stopping turn.")
                        self.dual_motors.stop()
                        break
                    
            # set new direction
            self.direction_facing = desired_direction
        else:
            print("Already facing the desired direction.")
        
    def move(self, number_of_junctions, time_to_run = 1):
        self.current_task = "moving"
        detected_junctions = 0
        junction_active = False  # Indicates a junction is currently being counted
        no_junction_counter = 0  # Counts consecutive cycles with no junction
        required_false_cycles = 2  # Number of consecutive False readings to reset the flag

        if number_of_junctions == 0:
            start_time = time()
            while (time() - start_time) < time_to_run:
                # 1) Get the next step's speeds from your line follower
                self.left_speed, self.right_speed = self.line_follower.follow_the_line(self.left_speed, self.right_speed)
                
                # 2) Drive motors with these speeds
                self.dual_motors.move_forward(self.left_speed, self.right_speed)
            
            self.dual_motors.stop()
            # Done

        while detected_junctions < number_of_junctions:
#             print(f"[DEBUG] Left Speed: {self.left_speed}, Right Speed: {self.right_speed}")
#             print(f"Current junction count: {detected_junctions}")
            print(self.line_follower.state_pattern)
            # 1) Get the next step's speeds from your line follower
            self.left_speed, self.right_speed = self.line_follower.follow_the_line(self.left_speed, self.right_speed)
            
            # 2) Drive motors with these speeds
            self.dual_motors.move_forward(self.left_speed, self.right_speed)
            
            # 3) Check for junction detection
            junction_detected = self.corner_identification.find_turn(self.line_follower.state_pattern)
#             print(f"[DEBUG] Junction sensor reading: {junction_detected}")

            if junction_detected:
                if not junction_active:
                    detected_junctions += 1
                    junction_active = True  # Count this junction
                    no_junction_counter = 0  # Reset counter when a junction is detected
#                     print("Junction detected, count incremented.")
            else:
                no_junction_counter += 1
                # Reset the junction_active flag only after several consecutive False readings
                if no_junction_counter >= required_false_cycles:
#                     if junction_active:
#                         print("Junction passed, resetting flag.")
                    junction_active = False

        print(self.line_follower.state_pattern)
#         print(f"Final junction count: {detected_junctions}")
        self.dual_motors.stop()

    def reverse(self,seconds):
        self.dual_motors.move_backward(50)
        sleep(seconds)
        self.dual_motors.stop()
    
    def reverse_to_junction(self):
        #get off current node
        self.dual_motors.move_backward(100)
        sleep(2)
        
        # Reverse until a junction is detected
        while True:
            self.dual_motors.move_backward(100)
            if self.corner_identification.find_turn(self.line_follower.state_pattern):
                break
        # potential issue: the robot "finds a turn" because it strays from the line and outer sensors hit the line
        self.dual_motors.stop()

    
    def stop(self):
        self.dual_motors.stop()

    # Block to perform at depot nodes
    def depot(self, depot_number):
        # Face the direction of the depot (south)
        self.face_direction(3)
        self.move(0, time_to_run = 1)

        while self.ultrasonic_sensor.detect_distance() >= 15:
            self.move(0,0.5)
        
        qr_message = self.qr.poll_for_code(5)
        next_node = self.navigation.graph.get_node(qr_message[0])

        if next_node is None:
            print("YOU FUCKED UP BRO THE QR CODE DIDNT SCAN")
        
        else:
            while self.ultrasonic_sensor.detect_distance() < 200:
                self.move(0,0.5)
            
            self.linear_actuator.retract(5)
            
            self.boxes_in_depot[depot_number] -= 1

            self.face_direction(1, depot_number)

            self.move(1)

            self.goto_node(next_node)

        # TODO: Implement depot logic   
                
        self.stop()
        

    
    # Block to perform at goal node
    def target_node(self):
        destination_dic = {'A': 1, 'B': 3, 'C': 4, 'D': 3}
        if self.current_node.name in destination_dic:
            #face depot deposit zone
            self.face_direction(destination_dic[self.current_node.name])
            
            #move to the edge of the depot
            self.move(1)
            
            #move slightly further into drop zone
            self.dual_motors.move_forward(50,50)
            sleep(0.1)
            self.dual_motors.stop()

            #drop the package
            self.linear_actuator.extend(5)

            #reverse to the node
            self.reverse_to_junction()

            #go to next location
            if self.boxes_in_depot[1] != 0:
                self.goto_node(self.navigation.graph.get_node("Depot 1"))
            elif self.boxes_in_depot[2] != 0:
                self.goto_node(self.navigation.graph.get_node("Depot 2"))
            else:
                self.return_to_start()

