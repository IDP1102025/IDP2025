from machine import Pin
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from line_follow.follow_line import NotPidFollower
from sensors.sensor_manager import LineSensor, UltraSound
from sensors.code_reader import CodeReader
from navigation.corners import CornerIdentification
from time import time, sleep
from navigation.navigation import Navigation


class Robot:
    def __init__(self):
        """
        Inititalise button, LED, all sensors, motors, linefollower class and navigation class

        Robot variables + states:
        - Direction facing (Front, right, back, left) (represented as 1,2,3,4)
        - Boxes in each depot (4,3,2,1,0)
        - Current node
        """
        # Initialise LED and button
        self.led = Pin(16, Pin.OUT)
        self.button = Pin(22, Pin.IN, Pin.PULL_DOWN)

        # Initialising Motors and actuators
        self.dual_motors = DualMotor(
            left_dir_pin=4, left_pwm_pin=5, right_dir_pin=7, right_pwm_pin=6
        )
        self.linear_actuator = LinearActuatorDriver(
            0, 1
        )  # add pins for linear actuator

        # Init Sensors
        self.ultrasonic_sensor = UltraSound(26)

        self.qr = CodeReader(15, 14)
        self.outer_left_sensor = LineSensor(8)
        self.inner_left_sensor = LineSensor(9)
        self.inner_right_sensor = LineSensor(10)
        self.outer_right_sensor = LineSensor(11)

        self.corner_identification = CornerIdentification(
            self.outer_left_sensor,
            self.inner_left_sensor,
            self.inner_right_sensor,
            self.outer_right_sensor,
        )

        # Init line follower
        self.line_follower = NotPidFollower(
            self.outer_left_sensor,
            self.inner_left_sensor,
            self.inner_right_sensor,
            self.outer_right_sensor,
        )

        # Init navigaton
        self.navigation = Navigation()

        # Robot variables and states
        self.direction_facing = 1
        self.current_node = self.navigation.graph.get_node(
            "Start"
        )  # Inititalise at the start node
        self.boxes_in_depot = {1: 4, 2: 4}
        self.robot_node_path = []  # list of node objects
        self.robot_direction_path = (
            []
        )  # list of directions of travel and number of junctions to travel in that direction: (1,2) corresponds to (North, 2 junctions)

        # Inititalise robot speeds
        self.right_speed, self.left_speed = 100, 1 - 0

    def robot_standby(self):
        """
        Waits for the button to be pressed before starting the robot
        """
        while True:
            button_state = self.button.value()
            if button_state == 1:
                break

        print("Button pressed! Starting the robot...")  # Confirm action

        self.start()

    def start(self):
        """'
        Move out of the start box and start the loop
        """
        # Check if the robot is at the start node
        self.dual_motors.move_forward(100, 100)
        sleep(1)
        self.led.value(1)

        # Continues moving forward until it reaches the first node
        self.move(1)

    def return_to_start(self):
        """
        Return automatically to the start node from the current node
        """

        self.goto_node(self.navigation.graph.get_node("Start Node"))
        self.face_direction(3)
        self.move(1)
        self.dual_motors.move_forward(80, 80)
        sleep(1.87)
        self.led.value(0)
        self.stop()

    def goto_node(self, target_node):
        """
        Navigate to a specific node from the current node
        Args:
            target_node (Node): Node object to navigate to
        """
        # Performs navigation and pathing to a specific node from the current node

        # Clear current path of node objects
        self.robot_node_path = []

        # Clear current direction path
        self.robot_direction_path = []

        # Get the path to the target node
        # Algo function returns compressed paths already
        print(self.current_node, target_node)
        distance_to_node, self.robot_node_path, self.robot_direction_path = (
            self.navigation.dijkstra_with_directions(self.current_node, target_node)
        )

        # Execute path
        self.execute_pathing()

        #          Execute action at current node, either depot or goal
        if self.current_node.name in ["Depot 1", "Depot 2"]:
            self.depot(int(self.current_node.name[-1]))
        elif self.current_node.name in ["A", "B", "C", "D"]:
            self.target_node(self.current_node.name)

    def execute_pathing(self):
        # Execute the path to the target node usings the robot's inbuilt queue
        print(list(self.robot_node_path))
        print(list(self.robot_direction_path))
        self.current_node = (
            self.robot_node_path.pop()
        )  # first node is the node we are starting at so pop it off first
        while self.robot_direction_path:  # While direction queue is not empty
            next_node = self.robot_node_path.pop()
            next_direction, number_of_junctions_to_pass = (
                self.robot_direction_path.pop()
            )

            # Face the direction of the next node
            self.face_direction(next_direction)

            # Move to the next node, travelling for n junctions
            self.move(number_of_junctions_to_pass)
            self.current_node = (
                next_node  # Once new node is reached, update current node
            )
        print(self.current_node.name)
        if len(self.robot_direction_path) != 0:
            print("Error in pathing")
            print(self.robot_direction_path)

    def face_direction(self, desired_direction, depot_number=1):
        """
        Turn the robot to face a specific direction
        Args:
            desired_direction (int): Direction to face (1,2,3,4) = (Front, Right, Back, Left)
            depot_number (int): Depot number (1,2) to determine which direction to turn to avoid colliding into walls
        """
        # Add movement offset to account for wheel and sensor offset

        net_turn = desired_direction - self.direction_facing

        if self.direction_facing != desired_direction:
            self.dual_motors.move_forward(50, 50)

            if abs(net_turn) == 2:  # 180 degree turn
                if depot_number == 1:
                    self.dual_motors.u_turn(80)
                    sleep(1)
                    while True:
                        current_pattern = self.line_follower.scan_state_patterns()
                        # If the desired pattern is detected
                        if current_pattern == [0, 1, 1, 0]:
                            #                             print(f"[INFO] Detected pattern {current_pattern}. Stopping turn.")
                            self.dual_motors.stop()
                            break
                else:
                    self.dual_motors.left_u_turn(80)
                    sleep(1)
                    while True:
                        current_pattern = self.line_follower.scan_state_patterns()
                        # If the desired pattern is detected
                        if current_pattern == [0, 1, 1, 0]:
                            #                             print(f"[INFO] Detected pattern {current_pattern}. Stopping turn.")
                            self.dual_motors.stop()
                            break

            elif net_turn == 1 or net_turn == -3:  # 90 degree turn right
                sleep(0.2)
                self.dual_motors.turn_right(100)
                sleep(1)
                while True:
                    self.line_follower.scan_state_patterns()
                    if (
                        self.inner_right_sensor.read_sensor() == 1
                        and self.inner_left_sensor.read_sensor() == 1
                    ):
                        #                         print("[INFO] Detected inner sensors = 1. Stopping turn.")
                        self.dual_motors.stop()
                        break

            elif net_turn == -1 or net_turn == 3:  # 90 degree turn left
                sleep(0.2)
                self.dual_motors.turn_left(100)
                sleep(1)
                while True:
                    self.line_follower.scan_state_patterns()
                    if (
                        self.inner_right_sensor.read_sensor() == 1
                        and self.inner_left_sensor.read_sensor() == 1
                    ):
                        #                         print("[INFO] Detected inner sensors = 1. Stopping turn.")
                        self.dual_motors.stop()
                        break

            # set new direction
            self.direction_facing = desired_direction

    def move(self, number_of_junctions, time_to_run=1):
        """
        Move the robot forward for a certain number of junctions, or for a certain amount of time

        Args:
            number_of_junctions (int): Number of junctions to move forward
            time_to_run (float): Time to run in seconds (if the number_of_junctions = 0)
        """
        detected_junctions = 0
        junction_active = False  # Indicates a junction is currently being counted
        no_junction_counter = 0  # Counts consecutive cycles with no junction
        required_false_cycles = (
            3  # Number of consecutive False readings to reset the flag
        )

        if number_of_junctions == 0:
            start_time = time()
            while (time() - start_time) < time_to_run:
                # 1) Get the next step's speeds from your line follower
                self.left_speed, self.right_speed = self.line_follower.follow_the_line(
                    self.left_speed, self.right_speed
                )

                # 2) Drive motors with these speeds
                self.dual_motors.move_forward(self.left_speed, self.right_speed)

            self.dual_motors.stop()
            # Done

        while detected_junctions < number_of_junctions:
            # 1) Get the next step's speeds from your line follower
            self.left_speed, self.right_speed = self.line_follower.follow_the_line(
                self.left_speed, self.right_speed
            )

            # 2) Drive motors with these speeds
            self.dual_motors.move_forward(self.left_speed, self.right_speed)

            # 3) Check for junction detection
            junction_detected = self.corner_identification.find_turn(
                self.line_follower.state_pattern
            )
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

        # print(self.line_follower.state_pattern)
        #         print(f"Final junction count: {detected_junctions}")
        self.dual_motors.stop()

    def reverse(self, seconds):
        """
        Reverse the robot for a certain amount of time

        Args:
            seconds (float): Time to reverse in seconds
        """
        self.dual_motors.move_backward(100)
        sleep(seconds)
        self.dual_motors.stop()

    def stop(self):
        self.dual_motors.stop()

    def depot(self, depot_number):
        """
        Block to perform at depot node:
        - Scan for QR code
        - Parse next node from QR code data
        - Pick up box
        - Move to goal node
        """
        backup_factor = 5 - self.boxes_in_depot[depot_number]
        # Face the direction of the depot (south)
        self.face_direction(3)

        self.reverse(0.6)
        self.stop()
        forwards = True

        code = self.qr.poll_for_code(1.5)
        while code is None:
            if forwards:
                self.move(0, 0.5)
                forwards = False
                code = self.qr.poll_for_code(1.5)
            else:
                self.reverse(0.5)
                forwards = True
                code = self.qr.poll_for_code(1.5)

        print(code)
        qr_message = bytes(code).decode("utf-8")
        print(qr_message)
        next_node = self.navigation.graph.get_node(qr_message[0])
        print(next_node)

        #         next_node_name = choice(['A','B','C','D'])
        #         next_node = self.navigation.graph.get_node("A")

        if next_node is None:
            print("no code")

        else:
            reverse_time_start = time()  # Save time taken to reverse

            self.move_ultrasound(
                5.5
            )  # Move to the box, following the line until the reader detects 5.5cm distance
            self.linear_actuator.retract(3)  # Pick up the box

            self.boxes_in_depot[
                depot_number
            ] -= 1  # Decrement the number of boxes in the depot

            self.reverse(time() - reverse_time_start)  # Reverse back to the depot

            self.face_direction(1, depot_number)

            self.move(1)

            self.goto_node(next_node)

    def target_node(self, name):
        """
        Block to perform at goal node:
        - Drop box
        - Move to next depot or return to start
        """
        destination_dic = {"A": 1, "B": 3, "C": 4, "D": 3}
        if name in destination_dic:
            # Face depot deposit zone
            self.face_direction(destination_dic[name])
            start_time = time()

            # Fove to the edge of the depot
            self.move(1)
            elapsed_time = time() - start_time

            # move slightly further into drop zone
            self.dual_motors.move_forward(50, 50)
            sleep(0.1)
            self.dual_motors.stop()

            # drop the package
            self.linear_actuator.extend(3)
            # reverse slightly
            self.dual_motors.move_backward(50)

            # Reverse the time taken to move to the edge of the drop zone
            self.reverse(elapsed_time)
            # go to next location
            if self.boxes_in_depot[1] != 0:
                next_directions = {"A": 2, "B": 2, "C": 3, "D": 2}
                self.face_direction_reverse(next_directions[name])
                self.move(0, 0.2)
                self.goto_node(self.navigation.graph.get_node("Depot 1"))
            elif self.boxes_in_depot[2] != 0:
                next_directions = {"A": 4, "B": 4, "C": 3, "D": 4}
                self.face_direction_reverse(next_directions[name])
                self.move(0, 0.2)
                self.goto_node(self.navigation.graph.get_node("Depot 2"))
            else:
                next_directions = {"A": 2, "B": 2, "C": 3, "D": 2}
                self.face_direction_reverse(next_directions[name])
                self.move(0, 0.2)
                self.return_to_start()

    def face_direction_reverse(self, desired_direction):
        """
        Turn the robot to face a specific direction in the reverse direction
        Args:
            desired_direction (int): Direction to face (1,2,3,4) = (Front, Right, Back, Left)

        """
        # Add movement offset to account for wheel and sensor offset

        net_turn = desired_direction - self.direction_facing

        if net_turn == 1 or net_turn == -3:  # 90 degree turn right
            sleep(0.2)
            self.dual_motors.turn_right_reverse(100)
            sleep(1)
            while True:
                self.line_follower.scan_state_patterns()
                if (
                    self.inner_right_sensor.read_sensor() == 1
                    and self.inner_left_sensor.read_sensor() == 1
                ):
                    #                         print("[INFO] Detected inner sensors = 1. Stopping turn.")
                    self.dual_motors.stop()
                    break

        elif net_turn == -1 or net_turn == 3:  # 90 degree turn left
            sleep(0.2)
            self.dual_motors.turn_left_reverse(100)
            sleep(1)
            while True:
                self.line_follower.scan_state_patterns()
                if (
                    self.inner_right_sensor.read_sensor() == 1
                    and self.inner_left_sensor.read_sensor() == 1
                ):
                    # print("[INFO] Detected inner sensors = 1. Stopping turn.")
                    self.dual_motors.stop()
                    break

        # set new direction
        self.direction_facing = desired_direction


def move_ultrasound(self, distance_to_stop):
    """
    Move the robot forward until the ultrasound sensor detects a certain distance

    Args:
        distance_to_stop (float): Distance to stop moving forward
    """
    while self.ultrasonic_sensor.detect_distance() > distance_to_stop:
        # 1) Get the next step's speeds from your line follower
        self.left_speed, self.right_speed = self.line_follower.follow_the_line(
            self.left_speed, self.right_speed
        )

        # 2) Drive motors with these speeds
        self.dual_motors.move_forward(self.left_speed, self.right_speed)

    # if within distance, exit the loop
    self.dual_motors.stop()
