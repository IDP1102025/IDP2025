import time
import random

########################
# Mock Classes
########################
class MockLineSensor:
    """
    A mock sensor that returns a fixed or cycling value 
    (0=black, 1=white) from a list of states.
    """
    def __init__(self, name="Sensor", mock_states=None):
        # mock_states is a list of 0/1 for you to cycle through
        self.name = name
        if mock_states is None:
            mock_states = [0]*10  # default 10 black readings
        self.mock_states = mock_states
        self.index = 0

    def read_sensor(self):
        # Return the current state and move index forward
        val = self.mock_states[self.index]
        self.index = (self.index + 1) % len(self.mock_states)
        return val


class MockDualMotor:
    """
    A mock motor driver that simply prints out the speed commands 
    instead of controlling real motors.
    """
    def __init__(self):
        pass

    def move_forward(self, left_speed, right_speed):
        print(f"[MockDualMotor] move_forward called with L={left_speed:.2f}, R={right_speed:.2f}")

    def turn_left(self, speed):
        print(f"[MockDualMotor] turn_left called with Speed={speed}")

    def turn_right(self, speed):
        print(f"[MockDualMotor] turn_right called with Speed={speed}")

    def stop(self):
        print("[MockDualMotor] STOP")


class MockCornerIdentification:
    """
    A simple mock for corner detection. 
    If we see a certain pattern, we pretend we found a corner/junction.
    """
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor):
        self.outer_left_sensor = outer_left_sensor
        self.inner_left_sensor = inner_left_sensor
        self.inner_right_sensor = inner_right_sensor
        self.outer_right_sensor = outer_right_sensor

    def find_turn(self):
        # For example, if we see [1,1,1,1], pretend there's a junction
        s_pattern = [
            self.outer_left_sensor.read_sensor(),
            self.inner_left_sensor.read_sensor(),
            self.inner_right_sensor.read_sensor(),
            self.outer_right_sensor.read_sensor()
        ]
        return (s_pattern == [1,1,1,1])


########################
# PID / LineFollower (same as your code, 
# but slightly adapted to store or show corrections)
########################
class LineFollower:
    def __init__(self, outer_left_sensor, inner_left_sensor, inner_right_sensor, outer_right_sensor,
                 kp_high, kp_low, ki, kd):
        # Pin Assignment
        self.outer_left_sensor = outer_left_sensor
        self.inner_left_sensor = inner_left_sensor
        self.inner_right_sensor = inner_right_sensor
        self.outer_right_sensor = outer_right_sensor

        # PID Constants
        self.kp_low = kp_low
        self.kp_high = kp_high
        self.ki = ki
        self.kd = kd

        # PID Variables
        self.previous_error = 0
        self.integral = 0
        self.error_loop = 0  # used to detect repeated "straight" signals

    def calculate_pid(self, current, target, kp):
        error = current - target
        self.integral += error  # <- Uncomment to actually use integral
        derivative = error - self.previous_error
        self.previous_error = error

        correction = (kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return correction

    def adjust_motors(self, correction, current_left_speed, current_right_speed):
        new_left_speed  = current_left_speed - correction
        new_right_speed = current_right_speed + correction

        # Clamp speeds to [50, 100] for example
        new_left_speed  = max(50, min(100, new_left_speed))
        new_right_speed = max(50, min(100, new_right_speed))

        return new_left_speed, new_right_speed

    def follow_line_step(self, target_position, left_speed, right_speed):
        # 1) Read sensors
        outer_left_detect  = self.outer_left_sensor.read_sensor()
        inner_left_detect  = self.inner_left_sensor.read_sensor()
        inner_right_detect = self.inner_right_sensor.read_sensor()
        outer_right_detect = self.outer_right_sensor.read_sensor()

        # 2) Pattern
        state_pattern = [outer_left_detect,
                         inner_left_detect,
                         inner_right_detect,
                         outer_right_detect]

        kp = self.kp_low

        # 3) Determine error
        #    (You can tweak these conditions as needed)
        if state_pattern in ([0,1,1,0], [0,0,1,0], [0,1,0,0], [0,1,0,1], [1,0,1,0]):
            current_position = 0
            self.error_loop += 1
        else:
            self.error_loop = 0
            if state_pattern == [0,0,1,1]:
                current_position = -1
            elif state_pattern == [1,1,0,0]:
                current_position = 1
            elif state_pattern == [0,0,0,1]:
                current_position = -2
                kp = self.kp_high
            elif state_pattern == [1,0,0,0]:
                current_position = 2
                kp = self.kp_high
            else:
                # unrecognized pattern
                # fallback: treat it as 0 or do something safe
                current_position = 0

        # 4) Straight line speed-up example
        if self.error_loop == 10 and current_position == 0:
            new_left_speed, new_right_speed = (100, 100)
        else:
            # 5) Normal PID
            correction = self.calculate_pid(current_position, target_position, kp)
            new_left_speed, new_right_speed = self.adjust_motors(correction, left_speed, right_speed)

        # 6) Return
        # (Debug statement corrected to show new_right_speed)
        print(f"[LineFollower Debug] pattern={state_pattern}, err={current_position}, correction={correction:.2f}, speeds=({new_left_speed:.2f}, {new_right_speed:.2f})")
        return new_left_speed, new_right_speed


########################
# Mock Robot
########################
class MockRobot:
    """
    A simplified test version of your Robot class that does not require 
    physical hardware. We override the sensors, motors, and corner detection 
    with mocks. 
    """
    def __init__(self):
        # Here we define some sensor sequences to test your line follower.
        # Each sensor is 0 or 1. We'll give each sensor an entire sequence to cycle through.
        # E.g., outer_left = [0,0,0,1,1,1,0...], etc.
        self.outer_left_sensor  = MockLineSensor("OuterLeft",  [0,0,1,1,1,0,0,0,0,0])
        self.inner_left_sensor  = MockLineSensor("InnerLeft",  [1,1,1,0,1,1,1,1,0,0])
        self.inner_right_sensor = MockLineSensor("InnerRight", [1,1,0,0,0,1,0,1,1,0])
        self.outer_right_sensor = MockLineSensor("OuterRight", [0,0,0,0,0,0,1,0,1,1])

        self.corner_identification = MockCornerIdentification(
            self.outer_left_sensor, 
            self.inner_left_sensor, 
            self.inner_right_sensor, 
            self.outer_right_sensor
        )

        # Initialize line follower with some chosen PID constants:
        self.line_follower = LineFollower(
            outer_left_sensor  = self.outer_left_sensor,
            inner_left_sensor  = self.inner_left_sensor,
            inner_right_sensor = self.inner_right_sensor,
            outer_right_sensor = self.outer_right_sensor,
            kp_high = 1.5, 
            kp_low  = 1.0, 
            ki = 0.0, 
            kd = 0.0
        )

        # Mock motors
        self.dual_motors = MockDualMotor()

        # Speeds
        self.left_speed = 75
        self.right_speed = 75

    def move(self, number_of_junctions):
        """
        Simulates driving forward until we detect N junctions 
        using a PID line follower for steering.
        """
        print("\n[MockRobot] Starting move() with number_of_junctions =", number_of_junctions)
        detected_junctions = 0

        # We'll set a maximum iteration to avoid infinite loop in test
        max_iterations = 50
        iterations = 0

        while detected_junctions < number_of_junctions and iterations < max_iterations:
            iterations += 1
            # 1) Follow line (PID step)
            self.left_speed, self.right_speed = self.line_follower.follow_line_step(
                target_position=0, 
                left_speed=self.left_speed, 
                right_speed=self.right_speed
            )

            # 2) Move forward
            self.dual_motors.move_forward(self.left_speed, self.right_speed)

            # 3) Check junction
            # We'll read the sensors again for corner detection
            if self.corner_identification.find_turn():
                # We found a corner or junction
                detected_junctions += 1
                print(f"[MockRobot] Detected JUNCTION #{detected_junctions}")

            time.sleep(0.05)  # short delay to simulate loop

        self.dual_motors.stop()
        print(f"[MockRobot] move() done. Detected {detected_junctions} junction(s).")


########################
# MAIN TEST
########################
if __name__ == "__main__":
    # Create Mock Robot
    robot = MockRobot()

    # Call move(3) to see how speeds evolve
    robot.move(number_of_junctions=3)

    print("\nTest script complete.")
