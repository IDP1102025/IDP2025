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

        # Reference to motors (optional if you want direct motor methods)
        self.dual_motors = None

    def set_motor_controller(self, dual_motors):
        """ Assigns the motor controller to the LineFollower class. """
        self.dual_motors = dual_motors

    def calculate_pid(self, current, target, kp):
        """
        current: the 'current_position' from sensor reading
        target:  the desired line position (usually 0)
        kp:      proportional gain (can be high or low)
        """
        error =  current - target

        #self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error

        correction = (kp * error) #+ (self.ki * self.integral) + (self.kd * derivative) 

        return correction

    def adjust_motors(self, correction, base_speed):
        """
        Returns (left_speed, right_speed, direction_flag)
        direction_flag can be  0 = forward
                              +1 = turning right
                              -1 = turning left
        """
        left_speed  = base_speed - correction
        right_speed = base_speed + correction

        # Clamp the speeds to [20, 100] range
        left_speed  = max(20, min(100, left_speed))
        right_speed = max(20, min(100, right_speed))

        return (left_speed, right_speed)

    def follow_line_step(self, target_position=0, base_speed=50):
        """
        Reads sensor data and returns the motor speeds + direction
        in ONE iteration.  The calling loop decides how long to keep going.
        """

        # 1) Read sensors
        outer_left_detect  = self.outer_left_sensor.read_sensor()   # 0 or 1
        inner_left_detect  = self.inner_left_sensor.read_sensor()
        inner_right_detect = self.inner_right_sensor.read_sensor()
        outer_right_detect = self.outer_right_sensor.read_sensor()

        # 2) Determine sensor pattern
        state_pattern = [outer_left_detect,
                         inner_left_detect,
                         inner_right_detect,
                         outer_right_detect]

        current_position = 0
        kp = self.kp_low  # default gain

        # 3) Use pattern to figure out error
        #    (You can adapt these conditions as needed)
        if state_pattern in ([0,1,1,0], [0,0,1,0], [0,1,0,0], [0,1,0,1], [1,0,1,0]):
            current_position = 0
            self.error_loop += 1
        else:
            self.error_loop = 0
            if state_pattern == [0,0,1,1]:
                current_position = -1       # slightly left
            elif state_pattern == [1,1,0,0]:
                current_position = 1        # slightly right
            elif state_pattern == [0,0,0,1]:
                current_position = -2
                kp = self.kp_high          # bigger correction
            elif state_pattern == [1,0,0,0]:
                current_position = 2
                kp = self.kp_high
            #elif state_pattern == [0,0,0,0]:
                # All black => line lost
                # return special speeds or handle recovery
                #return self.recover_off_the_line(base_speed)

        # 4) If we've been going straight for a while, maybe speed up or do a special action
        if self.error_loop == 10 and current_position == 0:
            # Increase speed or do something if you want
            left_speed, right_speed = (100, 100)
        else:
            # 5) Normal PID
            correction = self.calculate_pid(current_position, target_position, kp)
            left_speed, right_speed= self.adjust_motors(correction, base_speed)
        print((left_speed, right_speed))
        return (left_speed, right_speed)

    def recover_off_the_line(self, base_speed):
        """
        If we detect the line is lost ([0,0,0,0] or any other scenario),
        we can either do a special turn or just return speeds to the caller.
        """
        print("[DEBUG] Robot lost the line! Attempting recovery...")

        

