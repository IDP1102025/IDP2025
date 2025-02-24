class NotPidFollower:
    def __init__(
        self,
        outer_left_sensor,
        inner_left_sensor,
        inner_right_sensor,
        outer_right_sensor,
    ):
        # Pin Assignment
        self.outer_left_sensor = outer_left_sensor
        self.inner_left_sensor = inner_left_sensor
        self.inner_right_sensor = inner_right_sensor
        self.outer_right_sensor = outer_right_sensor
        self.state_pattern = []
        self.error_loop = 0

    def follow_the_line(self, current_left_speed, current_right_speed):
        '''
        This function is used to follow the line using the line sensors

        Args:
            current_left_speed: Current speed of the left motor
            current_right_speed: Current speed of the right motor
        Returns:
            current_left_speed: Updated speed of the left motor
            current_right_speed: Updated speed of the right motor
        '''
        # 1) Read sensors
        outer_left_detect = self.outer_left_sensor.read_sensor()  # 0 or 1
        inner_left_detect = self.inner_left_sensor.read_sensor()
        inner_right_detect = self.inner_right_sensor.read_sensor()
        outer_right_detect = self.outer_right_sensor.read_sensor()

        # 2) Determine sensor pattern
        self.state_pattern = [
            outer_left_detect,
            inner_left_detect,
            inner_right_detect,
            outer_right_detect,
        ]

        if self.state_pattern == [0, 1, 1, 0]:
            self.error_loop += 1  # Add 1 to the straight counter
            # Set speeds to match
            if self.error_loop >= 15:
                current_left_speed = 100
                current_right_speed = 100
            else:
                current_left_speed = 100
                current_right_speed = 100
        else:
            self.error_loop = 0
            if self.state_pattern == [0, 0, 1, 0]:  # slightly left
                current_left_speed = 100
                current_right_speed = 85
            elif self.state_pattern == [0, 1, 0, 0]:  # slightly right
                current_left_speed = 85
                current_right_speed = 100

        return current_left_speed, current_right_speed

    def scan_state_patterns(self):
        '''
        Returns the current state of the IR sensors as a list
        '''
        # 1) Read sensors
        outer_left_detect = self.outer_left_sensor.read_sensor()  # 0 or 1
        inner_left_detect = self.inner_left_sensor.read_sensor()
        inner_right_detect = self.inner_right_sensor.read_sensor()
        outer_right_detect = self.outer_right_sensor.read_sensor()

        # 2) Determine sensor pattern
        self.state_pattern = [
            outer_left_detect,
            inner_left_detect,
            inner_right_detect,
            outer_right_detect,
        ]

        return self.state_pattern