from machine import Pin, PWM
from time import sleep


class LinearActuatorDriver:
    def __init__(self, dir_pin, pwn_pin):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwn_pin))
        self.pwm.freq(50)
        self.pwm.duty_u16(0)

    def stop(self):
        self.pwm.duty_u16(0)

    def extend(self, time_length):
        self.dir.value(0)
        self.pwm.duty_u16(65535)  # Full speed extension
        sleep(time_length)  # Extend for stipulated time
        self.pwm.duty_u16(0)

    def retract(self, time_length):
        self.dir.value(1)
        self.pwm.duty_u16(65535)  # Full speed retraction
        sleep(time_length)  # Extend for stipulated time
        self.pwm.duty_u16(0)


# lin = LinearActuatorDriver(0, 1)
#
# lin.extend(1)
# lin.stop()
