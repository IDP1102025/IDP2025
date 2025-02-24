from machine import Pin, ADC
from time import sleep


class LineSensor:
    def __init__(self, pin):
        # Initialise Sensor
        self.sensor_pin = pin
        self.sensor = Pin(self.sensor_pin, Pin.IN, Pin.PULL_DOWN)

    def read_sensor(self):
        return self.sensor.value()


# Distance Sensor
class UltraSound:
    def __init__(self, pin=26):
        # Initiatlise Sensor
        self.pin = ADC(pin)
        self.MAX_RANGE = 520
        self.adc_solution = 65535.0

    def detect_distance(self):
        # Convert returned value to distance
        sensitivity_t = self.pin.read_u16()
        dist_t = sensitivity_t * self.MAX_RANGE / self.adc_solution
        sleep(0.1)
        return dist_t
