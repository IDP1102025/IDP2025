from machine import Pin, ADC
from time import sleep

#Initialising Sensors

#Tracker Sensors 

class LineSensor:
    def __init__(self, pin):
        self.sensor_pin = pin
        self.sensor = Pin(self.sensor_pin, Pin.IN, Pin.PULL_DOWN)

    def read_sensor(self):
        return self.sensor.value()
    

#Crash Sensor
    
#Crash 
class CrashSensor:
    def __init__(self, pin):
        self.sensor_pin = pin
        self.sensor = Pin(self.sensor_pin, Pin.IN, Pin.PULL_UP)

    def read_sensor(self):
        return self.sensor.value()


#Distance Sensor
class UltraSound: 
    #trigger pin is the emitter of the wave, while the echo pin recieves it
    def __init__(self, pin=26):
        self.pin = ADC(pin)
        self.MAX_RANGE = 520
        self.adc_solution = 65535.0

    def detect_distance(self):
        sensitivity_t = self.pin.read_u16()
        dist_t = (sensitivity_t * self.MAX_RANGE / self.adc_solution)
        sleep(0.1)
        return dist_t


# ultrasound = UltraSound()
# 
# while True:
#     print(ultrasound.detect_distance())
# 
# 
#     






