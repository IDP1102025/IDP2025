from machine import Pin

import time

#Initialising Sensors

#Tracker Sensors, 

class LineSensor:
    def __init__(self, pin):
        self.sensor_pin = pin
        self.sensor = Pin(self.sensor_pin, Pin.IN, Pin.PULL_DOWN)

    def read_sensor(self):
        return self.sensor.value()
    

class CrashSensor:



class DistanceTracker: 

    




