from machine import Pin, time_pulse_us

import time

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
    def __init__(self, trigger_pin, echo_pin):
        self.trigger = Pin(trigger_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN, Pin.PULL_DOWN)

    def detect_distance(self):
        self.trigger.low()
        time.sleep_us(2)
        self.trigger.high()
        time.sleep_us(2)
        self.trigger.low()

        duration = time_pulse_us(self.echo, 1, 30000)

        distance = (duration / 2) / 29.1

        return distance






    




