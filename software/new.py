from machine import Pin, PWM
from controllers.motor_driver import DualMotor
from controllers.linear_actuator_driver import LinearActuatorDriver
from pid.pid import LineFollower
from sensors.sensor_manager import LineSensor, CrashSensor, UltraSound
from sensors.code_reader import CodeReader
from navigation.corners import CornerIdentification
from collections import deque
from time import time , sleep
from navigation.navigation import Navigation
from navigation.graph import Graph
from navigation.node import Node
from robot import Robot

def test_motors():
    retard = Robot()
    
