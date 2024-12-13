# -*- coding: utf-8 -*-
"""
Created on Mon Dec  2 13:37:22 2024

@author: NJ
"""

from pyb import Pin
import time

class LS:
    def __init__ (self):
        self.ctrl_even = Pin(Pin.cpu.C8, Pin.OUT)  # Control even-numbered emitters
        self.ctrl_odd = Pin(Pin.cpu.C4, Pin.OUT)   # Control odd-numbered emitters
        self.ls_1 = Pin(Pin.cpu.C12, Pin.OUT)  # Pin connected to sensor output
        self.ls_2 = Pin(Pin.cpu.H1, Pin.OUT)  # Pin connected to sensor output
        self.ls_3 = Pin(Pin.cpu.C11, Pin.OUT)  # Pin connected to sensor output
        self.ls_4 = Pin(Pin.cpu.H0, Pin.OUT)  # Pin connected to sensor output
        self.ls_5 = Pin(Pin.cpu.B1, Pin.OUT)  # Pin connected to sensor output
        self.ls_6 = Pin(Pin.cpu.B15, Pin.OUT)  # Pin connected to sensor output
        self.ls_7 = Pin(Pin.cpu.B12, Pin.OUT)  # Pin connected to sensor output
        self.ls_8 = Pin(Pin.cpu.B11, Pin.OUT)  # Pin connected to sensor output
    def measure_reflectivity(self,sensor_pin):
        # Turn on IR LEDs (optional)
        self.ctrl_even.value(0)  # Active low - turn on even-numbered emitters
        self.ctrl_odd.value(0)   # Active low - turn on odd-numbered emitters

        # Set sensor pin to output and drive it high to charge the capacitor
        sensor_pin.init(Pin.OUT)
        sensor_pin.value(1)  # Drive the pin high

        # Allow time for the sensor output to rise (at least 10 µs)
        time.sleep_us(10)

        # Set sensor pin to input (high impedance)
        sensor_pin.init(Pin.IN)

        # Measure the time for the voltage to decay by waiting for the pin to go low
        start_time = time.ticks_us()  # Record the start time in microseconds

        while sensor_pin.value() == 1:
            if time.ticks_diff(time.ticks_us(), start_time) > 10000:  # Timeout after 10 ms
                decay_time = -1  # Indicate timeout
                break
        else:
            decay_time = time.ticks_diff(time.ticks_us(), start_time)

        # Turn off IR LEDs (optional)
        self.ctrl_even.value(1)  # Set to high to turn off even-numbered emitters
        self.ctrl_odd.value(1)   # Set to high to turn off odd-numbered emitters

        # Return the measured decay time (in microseconds)
        return decay_time