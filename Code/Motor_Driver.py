# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 14:13:09 2024

@author: NJ
"""

from pyb import Pin, Timer

class Motor:
    # @brief: interface with step motor
    # @details: create functions to intialize, set the duty cycle, and enable/disabled the L6206 motor
    def __init__ (self, PWM_tim, PWM_pin, DIR_pin, EN_pin):
        # @brief: intialization of the motor
        # @details: create motor object with the timer, directional, and enabler pins
        #           sets up the timer as PWM timer with 2 channels to control motor direction and speed
        #           sets up other pins as on/off pins to enable motor and change direction
        # @param PWM_tim: timer that is used to control PWM duty cycle for motor speed
        # @param IN1_pin: on/off pin that causes motor to spin CCW
        # @param IN2_pin: on/off pin that causes motor to spin CW
        # @param EN_pin: on/off pin that enables/disabled motor
        self.PWM_pin = Pin(PWM_pin, mode=Pin.OUT_PP)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP)
        self.PWM_CH = PWM_tim.channel(1, mode=Timer.PWM, pin=self.PWM_pin, pulse_width_percent=0)
        self.DIR_pin.low()
        #self.PWM_CH2 = PWM_tim.channel(2, mode=Timer.PWM, pin=self.IN2_pin, pulse_width_percent=0)
        pass
    
    def set_duty (self,duty):
        # @brief: set PWM duty cycle for DC motor
        # @details: sets the timer duty cycle to be sent to the L6206 to a given level, which is duty
        #           changes which PWM channel to enable/disable due based off of sign of duty
        # @param duty: a signed number holding the duty cycle of the PWM signal sent to the L6206
        # self.PWM_CH.pulse_width_percent(duty)
        #self.PWM_CH.pulse_width_percent(duty)
        if duty>=0:
            self.PWM_CH.pulse_width_percent(duty)
            self.DIR_pin.low()
        if duty<0:
            self.PWM_CH.pulse_width_percent(-duty)
            self.DIR_pin.high()
        pass
    
    def enable (self):
        # @brief: enable the L6206 motor
        # @details: turns on the enable pin associated with one pin of the L6206 to enable the motor
        self.EN_pin.high()
        pass
    def disable (self):
        # @brief: disable the L6206 motor
        # @details: turns off the eanble pin associated with one pin of the L6206 to disable the motor
        self.EN_pin.low()
        pass