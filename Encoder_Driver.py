# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 14:13:51 2024

@author: NJ
"""
from pyb import Pin, Timer
from time import ticks_us, ticks_diff
import math

class Encoder:
    # @brief: interface with quadrature encoders
    # @details: create functions to initialize, update, zero, and return position and delta values from the encoder
    def __init__ (self, ENC_tim, CH_A_PIN, CH_B_PIN):
        # @brief: intialization of the encoder
        # @details: create encoder object with a timer and 2 channels of that timer
        #           setup timer as an encoder, and also reset all positional variables
        # @param ENC_tim: timer to be used for the encoder
        # @param ENC_CHA: read a timer channel as encoder channel A
        # @param ENC_CHB: read a timer channel as encoder channel B
        self.ENC_tim = ENC_tim
        self.ENC_CHA = ENC_tim.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.ENC_CHB = ENC_tim.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        self.prev_count = 0
        self.position = 0
        self.delta = 0
        self.ticks = 0
        pass
    
    def update (self):
        # @brief: update positional encoder values
        # @details: update a count variable with the current counter number
        #           update delta as the difference between current count and previous count
        #           account for overflow on the delta value so position stays accurate
        #           update position value as the sum of current position and delta
        #           update previous count value with current count after all other functions
        count = self.ENC_tim.counter()
        self.delta = count-self.prev_count
        if self.delta > 32768:
            self.delta -= 65536
        elif self.delta < -32768:
            self.delta += 65536
        #self.position = self.position*2*3.14159/16384
        #self.delta = self.delta*2*3.14159/16384*1000  #multiplied by frequency of data reading
        self.position = self.position+self.delta
        self.prev_count = count
        self.true_position = -self.position*2*3.14/1440
        self.true_delta = -self.delta*2*3.14/1440
        pass
    
    def get_position (self):
        # @brief: return position value
        # @details: return the position value found in update function
        return self.true_position
    
    def get_delta(self):
        # @brief: return delta value
        # @details: return the delta value found in update function
        return self.true_delta
    
    def zero(self):
        # @brief: set all positional values to zero
        # @details: set previous count, position, and delta all to zero
        self.prev_count = 0
        self.position = 0
        self.delta = 0
        self.true_position = 0
        self.true_delta = 0
        self.velocity = 0
        self.ticks = 0
        pass
    
    def get_velocity(self):
        deltaticks = ticks_diff(ticks_us(), self.ticks)
        self.velocity = self.delta*(2*3.14/1440)*1000000/deltaticks
        self.ticks = ticks_us()
        return -self.velocity, deltaticks
    
    def calculate_robot_position(self, left_encoder, right_encoder):
            wheel_radius = 0.035  # Wheel radius in meters
            track_width = 0.141    # Distance between wheels in meters
            # Calculate distances for left and right wheels
            left_distance = left_encoder.get_position() * self.wheel_radius
            right_distance = right_encoder.get_position() * self.wheel_radius
    
            # Compute change in orientation and linear distance
            delta_theta = (right_distance - left_distance) / self.track_width
            delta_distance = (right_distance + left_distance) / 2.0
    
            # Update robot's orientation
            self.theta += delta_theta
            self.theta = self.theta % 360  # Wrap theta to [0-360]
    
            # Update robot's position
            self.x_position += delta_distance * math.cos(math.radians(self.theta))
            self.y_position += delta_distance * math.sin(math.radians(self.theta))
    
            return self.x_position, self.y_position, self.theta