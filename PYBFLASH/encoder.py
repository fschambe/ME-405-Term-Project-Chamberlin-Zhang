from time import ticks_ms, ticks_diff   # Use to get dt value in update()
from pyb import Pin
import math

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
    
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.ARR       = tim.period()  # Auto-reload value of the timer
        tim.channel(1, pin=Pin(chA_pin), mode=tim.ENC_AB)
        tim.channel(2, pin=Pin(chB_pin), mode=tim.ENC_AB)
        self.tim = tim
        self.prev_time = ticks_ms()
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        current_count = self.tim.counter()
        self.dt = ticks_diff(ticks_ms(), self.prev_time)
        self.delta = current_count - self.prev_count
        if self.delta < -self.ARR//2:
            self.delta += self.ARR + 1
        elif self.delta > self.ARR//2:
            self.delta -= self.ARR + 1
        self.position += self.delta
        self.prev_count = current_count
        self.prev_time = ticks_ms()

            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position*70*math.pi/1440  # convert to mm
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return (self.delta/self.dt) * 70 * math.pi * 1000 / 1440
    
    def get_rad_velocity(self):
        '''Returns a measure of velocity in rad/s using the the most recently updated
           value of delta as determined within the update() method'''
        return (self.delta/self.dt) * ( 2* math.pi * 1000 / 1440 )
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.prev_count = self.tim.counter()
        pass