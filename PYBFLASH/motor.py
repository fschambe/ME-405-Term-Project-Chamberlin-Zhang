from pyb import Pin, Timer

class Motor:
    '''A motor driver interface encapsulated in a Python class. Works with
       motor drivers using separate PWM and direction inputs such as the DRV8838
       drivers present on the Romi chassis from Pololu.'''
    
    def __init__(self, DIR_pin, nSLP_pin, PWM_pin, tim, chan):
        '''Initializes a Motor object'''
        self.DIR_pin = Pin(DIR_pin, Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP_pin, Pin.OUT_PP, value = 0)
        self.pwm_chan = tim.channel(chan, pin = Pin(PWM_pin), mode = Timer.PWM, pulse_width=0)
    
    def set_effort(self, effort: float):
        '''Sets the present effort requested from the motor based on an input value
           between -100 and 100'''
        if effort > 100.0:
            effort = 100.0
        elif effort < -100.0:
            effort = -100.0
        
        if effort >= 0:
            self.DIR_pin.low()
            self.pwm_chan.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.pwm_chan.pulse_width_percent(-effort)
        pass
            
    def enable(self):
        '''Enables the motor driver by taking it out of sleep mode into brake mode'''
        self.nSLP_pin.high()
        self.set_effort(0)
        pass
            
    def disable(self):
        '''Disables the motor driver by taking it into sleep mode'''
        self.nSLP_pin.low()