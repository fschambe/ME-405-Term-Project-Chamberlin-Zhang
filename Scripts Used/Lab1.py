import pyb
import time
from pyb import Pin, Timer
from time import sleep_ms
class motor_driver:
    '''initiating motor driver'''
    def __init__(self, DIR_pin, nSLP_pin, PWM_pin, tim, chan):
        self.DIR = Pin(DIR_pin, Pin.OUT_PP)
        self.nSLP = Pin(nSLP_pin, Pin.OUT_PP)
        self.pwm_chan = tim.channel(chan, pin = PWM_pin, mode = Timer.PWM, pulse_width=0)

l_en = Pin('PA3', Pin.OUT_PP) # Left motor enable pin
l_dir = Pin('PB3', Pin.OUT_PP) # Left motor direction pin
r_en = Pin('PC0', Pin.OUT_PP) # Right motor enable pin
r_dir = Pin('PC1', Pin.OUT_PP) # Right motor direction pin

tim1 = Timer(1, freq = 1000) # Timer for PWM
l_pwm = tim1.channel(3, pin = Pin('PA10'), mode = Timer.PWM, pulse_width=0)
r_pwm = tim1.channel(1, pin = Pin('PA8'), mode = Timer.PWM, pulse_width=0)

tim_2 = Timer(2, period = 0xFFFF, prescaler = 0) # Timer for left encoder
tim_2.channel(1, pin=Pin('A0'), mode=pyb.Timer.ENC_AB)
tim_2.channel(2, pin=Pin('A1'), mode=pyb.Timer.ENC_AB)

tim_3 = Timer(3, period = 0xFFFF, prescaler = 0) # Timer for right encoder
tim_3.channel(1, pin=Pin('D5'), mode=pyb.Timer.ENC_AB)
tim_3.channel(2, pin=Pin('D4'), mode=pyb.Timer.ENC_AB)

# # # Enable both motors
# l_en.high()
# r_en.high()

# # Set direction forward
# l_dir.high()
# r_dir.high()

# # Spin slowly
# l_pwm.pulse_width_percent(20)
# r_pwm.pulse_width_percent(20)

# sleep_ms(2000)



# # Stop
# l_pwm.pulse_width_percent(0)
# r_pwm.pulse_width_percent(0)

# l_en.low()
# r_en.high()
  
# print(65535 - tim_2.counter() , 65536 - tim_3.counter())
l_en.high()
r_en.high()
start_time = time.time()
while True:
    elapsed_time = time.time() - start_time
    l_pwm.pulse_width_percent(20)
    r_pwm.pulse_width_percent(20)
    if elapsed_time < 3:
        if elapsed_time < 1.5:
            l_dir.low()
            r_dir.low()
        else:
            l_dir.high()
            r_dir.high()
    else:
        break
    print("Left wheel: ", 65535 - tim_3.counter(), "Right wheel: ", 65536 - tim_2.counter())
l_pwm.pulse_width_percent(0)
r_pwm.pulse_width_percent(0)

l_en.low()
r_en.low()

