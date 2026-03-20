import pyb
import time
from motor import Motor
from encoder import Encoder
from pyb import Pin, Timer

# Initialize motors and encoders objects
tim1 = Timer(1, freq=20000)
left_motor = Motor(DIR_pin='PB3', PWM_pin='PA10', nSLP_pin='PC4', tim=tim1, chan=3)
right_motor = Motor(DIR_pin='PC1', PWM_pin='PA8', nSLP_pin='PC0', tim=tim1, chan=1)
tim2 = Timer(2, period = 0xFFFF, prescaler = 0) # Timer for left encoder
tim3 = Timer(3, period = 0xFFFF, prescaler = 0) # Timer for right encoder
right_encoder = Encoder(tim2, chA_pin='A0', chB_pin='A1')
left_encoder = Encoder(tim3, chA_pin='D5', chB_pin='D4')

def update_encoders(tim):
    left_encoder.update()
    right_encoder.update()
tim7 = pyb.Timer(7, freq=1000)
tim7.callback(update_encoders)

left_motor.enable()
right_motor.enable()
global task
task = 0
button = Pin('C13', Pin.IN)
def button_cb(pin):
    global task
    task += 1
button.irq(trigger=Pin.IRQ_FALLING, handler=button_cb)
try:
    while True:
        pyb.delay(100)
        print(f"Left Position: {left_encoder.get_position()}, Left Velocity: {left_encoder.get_velocity():.2f} mm/s")
        print(f"Right Position: {right_encoder.get_position()}, Right Velocity: {right_encoder.get_velocity():.2f} mm/s")
        print("task =", task)
        if task == 1:
            # Spin forward
            left_motor.set_effort(30)
            right_motor.set_effort(30)
            print(left_encoder.get_rad_velocity(), right_encoder.get_rad_velocity())
        elif task == 2:
            # Individual wheel test
            left_motor.set_effort(50)
            right_motor.set_effort(50)
            print(left_encoder.get_rad_velocity(), right_encoder.get_rad_velocity())
        elif task == 3:
            # Individual wheel test 
            left_motor.set_effort(70)
            right_motor.set_effort(70)
            print(left_encoder.get_rad_velocity(), right_encoder.get_rad_velocity())    
        elif task == 4:
            # Stop
            left_motor.disable()
            right_motor.disable()
            tim7.callback(None)
            print("Stopped.")
except KeyboardInterrupt:
    pass
finally:
    # ensure motors are stopped and timer callback removed on exit
    left_motor.set_effort(0)
    right_motor.set_effort(0)
    left_motor.disable()
    right_motor.disable()
    tim7.callback(None)
    print("Program terminated.")
