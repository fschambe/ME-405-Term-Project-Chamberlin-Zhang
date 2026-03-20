import pyb
from pyb import Pin, Timer, ADC
from array import array
from time import sleep_ms
PA5 = Pin('PC1', pyb.Pin.OUT_PP)
PA4 = Pin(pyb.Pin('PC0'))
idx = 0
data = array('H', 1000*[0])
adc = pyb.ADC(PA4)
button = Pin('C13', Pin.IN)
printData = False

def tim_cb(tim):
    global data, idx
    if idx < len(data):
        PA5.high()
    if idx >= len(data):
        tim7.callback(None)
        PA5.low()
    data[idx] = adc.read()
    idx += 1

def button_cb(pin):
        global printData
        printData = True

button.irq(trigger=Pin.IRQ_FALLING, handler=button_cb)

tim7 = pyb.Timer(7, freq = 1000)
tim7.callback(tim_cb)
sleep_ms(1500)
while True:
     if printData:
        for i, value in enumerate(data):
            print(f"{i}, {data[i]}")
        printData = False