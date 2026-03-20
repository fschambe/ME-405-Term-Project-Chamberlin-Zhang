from imu_driver import IMU
from pyb import I2C, Pin
import time
import os

# Hardware reset pin for BNO055
rst = Pin(Pin.cpu.C9, Pin.OUT_PP)
rst.high()

# Create + init I2C (I2C(1) corresponds to I2C1)
i2c = I2C(1)
i2c.init(I2C.CONTROLLER, baudrate=100000)


imu = IMU(i2c)

while True:
    h = imu.yaw()
    gz = imu.yaw_rate()
    cal = imu.get_cal_status()

    print("Yaw:", h, "Yaw Rate:", gz, "Cal:", cal)

    if cal == (3,3,3,3):
        break
    time.sleep(0.5)
    
cal_data = imu.read_calibration()

with open("imu_cal.bin", "wb") as f:
    f.write(cal_data)

print("Calibration saved.")
print("Files on board:", os.listdir())
