from motor import Motor
from encoder      import Encoder
from linesensor import line_sensor
from imu_driver import IMU
from task_motor   import task_motor
from task_user    import task_user
from task_line_follow import task_line_follow
from task_state_estimation import task_state_estimation
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
from pyb         import Timer, Pin, USB_VCP, I2C

# Build all driver objects first
tim1 = Timer(1, freq=20000)
left_motor = Motor(DIR_pin='PB3', PWM_pin='PA10', nSLP_pin='PC4', tim=tim1, chan=3)
right_motor = Motor(DIR_pin='PC1', PWM_pin='PA8', nSLP_pin='PC0', tim=tim1, chan=1)
tim2 = Timer(2, period = 0xFFFF, prescaler = 0) # Timer for left encoder
tim3 = Timer(3, period = 0xFFFF, prescaler = 0) # Timer for right encoder
right_encoder = Encoder(tim2, chA_pin='A0', chB_pin='A1')
left_encoder = Encoder(tim3, chA_pin='D5', chB_pin='D4')
button = Pin('PC8', Pin.IN, Pin.PULL_DOWN)
# button.value = 1 when off, 0 when pressed
rst = Pin(Pin.cpu.C9, Pin.OUT_PP)
rst.high()
i2c = I2C(1)
i2c.init(I2C.CONTROLLER, baudrate=100000)
imu = IMU(i2c)

# Build shares and queues
leftMotorGo   = Share("B",     name="Left Mot. Go Flag")
rightMotorGo  = Share("B",     name="Right Mot. Go Flag")
# Separate buffers for left and right motors so they don't contend
dataValues_left  = Queue("f", 100, name="Left Data Buffer")
timeValues_left  = Queue("L", 100, name="Left Time Buffer")
dataValues_right = Queue("f", 100, name="Right Data Buffer")
timeValues_right = Queue("L", 100, name="Right Time Buffer")
setpoint = Share("f", name="Velocity Setpoint")
Kp = Share("f", name="Proportional Gain")
Ki = Share("f", name="Integral Gain")
followFlag = Share("B", name="Follow Enabled")
followFlag.put(False)

steer = Share("f", name="Steering dv (mm/s)")
steer.put(0.0)

xhat0 = Share("f", name="xhat0")
xhat1 = Share("f", name="xhat1")
xhat2 = Share("f", name="xhat2")
xhat3 = Share("f", name="xhat3")
y0 = Share("f", name="y0_meas")
y1 = Share("f", name="y1_meas")
y2 = Share("f", name="y2_meas")
y3 = Share("f", name="y3_meas")

u_left  = Share("f", name="u_left_cmd")
u_right = Share("f", name="u_right_cmd")

# Defaults
# setpoint.put(20.0)
# Kp.put(0.2)
# Ki.put(0.4)
setpoint.put(50)
Kp.put(1.0)
Ki.put(0.0)
calibration = "imu_cal.bin"

# QTR-HD-07A pins (7 analogs)
LINE_PINS = ['PC5','PA5','PA6','PA7','PB1','PC2','PC3']
LINE_PINS.reverse()
#LINE_PINS = ['PC3', 'PC2', 'PB1', 'PA7', 'PA6', 'PA5', 'PC5']
sensor = line_sensor(LINE_PINS, emit_even='PH0', emit_odd='PH1', n_avg=5)
sensor.emitters_on()

# Build task class objects
leftMotorTask  = task_motor(left_motor, left_encoder,
                            leftMotorGo, setpoint, Kp, Ki, dataValues_left, timeValues_left,
                            followFlag=followFlag, steer=steer, side=-1)

rightMotorTask = task_motor(right_motor, right_encoder,
                            rightMotorGo, setpoint, Kp, Ki, dataValues_right, timeValues_right,
                            followFlag=followFlag, steer=steer, side=+1)
userTask = task_user(leftMotorGo, rightMotorGo, setpoint, Kp, Ki,
                     dataValues_left, timeValues_left,
                     dataValues_right, timeValues_right,
                     sensor=sensor, followFlag=followFlag, s=xhat0, psi=xhat1)
lineTask = task_line_follow(sensor, followFlag, setpoint, Kp, Ki, steer)
estTask = task_state_estimation(
    left_encoder, right_encoder,
    imu=imu,             # or None until you add the driver
    u0=u_left, u1=u_right,
    xhat0=xhat0, xhat1=xhat1, xhat2=xhat2, xhat3=xhat3,
    y0=y0, y1=y1, y2=y2, y3=y3,
    Ts_ms=10
)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 2, period = 10, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 2, period = 10, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))
task_list.append(Task(lineTask.run, name="Line Follow Task",
                      priority=2, period=20, profile=True))
task_list.append(Task(estTask.run, name="State Est Task",
                      priority=1, period=20, profile=True))

# Run the garbage collector preemptively
collect()

left_motor.enable()
right_motor.enable()

# Load saved calibration if it exists
try:
    with open(calibration, "rb") as f:
        cal = f.read()
    imu.write_calibration(cal)   # your driver must implement this
    print("Loaded IMU calibration from file.")
except OSError:
    print("No IMU calibration file found; calibrate once and save.")

# Calibrate/set gyro bias for estimator
print("Setting Gyro Bias, hold robot still")
imu.calibrate_gyro_bias(samples=200, dt_ms=5)
print("Gyro bias set.")

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        # if button.value():
        #     print(f"Button is off {button.value()}")
        # else:
        #     print(f"Button is pressed {button.value()}")
        
    except KeyboardInterrupt:
        print("Program Terminating")
        left_motor.disable()
        right_motor.disable()
        break

print("\n")
print(task_list)
print(show_all())