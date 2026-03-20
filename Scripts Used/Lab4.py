from motor import Motor
from encoder      import Encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue, show_all
from cotask       import Task, task_list
from gc           import collect
from pyb         import Timer, Pin, USB_VCP

# Build all driver objects first
tim1 = Timer(1, freq=20000)
left_motor = Motor(DIR_pin='PB3', PWM_pin='PA10', nSLP_pin='PC4', tim=tim1, chan=3)
right_motor = Motor(DIR_pin='PC1', PWM_pin='PA8', nSLP_pin='PC0', tim=tim1, chan=1)
tim2 = Timer(2, period = 0xFFFF, prescaler = 0) # Timer for left encoder
tim3 = Timer(3, period = 0xFFFF, prescaler = 0) # Timer for right encoder
right_encoder = Encoder(tim2, chA_pin='A0', chB_pin='A1')
left_encoder = Encoder(tim3, chA_pin='D5', chB_pin='D4')

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

# Build task class objects
leftMotorTask  = task_motor(left_motor,  left_encoder,
                            leftMotorGo, setpoint, Kp, Ki, dataValues_left, timeValues_left)
rightMotorTask = task_motor(right_motor, right_encoder,
                            rightMotorGo, setpoint, Kp, Ki, dataValues_right, timeValues_right)
userTask = task_user(leftMotorGo, rightMotorGo, setpoint, Kp, Ki,
                     dataValues_left, timeValues_left,
                     dataValues_right, timeValues_right)

# Add tasks to task list
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority = 1, period = 50, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority = 0, period = 0, profile=False))

# Run the garbage collector preemptively
collect()

left_motor.enable()
right_motor.enable()

# Run the scheduler until the user quits the program with Ctrl-C
while True:
    try:
        task_list.pri_sched()
        
    except KeyboardInterrupt:
        print("Program Terminating")
        left_motor.disable()
        right_motor.disable()
        break

print("\n")
print(task_list)
print(show_all())