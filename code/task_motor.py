''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
from motor import Motor
from encoder      import Encoder
from task_share   import Share, Queue
from utime        import ticks_ms, ticks_diff
import micropython

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control

class task_motor:
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: Motor, enc: Encoder,
                 goFlag: Share, setpoint: Share, Kp: Share, Ki: Share, dataValues: Queue, timeValues: Queue, followFlag=None, steer=None, side=0, driveEnable=None):
        '''
        Initializes a motor task object
        
        Args:
            mot (Motor): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            setpoint (Share):   A share object representing the motor setpoint
            Kp (Share):         A share object representing the motor proportional gain
            Ki (Share):         A share object representing the motor integral gain
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
            followFlag (Share): A share object representing a boolean flag to
                                enable line following (optional, for line follow task)
            steer (Share):      A share object representing the steering command
                                for line following (optional, for line follow task)
            side (int):         An integer representing the side of the robot
                                that this motor is on (0 for left, 1 for right, optional)
        '''

        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: Motor = mot        # A motor object
        
        self._enc: Encoder      = enc        # An encoder object
        
        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection
        
        self._setpoint: Share   = setpoint   # A share object representing the
                                             # motor setpoint
        
        self._Kp: Share        = Kp         # A share object representing the
                                             # motor proportional gain
        self._Ki: Share        = Ki         # A share object representing the
                                             # motor integral gain
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data
        
        self._follow = followFlag    # A share object representing a boolean flag to
                                     # enable line following (optional, for line follow task)
        self._steer = steer         # A share object representing the steering command
                                     # for line following (optional, for line follow task)  
        self._side = side           # An integer representing the side of the robot
                                        # that this motor is on (0 for left, 1 for right, optional)
        self._driveEnable=driveEnable
        print("Motor Task object instantiated")
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                esum = 0
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                follow_on = (self._follow is not None and self._follow.get())
                drive_on = (self._driveEnable is not None and self._driveEnable.get())
                if self._goFlag.get() or follow_on or drive_on:
                    # print("Starting motor loop")
                    # self._mot.enable() # Enable the motor driver so that the motor can be actuated when effort is applied
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    esum = 0
                    prev_time = ticks_ms()
                    self._enc.zero() # Set the present position of the motor to be zero so that the control law has a well-defined starting point
                    self._startTime = ticks_ms()

                    # Clear the queues of any old data
                    if self._goFlag.get():
                        while self._dataValues.any():
                            self._dataValues.get()
                        while self._timeValues.any():
                            self._timeValues.get()

                    self._state = S2_RUN
                
            elif self._state == S2_RUN: # Closed-loop control state
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")
                
                # Run the encoder update algorithm and then capture the present
                # position of the encoder. You will eventually need to capture
                # the motor speed instead of position here.
                self._enc.update()
                vel = self._enc.get_velocity()
                # Collect a timestamp to use for this sample
                t   = ticks_ms()
                dt = ticks_diff(t, prev_time)
                if (dt/1000) <= 0:
                    dt = 1e-3 # Avoid division by zero
                prev_time = t

                base = self._setpoint.get()

                if self._steer is not None:
                    dv = self._steer.get()
                follow_on = (self._follow is not None and self._follow.get())
                drive_on = (self._driveEnable is not None and self._driveEnable.get())

                if (follow_on or drive_on) and self._steer is not None:
                    setpoint = base + self._side * dv
                else: 
                    setpoint = base

                # if self._follow is not None and self._follow.get() and self._steer is not None:
                #     dv = self._steer.get()
                #     # left side should subtract, right should add (or vice versa)
                #     setpoint = base + self._side * dv
                # else:
                #     setpoint = base
                # if self._driveEnable.get() and base == 0.0 and abs(self._steer.get()) > 1:
                #     if self._side == -1:
                #         print("TURN: base", base, "dv", dv, "L sp", setpoint)
                #     else:
                #         print("TURN: base", base, "dv", dv, "R sp", setpoint)
                
                # Actuate the motor using a control law. PID controller that
                # uses feedback from the velocity measurement.
                Kp = self._Kp.get()
                Ki = self._Ki.get()
                error = setpoint - vel
                effort = Kp * error + Ki * esum
                esum += error * dt/1000
                # Saturate the effort to be within the bounds of -100 to 100
                if effort > 100.0:
                    effort = 100.0
                elif effort < -100.0:
                    effort = -100.0
                self._mot.set_effort(effort)

                # Log only if goFlag is set
                if self._goFlag.get():
                    # Store the sampled values in the queues
                    self._dataValues.put(vel)
                    self._timeValues.put(ticks_diff(t, self._startTime))
                    # Continue following when queue is full
                    if self._dataValues.full():
                        self._goFlag.put(False)
                # Stop logging if goFlag is not set and follow is not on
                follow_on = (self._follow is not None and self._follow.get())
                drive_on = (self._driveEnable is not None and self._driveEnable.get())
                if not self._goFlag.get() and not follow_on and not drive_on:
                    # print("Stopping motor loop")
                    self._mot.set_effort(0) # Stop motor, but allow it to be actuated again without re-enabling
                    self._state = S1_WAIT

            yield self._state
