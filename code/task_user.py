''' This file demonstrates an example UI task using a custom class with a
    run method implemented as a generator
'''
from pyb import USB_VCP
from task_share import Share
import micropython
from linesensor import line_sensor

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_CMD  = micropython.const(1) # State 1 - wait for character input
S2_DIGIT = micropython.const(2) # State 2 - read digits until terminator
S3_COL  = micropython.const(3) # State 3 - wait for data collection to end
S4_DIS  = micropython.const(4) # State 4 - display the collected data

UI_prompt = ">: "
Menu = """+------------------------------------------------------------------------------+\r
| ME 405 Romi Tuning Interface Help Menu                                       |\r
+---+--------------------------------------------------------------------------+\r
| h | Print help menu                                                          |\r
| p | Enter proportional gain values                                           |\r
| i | Enter integral gain value                                                |\r
| s | Choose a new setpoint                                                    |\r
| g | Trigger step response and print results                                  |\r
| w | Calibrate white values for line sensor (place emitters on white)         |\r
| b | Calibrate black values for line sensor (place emitters on black)         |\r
| f | Toggle line following on/off (calibrate first)                           |\r
| v | Record State Variables                                                   |\r
+---+--------------------------------------------------------------------------+\r\n"""

class task_user:
    '''
    A class that represents a UI task. The task is responsible for reading user
    input over a serial port, parsing the input for single-character commands,
    and then manipulating shared variables to communicate with other tasks based
    on the user commands.
    '''

    def __init__(self, leftMotorGo, rightMotorGo, setpoint, Kp, Ki, dataValues, timeValues, dataValues_right=None, timeValues_right=None, sensor=None, followFlag: Share=None, s=None, psi=None):
        '''
        Initializes a UI task object
        
        Args:
            leftMotorGo (Share):  A share object representing a boolean flag to
                                  start data collection on the left motor
            rightMotorGo (Share): A share object representing a boolean flag to
                                  start data collection on the right motor
            setpoint (Share):     A share object representing the motor setpoint
            Kp (Share):           A share object representing the motor proportional gain
            Ki (Share):           A share object representing the motor integral gain
            dataValues (Queue):   A queue object used to store collected encoder
                                  position values
            timeValues (Queue):   A queue object used to store the time stamps
                                  associated with the collected encoder data
            dataValues_right (Queue): An optional queue object used to store collected encoder position values for the right motor (if not provided, will use dataValues)
            timeValues_right (Queue): An optional queue object used to store time stamps for the right motor (if not provided, will use timeValues)
            sensor (line_sensor): A line sensor object used for calibration commands
            followFlag (Share):   A share object representing a boolean flag to
                                    enable line following (used for follow command)
        '''
        
        self._state: int          = S0_INIT      # The present state
        
        self._leftMotorGo: Share  = leftMotorGo  # The "go" flag to start data
                                                 # collection from the left
                                                 # motor and encoder pair
        
        self._rightMotorGo: Share = rightMotorGo # The "go" flag to start data
                                                 # collection from the right
                                                 # motor and encoder pair
        self._setpoint: Share    = setpoint      # A share object representing the
                                                    # motor setpoint  
        self._Kp: Share         = Kp            # A share object representing the
                                                    # motor proportional gain
        self._Ki: Share         = Ki            # A share object representing the
                                                    # motor integral gain
        self._ser: stream         = USB_VCP()    # A serial port object used to
                                                 # read character entry and to
                                                 # print output
        self._sensor: line_sensor = sensor         # A line sensor object used for calibration commands
        self._follow: Share       = followFlag     # A share object representing a boolean flag
        # Support separate left/right buffers. If right buffers aren't
        # provided, fall back to the single buffers (backwards compatible).
        self._dataValues_left: Queue = dataValues
        self._timeValues_left: Queue = timeValues
        self._s = s
        self._psi = psi

        if dataValues_right is None:
            self._dataValues_right: Queue = dataValues
        else:
            self._dataValues_right: Queue = dataValues_right

        if timeValues_right is None:
            self._timeValues_right: Queue = timeValues
        else:
            self._timeValues_right: Queue = timeValues_right

        # Backwards-compatible alias
        self._dataValues: Queue = self._dataValues_left
        self._timeValues: Queue = self._timeValues_left

        # Tracks which motor(s) are being collected: 'left', 'right', 'both'
        self._collect_target: str = None
        self.out_share: BaseShare = Share('f', name="A float share") # A share object to store the computed number to be sent to the UI
        self.char_buf: str      = "" # A character buffer used to store incoming characters as they're received by the command processor
        self.digits:   set(str) = set(map(str,range(10))) # A set used to quickly check if a character entered by the user is a numerical digit.
        self.term:     set(str) = {"\r", "\n"} # A set used to quickly check if a character entered by the user is a terminator (a carriage return or newline)
        self.done = False # A flag used to track whether or not the command processing is still active

        self._ser.write("User Task object instantiated\r\n")


    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                self._ser.write(Menu)
                self._ser.write(UI_prompt)
                setKp = False
                setKi = False
                setSetpoint = False
                self._state = S1_CMD
                
            elif self._state == S1_CMD: # Wait for UI commands
                # Wait for at least one character in serial buffer
                if self._ser.any():
                    # Read the character and decode it into a string
                    raw = self._ser.read(1)
                    if raw:
                        try:
                            ch = raw.decode()
                        except:
                            ch = "" # default to newline if decoding fails
                        if ch in ("\r", "\n"):
                            self._ser.write(UI_prompt)
                            yield self._state
                            continue
                        if ch in {"l", "L"}: # Start data collection for left motor
                            while self._dataValues_left.any():
                                self._dataValues_left.get()
                            self._leftMotorGo.put(True)
                            self._collect_target = 'left'
                            self._ser.write("Starting left motor loop...\r\n")
                            self._ser.write("Starting data collection...\r\n")
                            self._ser.write("Please wait... \r\n")
                            self._state = S3_COL
                        elif ch in {"r", "R"}: # Start data collection for right motor
                            while self._dataValues_right.any():
                                self._dataValues_right.get()  
                            self._rightMotorGo.put(True)
                            self._collect_target = 'right'
                            self._ser.write("Starting right motor loop...\r\n")
                            self._ser.write("Starting data collection...\r\n")
                            self._ser.write("Please wait... \r\n")
                            self._state = S3_COL
                        elif ch in {"g", "G"}: # Trigger step response and print results
                            while self._dataValues_left.any():
                                self._dataValues_left.get()
                            while self._dataValues_right.any():
                                self._dataValues_right.get()
                            self._leftMotorGo.put(True)
                            self._rightMotorGo.put(True)
                            self._collect_target = 'both'
                            self._ser.write("Starting step response...\r\n")
                            self._ser.write("Starting data collection...\r\n")
                            self._ser.write("Please wait... \r\n")
                            self._state = S3_COL
                        elif ch in {"s", "S"}: # Set the velocity setpoint for the motor loop
                            self._ser.write("Enter Setpoint Value, s: ")
                            setSetpoint = True
                            self._state = S2_DIGIT

                        elif ch in {"p", "P"}: # Set the proportional gain for the motor loop
                            self._ser.write("Enter proportional gain, Kp: ")
                            setKp = True
                            self._state = S2_DIGIT
                            
                        elif ch in {"i", "I"}: # Set the integral gain for the motor loop
                            self._ser.write("Enter integral gain, Ki: ")
                            setKi = True
                            self._state = S2_DIGIT

                        elif ch in {"w", "W"}:
                            self._ser.write("Calibrating WHITE for 1s... place on white.\r\n")
                            self._sensor.calibrate_white(seconds=1.0)
                            self._ser.write("Done.\r\n")
                            self._ser.write(UI_prompt)

                        elif ch in {"b", "B"}:
                            self._ser.write("Calibrating BLACK for 1s... place on black line.\r\n")
                            self._sensor.calibrate_black(seconds=1.0)
                            self._ser.write("Done.\r\n")
                            self._ser.write(UI_prompt)

                        elif ch in {"f", "F"}:
                            now = not self._follow.get()
                            self._follow.put(now)
                            # if now:
                            #     self._ser.write("Follow Enabled\r\n")
                            # else:
                            #     self._ser.write("Follow Disabled\r\n")
                            #     self._follow.put(False)
                            self._ser.write("Follow " + ("ENABLED\r\n" if now else "DISABLED\r\n"))
                            self._ser.write(UI_prompt)
                        
                        elif ch in {"v","V"}:
                            distance = self._s.get()
                            heading = self._psi.get()
                            self._ser.write(f"Distance traveled: {distance} Current heading: {heading}\r\n")
                            self._ser.write(UI_prompt)

                        else:
                            self._ser.write("Invalid command\r\n")
                            self._ser.write(UI_prompt)     
                            self._state = S0_INIT                   

            elif self._state == S2_DIGIT:
                if not self.done:
                    if self._ser.any():
                        char_in = self._ser.read(1).decode()
                        if char_in in self.digits:
                            self._ser.write(char_in)
                            self.char_buf += char_in
                        elif char_in == "." and "." not in self.char_buf:
                            self._ser.write(char_in)
                            self.char_buf += char_in
                        elif char_in == "-" and len(self.char_buf) == 0:
                            self._ser.write(char_in)
                            self.char_buf += char_in
                        elif char_in == "\x7f" and len(self.char_buf) > 0:
                            self._ser.write(char_in)
                            self.char_buf = self.char_buf[:-1]
                        elif char_in in self.term:
                            if len(self.char_buf) == 0:
                                self._ser.write("\r\n")
                                self._ser.write("Value not changed\r\n")
                                self.char_buf = ""
                                self.done = True
                            elif self.char_buf not in {"-", "."}:
                                self._ser.write("\r\n")
                                value = float(self.char_buf)
                                self.out_share.put(value)
                                self._ser.write(f"Value set to {value}\r\n")
                                self.char_buf = ""
                                self.done = True
                        else:
                            self._ser.write("\r\nInvalid character, please enter a number:\r\n")
                            self.char_buf = ""
                else:
                    if setSetpoint:
                        self._setpoint.put(self.out_share.get())
                        setSetpoint = False
                    elif setKi:
                        self._Ki.put(self.out_share.get())
                        setKi = False
                    elif setKp:
                        self._Kp.put(self.out_share.get())
                        setKp = False
                    self.char_buf = ""
                    self.done = False
                    self._ser.write("Setpoint: " + str(self._setpoint.get()) + " mm/s\r\n")
                    self._ser.write("Kp: " + str(self._Kp.get()) + "%*s/mm\r\n")
                    self._ser.write("Ki: " + str(self._Ki.get()) + "%/mm\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
            
            elif self._state == S3_COL:
                # While the data is collecting (in the motor task) block out the
                # UI and discard any character entry so that commands don't
                # queue up in the serial buffer
                if self._ser.any(): self._ser.read(1)
                
                # When both go flags are clear, the data collection must have
                # ended and it is time to print the collected data.
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Setpoint: " + str(self._setpoint.get()) + " mm/s\r\n")
                    self._ser.write("Kp: " + str(self._Kp.get()) + "%*s/mm\r\n")
                    self._ser.write("Ki: " + str(self._Ki.get()) + "%/mm\r\n")
                    self._ser.write("BEGIN\r\n")
                    self._ser.write("Time, Velocity (mm/s)\r\n")
                    self._state = S4_DIS
            
            elif self._state == S4_DIS:
                # While data remains in the buffer, print that data in a command
                # separated format. Otherwise, the data collection is finished.
                # Print from the appropriate buffer(s) depending on which
                # motor(s) were collected. If both were collected, print the
                # left buffer then the right buffer.
                if (self._collect_target in ('left','both') and self._dataValues_left.any()):
                    t_s = self._timeValues_left.get() / 1000
                    v = self._dataValues_left.get()
                    self._ser.write(f"{t_s:.3f}, {v:.3f}\r\n")
                elif (self._collect_target in ('right','both') and self._dataValues_right.any()):
                    t_s = self._timeValues_right.get() / 1000
                    v = self._dataValues_right.get()
                    self._ser.write(f"{t_s:.3f}, {v:.3f}\r\n")
                else:
                    self._ser.write("END\r\n")
                    self._ser.write("--------------------\r\n")
                    self._ser.write("Distance traveled at final time: " + str(self._s.get()) + " mm\r\n")
                    # self._ser.write("Waiting for go command: 'l' for left, 'r' for right\r\n")
                    self._ser.write(Menu)
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD
            
            yield self._state
