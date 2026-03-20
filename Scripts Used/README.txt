All scripts, except for automated_script.py are to be replaced by main.py in the PYBFLASH. Each LabX.py represents different labs, following the progression of the final product.

Lab0.py: Interrupt Callbacks and ADC Reading
Lab1.py: Romi Hardware Configuration
Lab2.py: Writing Hardware Drivers
Lab3.py: Closed Loop Control and the Scheduler
Lab4.py: UI Design and Automated Data Collection
Lab5.py: Line Following
Lab6.py: State Estimation

Helper mains:
IMU_Calibration.py: Used past Lab6, IMU parameters are set by running the program and moving the robot along many orientations until all 4 parts are calibrated (3). The parameters are automatically saved into imu_cal.bin file, which will be loaded upon main.
Km.py: Follows the button-style FSM system in Lab2, uses three different applied voltages and displays the wheel velocities, allows calculation of Km that was used in the MATLAB observer design.
automated_script: Built to supplement Lab4 for automatic data collection. Allows the script to send serial inputs without having the user be connected through the serial port. Script will automatically run tests onto the robot, save the step responses, then display and save the step response plot in a png and csv.