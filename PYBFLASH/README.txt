This is a MicroPython board

You can get started right away by writing your Python code in 'main.py'.

For a serial prompt:
 - Windows: you need to go to 'Device manager', right click on the unknown device,
   then update the driver software, using the 'pybcdc.inf' file found on this drive.
   Then use a terminal program like Hyperterminal or putty.
 - Mac OS X: use the command: screen /dev/tty.usbmodem*
 - Linux: use the command: screen /dev/ttyACM0

For online docs please visit http://docs.micropython.org/

Driver Layer (Low level):
 Motor - for PWM and motor direction control
 Encoder - for wheel position and velocity
 line_sensor - for QTR line sensing
 imu_driver - for yaw-rate measuremnets from the BNO055

Task-based control layer (middle level):
 task_motor - inner loop wheel speed control. Reads encoder velocity, compares it to desire setpoint, and computes motor effort using PI control. Also allows steering corrections to left and right setpoints differently.
 task_line_follow - outerloop line following. Reads line sensor centroid, converted line position error into steering command.
 task_state_estimation - Combines encoder and IMU measurements to estimate important states such as displacement and heading.
 task_user - Handles user interface and interaction, using input commands to allow the robot to start, test, and monitored.
 task_course - Controls the high level command for running throuhout the tasks. Adjustments to distance and angle parameters can be easily modified be changing the attributes found in the init state of the file. This is necessary, as measured distances can change depending on how reliable each component are, like the state estimation and power from the battery.

High Level:
main contains all the queues and shares, as well as instantiate the drivers, import calibrations, build task objects, set up defaults, and most importantly, runs the scheduler in perpetuity.
cotask - implements the scheduler
taskshare - implements shares and queues, variables that help tasks communicate with each other

How to use:
Upon start up, the user is prompted with the UI, displaying the commands that the user must follow. Any input not showned within the UI will re-prompt with all the commands. The user must not move the robot for a few seconds, as the gyro bias must be initiated, requiring the robot to be still. Only when the display "Gyro Bias set", may the user move the robot. The common flow is as followed:
Place robot over a black surface, where the line sensor will only see black, then press 'b' and wait for 1 second.
Place robot over a white surface, where the line sensor will only see white, then press 'w' and wait for 1 second.
The user may start the course by placing the robot in checkpoint 0, making sure the wheels are properly aligned, and it would be better if the line sensor is directly in the center of the line. Then the user may press the blue 'C13' user button to start the course.
If the user wants to do some testing with the robot, they can do the following:
Pressing 'p', 'i', or 's' to change the PI control and the setpoints. By default, setpoint is 50mm/s, Kp is 0.5, and Ki is 0. Then the user may press 'f' to enable line following or 'g' to generate a step response. The 'f' key is to be pressed again when the user wants to disable line following.
The common recommendation is to disable integral control (i=0) for line following, but use it for straight segments/set responses and lower Kp for a more stable and smooth movement (something like Kp = 0.2, Ki = 0.4).
