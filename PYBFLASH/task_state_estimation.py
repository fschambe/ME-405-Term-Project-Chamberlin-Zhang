"""
State estimation task (discrete observer).

Implements:
    xhat_{k+1} = Ad * xhat_k + Bd * uhat_k
where:
    uhat_k = [u_k; y_k]  (stacks inputs and measured outputs)

"""

from utime import ticks_ms, ticks_diff
import micropython
from task_share import Share
import math
from ulab import numpy as np

S0_INIT = micropython.const(0) # State for initializing values and matricies
S1_RUN  = micropython.const(1) # State for state estimation, runs indefinitely



class task_state_estimation:
    """
    Discrete observer task.

    You provide:
      - encoders (or velocity shares) + IMU object (or yaw/yaw_rate shares)
      - input shares for u_k (e.g., commanded wheel velocities or efforts)
      - observer matrices Ad, Bd (precomputed offline)
      - output shares for xhat

    Minimal measured outputs example (edit to match your model):
      y_k = [v_L, v_R, yaw, yaw_rate]
    """

    def __init__(self,
                 left_encoder,
                 right_encoder,
                 imu=None,
                 # inputs u_k 
                 u0: Share = None, # applied voltage on left motor
                 u1: Share = None, # applied voltage on right motor
                 # output shares (states)
                 xhat0: Share = None, # distance traveled (mm)
                 xhat1: Share = None, # heading (rad)
                 xhat2: Share = None, # left wheel velocity (rad/s)
                 xhat3: Share = None, # right wheel velocity (rad/s)

                 y0: Share = None, # estimated distance traveled of left motor (mm)
                 y1: Share = None, # estimated distance traveled of right motor (mm)
                 y2: Share = None, # yaw (rad)
                 y3: Share = None, # yaw rate (rad/s)
                 # observer matrices
                 Ad=None, # Discrete System matrix (state memory)
                 Bd=None, # Discrete Input matrix (input effect)
                 Cd=None, # Discrete Output matrix (sensor data)
                 Dd=None, # Discrete Feedthrough matrix (input effect on sensors)
                 # task period (ms) only for documentation / debug prints
                 Ts_ms=20
                 ):

        self._state = S0_INIT

        self._encL = left_encoder
        self._encR = right_encoder
        self._imu  = imu
        self._psi0 = math.radians(self._imu.yaw()) if imu is not None else 0.0

        self._u0 = u0
        self._u1 = u1

        self._xhat0 = xhat0
        self._xhat1 = xhat1
        self._xhat2 = xhat2
        self._xhat3 = xhat3

        self._y0 = y0
        self._y1 = y1
        self._y2 = y2
        self._y3 = y3

        # Input into discrete observer MATLab script
        # self.r = 35 # mm
        # self.w = 141 # mm
        # self.tau = 51.1*4.7/1000 # s
        # self.Km = 4 # rad/V/s
        self.Ad = np.array([
            [0.5103, 0.0, 0.2494, 0.2494],
            [0.0, 0.0061, 0.0, 0.0],
            [-0.1763, 0.0, 0.4354, 0.4257],
            [-0.1763, 0.0, 0.4257, 0.4354],
        ])
        self.Bd = np.array([
            [0.0415264, 0.0415264, 0.2448702, 0.2448702, 0.0, 0.0],
            [0.0, 0.0, -0.0070483, 0.0070483, 0.0001, 0.0038972],
            [0.1704340, 0.1073202, 0.0881530, 0.0881530, 0.0, -1.9629909],
            [0.1073202, 0.1704340, 0.0881530, 0.0881530, 0.0, 1.9629909],
        ])

        self.Cd = np.array([
            [1.0, -70.5, 0.0, 0.0], # vL
            [1.0, 70.5, 0.0, 0.0], # vR
            [0.0, 1.0, 0.0, 0.0], # yaw
            [0.0, 0.0, -0.2482, 0.2482], # yaw_rate
        ])
        self.Dd = np.array([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ])
        self.L = np.array([ # observer gain matrix
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])


        self._Ts_ms = Ts_ms
        print("State Estimation Task object instantiated")

    def run(self):
        # Initial estimate
        xhat = np.array([0.0, 0.0, 0.0, 0.0])
        t_prev = ticks_ms()

        while True:
            if self._state == S0_INIT:
                # publish zeros on start
                if self._xhat0: self._xhat0.put(xhat[0])
                if self._xhat1: self._xhat1.put(xhat[1])
                if self._xhat2: self._xhat2.put(xhat[2])
                if self._xhat3: self._xhat3.put(xhat[3])
                t_prev = ticks_ms()
                self._state = S1_RUN

            elif self._state == S1_RUN:
                # time bookkeeping 
                t_now = ticks_ms()
                _dt_ms = ticks_diff(t_now, t_prev)
                t_prev = t_now

                # ---------
                # 1) read measured outputs y_k 
                # ---------
                # encoders are updated in motor task, but safe to update here too if needed
                try:
                    self._encL.update()
                    self._encR.update()
                except:
                    pass

                sL = self._encL.get_position()
                sR = self._encR.get_position()

                # IMU
                yaw = 0.0
                yaw_rate = 0.0
                if self._imu is not None:
                    try:
                        yaw = math.radians(self._imu.yaw()-self._psi0)          # rad
                        if yaw > math.pi:
                            yaw -= 2*math.pi
                        elif yaw < -math.pi:
                            yaw += 2*math.pi
                        yaw_rate = math.radians(self._imu.yaw_rate())# rad/s
                    except:
                        pass

                y = np.array([sL, sR, yaw, yaw_rate])

                # optionally publish measured y for logging/debug
                if self._y0: self._y0.put(y[0])
                if self._y1: self._y1.put(y[1])
                if self._y2: self._y2.put(y[2])
                if self._y3: self._y3.put(y[3])

                # ---------
                # 2) read inputs u_k (EDIT TO MATCH YOUR MODEL)
                # ---------
                # Example: two inputs u0,u1 (could be commanded wheel velocities, efforts, etc.)
                u0 = self._u0.get() if self._u0 is not None else 0.0
                u1 = self._u1.get() if self._u1 is not None else 0.0
                # u = [u0, u1]
                u = np.array([u0, u1])

                # Stack into uhat = [u; y]
                # uhat = u + y  # length must match Bd column count
                uhat = np.concatenate((u, y))

                # ---------
                # 3) observer update
                # ---------
                Ax = np.dot(self.Ad, xhat)
                Bu = np.dot(self.Bd, uhat)
                Cx = np.dot(self.Cd, xhat)
                Du = np.dot(self.Dd, uhat)
                yhat = Cx + Du
                xhat = Ax + Bu
                y_err = y - yhat
                L_err = np.dot(self.L, y_err)
                xhat = xhat + L_err

                # ---------
                # 4) publish state estimates
                # ---------
                if self._xhat0: self._xhat0.put(xhat[0])
                if self._xhat1: self._xhat1.put(xhat[1])
                if self._xhat2: self._xhat2.put(xhat[2])
                if self._xhat3: self._xhat3.put(xhat[3])

            yield self._state