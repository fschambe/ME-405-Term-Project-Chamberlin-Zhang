import micropython
from task_share import Share
from ulab import numpy as np

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_observer:
    """
    Discrete-time observer:
        xhat[k+1] = Ad*xhat[k] + Bd*u[k] + Ld*(y[k] - C*xhat[k])
    """

    def __init__(self,
                 u_left, u_right,
                 s_left_meas, s_right_meas,
                 theta_meas, omega_meas,
                 xhat0, xhat1, xhat2, xhat3):

        self._state = S0_INIT

        # Shares in
        self.u_left = u_left
        self.u_right = u_right
        self.sL = s_left_meas
        self.sR = s_right_meas
        self.theta = theta_meas
        self.omega = omega_meas

        # Shares out
        self.xhat0 = xhat0
        self.xhat1 = xhat1
        self.xhat2 = xhat2
        self.xhat3 = xhat3

        # ---- PLACEHOLDER MATRICES (replace with your precomputed values) ----
        # xhat is 4x1, u is 2x1, y is 4x1
        self.Ad = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1],
        ], dtype=float)

        self.Bd = np.array([
            [0,0],
            [0,0],
            [0,0],
            [0,0],
        ], dtype=float)

        self.C = np.array([
            [1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1],
        ], dtype=float)

        self.Ld = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
        ], dtype=float)

        # State memory
        self.xhat = np.zeros((4,1), dtype=float)

    def run(self):
        while True:

            if self._state == S0_INIT:
                self.xhat[:] = 0.0
                self._state = S1_RUN

            elif self._state == S1_RUN:

                # Build u (2x1)
                u = np.array([
                    [float(self.u_left.get())],
                    [float(self.u_right.get())]
                ], dtype=float)

                # Build y (4x1)  (theta/omega are placeholders until IMU exists)
                y = np.array([
                    [float(self.sL.get())],
                    [float(self.sR.get())],
                    [float(self.theta.get())],
                    [float(self.omega.get())]
                ], dtype=float)

                # Observer update
                yhat = np.dot(self.C, self.xhat)
                self.xhat = np.dot(self.Ad, self.xhat) + np.dot(self.Bd, u) + np.dot(self.Ld, (y - yhat))

                # Publish estimated states
                self.xhat0.put(float(self.xhat[0,0]))
                self.xhat1.put(float(self.xhat[1,0]))
                self.xhat2.put(float(self.xhat[2,0]))
                self.xhat3.put(float(self.xhat[3,0]))

            yield self._state
