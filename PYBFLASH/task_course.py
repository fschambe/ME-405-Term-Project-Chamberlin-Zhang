# task_course.py
"""Task course follows a simple FSM, where each state acts as a given action
"""
from utime import ticks_ms, ticks_diff
import micropython
import math

# ----------------------------
# Helper functions
# ----------------------------
def wrap_pi(a):
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def ang_diff(a, b):
    """Smallest signed difference a-b wrapped to [-pi,pi]."""
    return wrap_pi(a - b)

# ----------------------------
# States
# ----------------------------
S0_INIT                = micropython.const(0)
S1_LF_FAST_CP0_TO_CP1  = micropython.const(1)
S2_LF_TO_GARAGE_DIST    = micropython.const(2)
S2B_TURN_90_IN_PLACE    = micropython.const(3)
S3_FWD_SHORT            = micropython.const(4)
S4_TURN_90_A            = micropython.const(5)
S5_FWD_UNTIL_BUTTON     = micropython.const(6)
S6_BACK_SHORT           = micropython.const(7)
S7_TURN_LEFT_90         = micropython.const(8)
S8_LF_TO_CP2            = micropython.const(9)
S9_TURN_RIGHT_90        = micropython.const(10)
S10_LF_TO_CP4           = micropython.const(11)
S11_TURN_TO_CP5_ANGLE   = micropython.const(12)
S12_FWD_TO_CP5_DIST     = micropython.const(13)
S13_DONE                = micropython.const(14)

class task_course:
    """
    Course-level state machine using:
      - followFlag: enables line follow outer loop
      - setpoint: base forward speed (mm/s)
      - steer: dv (mm/s) used by motor tasks (setpoint +/- steer)
      - xhat0: estimated displacement s (mm)
      - xhat1: estimated heading psi (rad)
      - button: button Pin, where 0 = pressed
      - startFlag: begins the course
      - driveEnable: enables wheels to move without followFlag or goFlag
      - Kp: allows adjustments to Kp mid course
      - debug: boolean to enable serial printing on which part of the course the robot is on
    """

    def __init__(self, followFlag, setpoint, steer,
                 xhat0_s, xhat1_psi,
                 bumper_pin,
                 startFlag,
                 driveEnable,
                 Kp,
                 debug=False):

        self.followFlag = followFlag
        self.setpoint   = setpoint
        self.steer      = steer

        self.xhat0_s    = xhat0_s
        self.xhat1_psi  = xhat1_psi

        self.button     = bumper_pin
        self.startFlag  = startFlag
        self.driveEnable = driveEnable
        self.Kp = Kp
        self.debug      = debug

        self.state = S0_INIT
        self.t0 = ticks_ms()

        # ----------------------------
        # Units:
        #   - distances are in mm
        #   - angles are radians.
        # ----------------------------
        self.V_FAST = 200.0     # mm/s (fast line follow)
        self.V_SLOW = 35.0     # mm/s (slow line follow for accurate turning)
        self.V_FWD  = 60.0     # mm/s (straight segments without line follow)
        self.V_STRAIGHT = 100  # mm/s (straight segment in garage)
        self.V_BACK = -30.0    # mm/s (back up)
        self.TURN_DV = 100.0    # mm/s steering command for in-place turns (setpoint=0)

        # Distances (mm)
        self.D_CP0_CP1 = 1360.0
        self.D_TURN_GARAGE = 300
        self.D_FWD_SHORT = 150.0
        self.D_BACK_SHORT = 25.0
        self.D_CP1_TO_CP2 = 385.0
        self.D_CP2_TO_CP4 = 1800.0
        self.D_TO_CP5 = 425.0

        # Angles (rad) 
        self.ANG_90 =  math.pi / 2.0 

        # Turn directions (sign convention):
        # If your steer mixing is:
        #   left_target  = setpoint - steer
        #   right_target = setpoint + steer
        # then:
        #   steer > 0 makes right wheel faster -> tends to turn LEFT (CCW).
        #
        # If your robot turns the opposite, flip these signs.
        self.SIGN_TURN_LEFT  = +1.0
        self.SIGN_TURN_RIGHT = -1.0

        # For CP5 special turn
        self.ANG_180 = 3.1           # rad
        self.ANG_TO_CP5 = 2.32       # rad
        self.ANG_OUT_OF_GARAGE = 1.4 #rad

        # Internal references
        self.s_ref   = 0.0
        self.psi_ref = 0.0

    # ----------------------------
    # Actuation helpers
    # ----------------------------
    def _lf(self, v_mm_s):
        """Enable line following at base speed v."""
        self.followFlag.put(True)
        self.setpoint.put(float(v_mm_s))
        # steer is produced by lineTask when followFlag is True
        # but still safe to clear if lineTask is gated correctly
        # (lineTask should overwrite steer quickly)

    def _straight(self, v_mm_s):
        """Drive straight without line follow."""
        self.followFlag.put(False)
        self.steer.put(0.0)
        self.setpoint.put(float(v_mm_s))

    def _turn_in_place(self, direction_sign):
        """
        Turn in place by commanding setpoint=0 and steer +/- TURN_DV.
        direction_sign should be SIGN_TURN_LEFT or SIGN_TURN_RIGHT.
        """
        self.followFlag.put(False)
        self.driveEnable.put(True)
        self.setpoint.put(0.0)
        self.steer.put(float(direction_sign) * float(self.TURN_DV))

    def _stop(self):
        self.followFlag.put(False)
        self.setpoint.put(0.0)
        self.steer.put(0.0)
        self.driveEnable.put(False)

    def _capture_s(self):
        self.s_ref = float(self.xhat0_s.get())

    def _capture_psi(self):
        self.psi_ref = float(self.xhat1_psi.get())

    def _dist_traveled(self):
        return float(self.xhat0_s.get()) - self.s_ref

    def _heading_delta(self):
        # how far current psi has moved from psi_ref
        return ang_diff(float(self.xhat1_psi.get()), self.psi_ref)

    def _button_pressed(self):
        # button.value = 1 when off, 0 when pressed
        return (self.button.value() == 0)

    def _dbg(self, msg):
        if self.debug:
            # Keep debug prints slow to avoid USB blocking
            now = ticks_ms()
            if ticks_diff(now, self.t0) > 250:
                self.t0 = now
                print("[COURSE]", msg)

    # ----------------------------
    # Main task
    # ----------------------------
    def run(self):
        while True:
            if self.state == S0_INIT:
                # Start clean
                self._stop()
                if self.startFlag.get():
                    self.startFlag.put(False)
                    self._capture_s()
                    # since robot lf is jittery, don't capture psi, 
                    # it will start capturing after its first initial 180 degree turn
                    self.driveEnable.put(True)
                    self._dbg("INIT -> CP0")
                    self.state = S1_LF_FAST_CP0_TO_CP1

            # ------------------------------------------------------------
            # CP0 -> CP1: fast setpoint + line follow for distance
            # ------------------------------------------------------------
            elif self.state == S1_LF_FAST_CP0_TO_CP1:
                self._lf(self.V_FAST)
                if self._dist_traveled() >= self.D_CP0_CP1:
                    self._capture_s()
                    self._dbg("Reached CP1 (by distance). Slow LF until +90deg heading change.")
                    self.state = S2_LF_TO_GARAGE_DIST

            # ------------------------------------------------------------
            # At CP1: slow LF until heading change is +90 degrees
            # ------------------------------------------------------------
            elif self.state == S2_LF_TO_GARAGE_DIST:
                # Line follow at slow speed until you reach the "garage turn" distance
                self._lf(self.V_FWD)

                if self._dist_traveled() >= self.D_TURN_GARAGE:
                    # capture heading reference at the moment you start turning
                    self._capture_s()
                    self._dbg("Reached D_TURN_GARAGE. Turn in place until +90 deg.")
                    self.state = S2B_TURN_90_IN_PLACE


            elif self.state == S2B_TURN_90_IN_PLACE:
                # Turn in place until heading has changed by 90 degrees
                self._turn_in_place(self.SIGN_TURN_RIGHT)

                if abs(self._heading_delta()) >= self.ANG_90:
                    self._capture_s()
                    self._dbg("90 deg turn complete. Forward short distance.")
                    self.state = S3_FWD_SHORT
            # ------------------------------------------------------------
            # Move forward a short set distance (no line follow)
            # ------------------------------------------------------------
            elif self.state == S3_FWD_SHORT:
                self._straight(self.V_FWD)
                if self._dist_traveled() >= self.D_FWD_SHORT:
                    self._capture_s()
                    self._dbg("Short fwd done. Turn 90 deg.")
                    self.state = S4_TURN_90_A

            # ------------------------------------------------------------
            # Turn 90 degrees right
            # ------------------------------------------------------------
            elif self.state == S4_TURN_90_A:
                self._turn_in_place(self.SIGN_TURN_RIGHT)
                if abs(self._heading_delta()) >= self.ANG_180:
                    self._capture_s()
                    self._dbg("Turn done. Go straight until button pressed.")
                    self.state = S5_FWD_UNTIL_BUTTON

            # ------------------------------------------------------------
            # Go straight until button pressed
            # ------------------------------------------------------------
            elif self.state == S5_FWD_UNTIL_BUTTON:
                self._straight(self.V_STRAIGHT)
                if self._button_pressed():
                    self._capture_s()
                    self._dbg("Button pressed. Back up a little.")
                    self.state = S6_BACK_SHORT

            # ------------------------------------------------------------
            # Back up a little
            # ------------------------------------------------------------
            elif self.state == S6_BACK_SHORT:
                self._straight(self.V_BACK)
                if abs(self._dist_traveled()) >= self.D_BACK_SHORT:
                    self._capture_s()
                    self._capture_psi()
                    self._dbg("Back done. Turn 90 deg left.")
                    self.state = S7_TURN_LEFT_90

            # ------------------------------------------------------------
            # Turn 90 degrees left 
            # ------------------------------------------------------------
            elif self.state == S7_TURN_LEFT_90:
                self._turn_in_place(self.SIGN_TURN_LEFT)
                if abs(self._heading_delta()) >= self.ANG_OUT_OF_GARAGE:
                    self._capture_s()
                    self._dbg("Turn done. Line follow to CP2 (distance).")
                    self.state = S8_LF_TO_CP2

            # ------------------------------------------------------------
            # Line follow for set distance to CP2 
            # ------------------------------------------------------------
            elif self.state == S8_LF_TO_CP2:
                self._lf(self.V_FWD)
                if self._dist_traveled() >= self.D_CP1_TO_CP2:
                    self._capture_s()
                    self._capture_psi()
                    self._dbg("Reached CP2. Turn 90 deg right.")
                    self.state = S9_TURN_RIGHT_90

            # ------------------------------------------------------------
            # Turn 90 degrees right
            # ------------------------------------------------------------
            elif self.state == S9_TURN_RIGHT_90:
                self._turn_in_place(self.SIGN_TURN_RIGHT)
                if abs(self._heading_delta()) >= self.ANG_90:
                    self._capture_s()
                    self.Kp.put(1.0)
                    self._dbg("Turn done. Line follow to CP4 (long distance).")
                    self.state = S10_LF_TO_CP4

            # ------------------------------------------------------------
            # Line follow for longer distance until CP3
            # ------------------------------------------------------------
            elif self.state == S10_LF_TO_CP4:
                self._lf(self.V_SLOW)
                if self._dist_traveled() >= self.D_CP2_TO_CP4:
                    self._capture_s()
                    self._capture_psi()
                    self._dbg("Reached CP3. Turn to CP5 angle.")
                    self.state = S11_TURN_TO_CP5_ANGLE

            # ------------------------------------------------------------
            # Turn to some angle, then go to CP4 distance
            # ------------------------------------------------------------
            elif self.state == S11_TURN_TO_CP5_ANGLE:
                turn_sign = self.SIGN_TURN_LEFT if self.ANG_TO_CP4 >= 0 else self.SIGN_TURN_RIGHT
                self._turn_in_place(turn_sign)
                if abs(self._heading_delta()) >= abs(self.ANG_TO_CP5):
                    self._capture_s()
                    self.Kp.put(0.5)
                    self._dbg("Angle to CP5 reached. Go forward to CP5 distance.")
                    self.state = S12_FWD_TO_CP5_DIST

            elif self.state == S12_FWD_TO_CP5_DIST:
                self._straight(self.V_FAST)
                if self._dist_traveled() >= self.D_TO_CP5:
                    self._capture_s()
                    self._capture_psi()
                    self._dbg("Reached CP5. Done.")
                    self.state = S13_DONE

            elif self.state == S13_DONE:
                self._stop()

            yield self.state