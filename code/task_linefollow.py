from utime import ticks_ms, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_line_follow:
    def __init__(self, sensor, followFlag, setpoint, Kp, Ki, steer):
        self._sensor = sensor
        self._follow = followFlag
        self._setpoint = setpoint
        self._Kp = Kp
        self._Ki = Ki
        self._steer = steer
        self._state = S0_INIT

        # --- persistent controller state ---
        self.esum = 0.0
        self.t_prev = ticks_ms()
        self.last_good_ms = ticks_ms()
        self.last_sign = 1
        self.pos_f = 0.0

        # --- tuning constants ---
        self.STRENGTH_MIN = 0.15
        self.SEARCH_DV = 15.0      # smaller search = less runaway
        self.LINE_GAIN = 10.0      # scales centroid error
        self.ALPHA = 0.25          # low-pass filter for pos_mm
        self.I_LIMIT = 200.0       # integral clamp (anti-windup)
        self._t_last_ctrl = ticks_ms()
        self.CTRL_MS = 50

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._steer.put(0.0)
                self.esum = 0.0
                self.t_prev = ticks_ms()
                self.last_good_ms = ticks_ms()
                self.last_sign = 1
                self.pos_f = 0.0
                self._state = S1_RUN
                yield self._state

            elif self._state == S1_RUN:
                t_now = ticks_ms()
                if ticks_diff(t_now, self._t_last_ctrl) < self.CTRL_MS:
                    yield self._state
                    continue
                self._t_last_ctrl = t_now
                dt = ticks_diff(t_now, self.t_prev) / 1000.0
                self.t_prev = t_now
                if dt <= 0:
                    dt = 0.02  # fallback

                # --- if follow disabled: reset + do nothing ---
                if not self._follow.get():
                    self._steer.put(0.0)
                    self.esum = 0.0
                    yield self._state
                    continue

                pos_mm, strength, _ = self._sensor.centroid_mm(floor=0.05)
                # print("pos_mm:", pos_mm, "strength:", strength)

                # Low-pass filter position to reduce noise/oscillation
                self.pos_f = (1.0 - self.ALPHA) * self.pos_f + self.ALPHA * pos_mm

                # Line detected?
                if strength >= self.STRENGTH_MIN:
                    self.last_good_ms = t_now

                    # error: center is 0 mm
                    e = -self.pos_f
                    e_scaled = self.LINE_GAIN * e

                    Kp = self._Kp.get()
                    Ki = self._Ki.get()

                    # integrate with clamp (anti-windup)
                    self.esum += e_scaled * dt
                    if self.esum > self.I_LIMIT:
                        self.esum = self.I_LIMIT
                    elif self.esum < -self.I_LIMIT:
                        self.esum = -self.I_LIMIT

                    dv = Kp * e_scaled + Ki * self.esum

                    # remember sign for search direction
                    if dv > 0:
                        self.last_sign = 1
                    elif dv < 0:
                        self.last_sign = -1

                else:
                    # Lost line: DON'T hold old dv (this causes runaway)
                    # Search gently in last known direction, and bleed integrator.
                    self.esum *= 0.5
                    dv = self.last_sign * self.SEARCH_DV

                # Saturate dv based on base speed
                v_base = self._setpoint.get()
                dv_max = max(20.0, 0.8 * abs(v_base))  # smaller than before for stability
                if dv > dv_max:
                    dv = dv_max
                elif dv < -dv_max:
                    dv = -dv_max

                self._steer.put(dv)

            yield self._state
