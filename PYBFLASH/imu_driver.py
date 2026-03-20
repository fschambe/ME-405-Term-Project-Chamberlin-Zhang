import time

BNO055_ADDR = 0x28

# Register addresses
REG_OPR_MODE = 0x3D
REG_CALIB_STAT = 0x35
REG_EULER_H_LSB = 0x1A
REG_GYRO_DATA_Z_LSB = 0x18
REG_SYS_TRIGGER = 0x3F

# Operation modes
CONFIG_MODE = 0x00
NDOF_MODE   = 0x0C


class IMU:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = BNO055_ADDR

        self.reset()
        self.set_mode(CONFIG_MODE)
        time.sleep_ms(20)
        self.set_mode(NDOF_MODE)
        time.sleep_ms(20)
        self._last_gz = 0.0
        self._gz_bias = 0.0

    def reset(self):
        # Soft reset
        self.write_reg(REG_SYS_TRIGGER, 0x20)
        time.sleep_ms(700)

    def write_reg(self, reg, val):
        self.i2c.mem_write(bytes([val]), self.addr, reg)

    def read_reg(self, reg, nbytes=1):
        return self.i2c.mem_read(nbytes, self.addr, reg)

    def set_mode(self, mode):
        self.write_reg(REG_OPR_MODE, mode)
        time.sleep_ms(20)

    # -------------------------
    # Sensor Functions
    # -------------------------

    def get_cal_status(self):
        stat = self.read_reg(REG_CALIB_STAT)[0]
        sys  = (stat >> 6) & 0x03
        gyro = (stat >> 4) & 0x03
        accel= (stat >> 2) & 0x03
        mag  = stat & 0x03
        return sys, gyro, accel, mag
    
    def read_calibration(self):
        # Must be in CONFIG mode to read offsets
        self.set_mode(0x00)  # CONFIG
        time.sleep_ms(25)

        data = self.read_reg(0x55, 22)  # offset registers block

        self.set_mode(0x0C)  # back to NDOF
        time.sleep_ms(25)

        return data
    def write_calibration(self, data):
        self.set_mode(0x00)
        time.sleep_ms(25)

        self.i2c.mem_write(data, self.addr, 0x55)

        self.set_mode(0x0C)
        time.sleep_ms(25)

    def read_euler(self):
        data = self.read_reg(REG_EULER_H_LSB, 6)

        heading = int.from_bytes(data[0:2], 'little', True) / 16.0
        roll    = int.from_bytes(data[2:4], 'little', True) / 16.0
        pitch   = int.from_bytes(data[4:6], 'little', True) / 16.0

        return heading, roll, pitch

    def yaw(self):
        heading, _, _ = self.read_euler()
        return heading  # degrees

    def yaw_rate(self, max_abs_dps=500.0, retries=1):
        """
        Robust gyro Z (yaw rate) in deg/s.
        Never returns None. Filters bad I2C reads (0xFFFF) by holding last good value.
        Also clamps to +/- max_abs_dps for estimator safety.
        """
        for _ in range(retries + 1):
            data = self.read_reg(REG_GYRO_DATA_Z_LSB, 2)

            # Basic sanity
            if not data or len(data) != 2:
                continue

            # Reject common invalid read signature
            if data[0] == 0xFF and data[1] == 0xFF:
                continue

            # Signed conversion (your MicroPython supports positional signed flag)
            raw = int.from_bytes(data, 'little', True)

            # Another invalid signature (0xFFFF -> -1)
            if raw == -1:
                continue

            gz = raw / 16.0  # deg/s

            # Clamp (protect estimator)
            if gz > max_abs_dps:
                gz = max_abs_dps
            elif gz < -max_abs_dps:
                gz = -max_abs_dps

            self._last_gz = gz
            return gz

        # If all attempts failed, hold last value
        return self._last_gz
    
    def calibrate_gyro_bias(self, samples=200, dt_ms=5):
        """
        Estimate gyro Z bias while robot is stationary.
        Call once at startup with robot not moving.
        """
        s = 0.0
        n = 0
        for _ in range(samples):
            # read raw gyro z
            data = self.read_reg(REG_GYRO_DATA_Z_LSB, 2)
            if data and len(data) == 2 and not (data[0] == 0xFF and data[1] == 0xFF):
                raw = int.from_bytes(data, 'little', True)  # positional signed flag works for you
                if raw != -1:
                    s += (raw / 16.0)
                    n += 1
            time.sleep_ms(dt_ms)

        self._gz_bias = (s / n) if n > 0 else 0.0
        self._last_gz = 0.0
        return self._gz_bias