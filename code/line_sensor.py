import pyb
from pyb import Pin, ADC

class line_sensor:
    """
    QTR-HD-07A: 7 analog channels, 4mm pitch, black line on white.
    - Supports EVEN/ODD emitter control pins (PH0/PH1)
    - Oversampling
    - White/black calibration
    - Normalized readings
    - Centroid in mm (0 = centered)
    """

    def __init__(self, analog_pins, emit_even=None, emit_odd=None, n_avg=5, pitch_mm=4.0):
        assert len(analog_pins) == 7

        self.adc = [ADC(Pin(p)) for p in analog_pins]
        self.n_avg = n_avg
        self.pitch_mm = pitch_mm

        self.emit_even = Pin(emit_even, Pin.OUT_PP) if emit_even else None
        self.emit_odd  = Pin(emit_odd,  Pin.OUT_PP) if emit_odd  else None

        # Calibration per channel
        self.white = [4095.0]*7
        self.black = [0.0]*7

        # Weights in mm: [-12, -8, -4, 0, 4, 8, 12]
        self.w = [ (i - 3) * pitch_mm for i in range(7) ]

    def emitters_on(self):
        if self.emit_even: self.emit_even.high()
        if self.emit_odd:  self.emit_odd.high()

    def emitters_off(self):
        if self.emit_even: self.emit_even.low()
        if self.emit_odd:  self.emit_odd.low()

    def read_raw(self, emitters=True):
        if emitters:
            self.emitters_on()
        else:
            self.emitters_off()

        vals = [0.0]*7
        for _ in range(self.n_avg):
            for i in range(7):
                vals[i] += self.adc[i].read()
        return [v/self.n_avg for v in vals]

    def read_norm(self, floor=0.0):
        """
        Returns 7 values in [0,1] where BLACK is HIGH.
        """
        raw = self.read_raw(emitters=True)

        norm = [0.0]*7
        for i in range(7):
            lo = min(self.white[i], self.black[i])
            hi = max(self.white[i], self.black[i])
            if hi - lo < 1e-6:
                x = 0.0
            else:
                # x ~ brightness (white high or low depending on wiring)
                x = (raw[i] - lo) / (hi - lo)
            x = max(0.0, min(1.0, x))

            # We want BLACK = HIGH. Most commonly, black reflects less => lower ADC,
            # so "darkness" = 1 - brightness:
            x = 1.0 - x

            if x < floor:
                x = 0.0
            norm[i] = x

        return norm

    def calibrate_white(self, seconds=1.0):
        t0 = pyb.millis()
        while pyb.millis() - t0 < int(seconds*1000):
            r = self.read_raw(emitters=True)
            for i in range(7):
                self.white[i] = r[i]
            pyb.delay(20)

    def calibrate_black(self, seconds=1.0):
        t0 = pyb.millis()
        while pyb.millis() - t0 < int(seconds*1000):
            r = self.read_raw(emitters=True)
            for i in range(7):
                self.black[i] = r[i]
            pyb.delay(20)

    def centroid_mm(self, floor=0.02):
        """
        Returns (pos_mm, strength, norm[7])
        pos_mm: line position relative to center (mm)
        strength: sum(norm) (confidence)
        """
        s = self.read_norm(floor=floor)
        denom = sum(s)
        if denom <= 1e-6:
            return 0.0, 0.0, s
        num = 0.0
        for i in range(7):
            num += self.w[i] * s[i]
        return num/denom, denom, s
