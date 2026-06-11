"""Flight computer state machine: PAD_IDLE -> POWERED_FLIGHT -> BURNOUT.

Gates the controller: PID runs only during POWERED_FLIGHT. In simulation,
accel_magnitude_g is derived from thrust / (ROCKET_MASS * 9.81).
"""

PAD_IDLE = 'PAD_IDLE'
POWERED_FLIGHT = 'POWERED_FLIGHT'
BURNOUT = 'BURNOUT'

# Integer codes for logging
STATE_CODES = {PAD_IDLE: 0, POWERED_FLIGHT: 1, BURNOUT: 2}

LAUNCH_ACCEL_THRESHOLD_G = 2.0
COAST_ACCEL_THRESHOLD_G = 0.3


class FlightComputer:

    def __init__(self):
        self.state = PAD_IDLE
        self.launch_time = None
        self.burnout_time = None
        self._coast_sample_count = 0
        self._COAST_SAMPLES_REQUIRED = 5

    def update(self, t, accel_magnitude_g):
        """Call every control cycle. accel_magnitude_g = total accelerometer magnitude in g."""
        if self.state == PAD_IDLE:
            if accel_magnitude_g > LAUNCH_ACCEL_THRESHOLD_G:
                self.state = POWERED_FLIGHT
                self.launch_time = t

        elif self.state == POWERED_FLIGHT:
            if accel_magnitude_g < COAST_ACCEL_THRESHOLD_G:
                self._coast_sample_count += 1
            else:
                self._coast_sample_count = 0
            if self._coast_sample_count >= self._COAST_SAMPLES_REQUIRED:
                self.state = BURNOUT
                self.burnout_time = t

        # BURNOUT: terminal state, no further transitions

    @property
    def control_active(self):
        return self.state == POWERED_FLIGHT

    @property
    def state_int(self):
        return STATE_CODES[self.state]

    def reset(self):
        self.state = PAD_IDLE
        self.launch_time = None
        self.burnout_time = None
        self._coast_sample_count = 0
