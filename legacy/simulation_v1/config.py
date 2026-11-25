"""Central configuration for the TVC IAE simulation codebase.

Every other module imports physical parameters, gains, and sweep ranges
from this file. Do not duplicate constants elsewhere.
"""

# Motor
BURN_TIME       = 2.44        # s
PLOT_END        = 3.5         # s

# NAR certified E12-4 thrust data (time_s, thrust_N)
THRUST_TIME = [0.000, 0.050, 0.100, 0.150, 0.200, 0.287, 0.300, 0.350, 0.400,
               0.450, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000, 1.200, 1.400,
               1.600, 1.800, 2.000, 2.100, 2.200, 2.300, 2.380, 2.440]
THRUST_N    = [0.0,  5.1,  10.0, 24.1, 31.5, 33.0, 30.0, 17.1, 14.6,
               13.2, 11.7, 11.2, 11.2, 10.5, 10.1,  9.8,  9.8,  9.7,
                9.7,  9.7,  9.8,  9.8,  9.7,  9.5,   6.1,  0.0]

# Vehicle
ROCKET_MASS     = 0.661       # kg (dry 0.600 + propellant 0.0612)
MOI             = 0.0231      # kg.m^2
MOMENT_ARM      = 0.288       # m (CoM-to-nozzle distance)
CP_OFFSET       = 0.173       # m (CP-CG offset = MOMENT_ARM * 0.6)
AIR_DENSITY     = 1.225       # kg/m^3
ROCKET_RADIUS   = 0.022       # m
AERO_DAMP_COEFF = 0.5

# PID gains
KP              = 8.5
KI              = 0.0         # Pure PD (zero integrator)
KD              = 1.2
CONTROL_HZ      = 150
DT_SIM          = 0.001       # s

# Servo / actuator
GIMBAL_LIMIT    = 10.0        # degrees hard stop
SERVO_RATE_LIM  = 300.0       # deg/s

# Kalman
Q_ANGLE         = 0.001
Q_RATE          = 0.003
R_MEASURE       = 0.03
GYRO_NOISE_STD  = 0.5         # deg/s
ACCEL_NOISE_STD = 2.5         # degrees (NOT g-units)
GYRO_DRIFT_RATE = 0.8         # deg/s post-burnout

# Sensitivity sweep ranges
SENSITIVITY_PARAMS = {
    'KP':           {'nominal': 8.5,   'range': (5.0, 15.0),  'n': 10},
    'KD':           {'nominal': 1.2,   'range': (0.5,  3.0),  'n': 10},
    'THRUST_SCALE': {'nominal': 1.0,   'range': (0.7,  1.3),  'n': 10},
    'MOMENT_ARM':   {'nominal': 0.288, 'range': (0.23, 0.35), 'n': 10},
    'GIMBAL_LIMIT': {'nominal': 10.0,  'range': (6.0, 14.0),  'n': 10},
    'GYRO_BIAS':    {'nominal': 0.0,   'range': (-2.0, 2.0),  'n': 10},
}

# IAE phase boundaries
IAE_IGNITION_END  = 0.3       # s
IAE_PEAK_END      = 0.6       # s
# tail-off is IAE_PEAK_END to BURN_TIME

# Figure output directory
FIG_DIR = 'figures_iae'
