// ============================================================================
// tvc_2dof.ino
// 2-DOF (pitch + yaw) Thrust Vector Control firmware for Teensy 4.1.
// Static test stand live fire with an Estes E12-4 motor.
//
// The control law (PID gains, Kalman parameters, loop rate, gimbal limits)
// is numerically identical to the Python simulation (simulation/config.py,
// kalman.py, control.py). Eliminating control-law discrepancies as a
// variable is the entire point: only physics differences (no aerodynamic
// damping on a static stand, motor-to-motor variance) should explain dIAE.
//
// Hardware:
//   MCU:    Teensy 4.1
//   IMU:    MPU-6050, I2C at 400 kHz, address 0x68 (SDA pin 18, SCL pin 19)
//   Servos: SG90, pulse range 500-2500 us, via PWMServo (Teensy-native)
//   SD:     Teensy 4.1 built-in SD slot (BUILTIN_SDCARD)
//   Power:  servos from a regulated 5V supply, NOT the Teensy 5V pin
//   Debug:  Serial at 115200 baud
//
// PAPER NOTE (servo model): the paper's control effort section refers to
// MG90S servos but this firmware is written for SG90s. Verify which servo
// is actually installed and adjust SERVO_MIN_US / SERVO_MAX_US accordingly.
// SG90 pulse range (500-2500 us) may need per-servo calibration.
//
// PRE-FLIGHT CHECKLIST (all on the bench, before live fire):
//   1. Set SERVO_PITCH_PIN and SERVO_YAW_PIN below.
//   2. Verify IMU axis mapping in mpu_read() matches the physical mount.
//   3. Verify gimbal correction direction (see sign convention note at the
//      PID call in loop()). Tilt the rocket by hand: the motor must tilt to
//      push the nose back toward vertical, not away from it.
//   4. Calibrate servo pulse range per servo if travel looks asymmetric.
//
// CSV log format (fixed, consumed by compute_hardware_iae.py, do not edit):
//   timestamp_ms,accel_pitch_deg,accel_yaw_deg,gyro_pitch_dps,gyro_yaw_dps,
//   accel_mag_g,kalman_pitch_deg,kalman_yaw_deg,gimbal_pitch_cmd_deg,
//   gimbal_yaw_cmd_deg,flight_state,iae_pitch,iae_yaw
// ============================================================================

#include <Wire.h>
#include <PWMServo.h>
#include <SD.h>
#include <math.h>

// TODO: Set your servo pins before flashing
const int SERVO_PITCH_PIN = -1;  // TODO: e.g. 9
const int SERVO_YAW_PIN   = -1;  // TODO: e.g. 10

#define MPU_ADDR  0x68
#define SD_CS     BUILTIN_SDCARD

// ---------------------------------------------------------------------------
// Control parameters. MUST exactly match simulation/config.py.
// ---------------------------------------------------------------------------

// PID gains (config.py: KP, KI, KD)
const float KP = 8.5f;
const float KI = 0.0f;   // Pure PD
const float KD = 1.2f;

// Kalman noise parameters (config.py: Q_ANGLE, Q_RATE, R_MEASURE)
const float Q_ANGLE   = 0.001f;
const float Q_RATE    = 0.003f;
const float R_MEASURE = 0.03f;

// Actuator limits (config.py: GIMBAL_LIMIT, SERVO_RATE_LIM)
const float GIMBAL_LIMIT_DEG = 10.0f;   // +/-10 deg hard stop
const float SERVO_RATE_LIM   = 300.0f;  // deg/s

// Control loop (config.py: CONTROL_HZ)
const int CONTROL_HZ       = 150;
const int LOOP_INTERVAL_US = 6667;      // microseconds (1e6 / 150)

// Servo geometry. SG90 pulse range may need per-servo calibration; adjust
// SERVO_MIN_US / SERVO_MAX_US if the gimbal travel is asymmetric or short.
const int   SERVO_CENTER_US = 1500;     // us = 90 deg neutral
const int   SERVO_MIN_US    = 500;      // us
const int   SERVO_MAX_US    = 2500;     // us
const float DEG_PER_US      = 10.0f / 500.0f;  // 10 deg per 500 us half-range

// Launch/burnout detection.
// STATIC STAND CAVEAT: with the rocket rigidly clamped, the accelerometer
// reads ~1 g throughout the burn (the stand reacts the thrust), so launch
// detection relies on the ignition vibration spike exceeding LAUNCH_ACCEL_G
// and burnout will usually be caught by the BURN_TIME_MAX_S timeout rather
// than the free-fall (<0.3 g) criterion. Both paths end in BURNOUT safely.
const float LAUNCH_ACCEL_G  = 2.0f;   // above this = motor lit
const float BURNOUT_ACCEL_G = 0.3f;   // below this for N samples = burnout
const int   BURNOUT_SAMPLES = 5;      // consecutive samples required
const float BURN_TIME_MAX_S = 3.0f;   // safety timeout (E12-4 burn is 2.44 s)

// ---------------------------------------------------------------------------
// Kalman filter, one instance per axis. Scalar expansion of the 2-state
// filter in simulation/kalman.py: state = [angle_deg, gyro_bias_dps],
// F = [[1, -dt], [0, 1]], B = [dt, 0], H = [1, 0],
// Q = diag(Q_ANGLE, Q_RATE), R = R_MEASURE. Numerically identical.
// ---------------------------------------------------------------------------
struct KalmanAxis {
  float angle;       // estimated angle (deg)
  float bias;        // estimated gyro bias (deg/s)
  float P[2][2];     // error covariance

  void init() {
    angle = 0; bias = 0;
    P[0][0] = 1; P[0][1] = 0; P[1][0] = 0; P[1][1] = 1;
  }

  float update(float gyro_rate_dps, float accel_angle_deg, float dt) {
    // Predict: integrate bias-corrected gyro rate; propagate covariance
    float angle_pred = angle + (gyro_rate_dps - bias) * dt;
    float P00 = P[0][0] - dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_ANGLE;
    float P01 = P[0][1] - dt * P[1][1];
    float P10 = P[1][0] - dt * P[1][1];
    float P11 = P[1][1] + Q_RATE;

    // Update: innovate against the accelerometer tilt angle
    float S  = P00 + R_MEASURE;
    float K0 = P00 / S;
    float K1 = P10 / S;
    float y  = accel_angle_deg - angle_pred;
    angle = angle_pred + K0 * y;
    bias  = bias + K1 * y;
    P[0][0] = (1 - K0) * P00;
    P[0][1] = (1 - K0) * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P10;
    return angle;
  }
};

// ---------------------------------------------------------------------------
// PID controller, one instance per axis. Mirrors simulation/control.py:
// anti-windup clamp on the integrator, servo rate limit, then hard stop.
// ---------------------------------------------------------------------------
struct PIDAxis {
  float integral;
  float prev_error;
  float prev_output;

  void init() { integral = 0; prev_error = 0; prev_output = 0; }

  float update(float error, float dt) {
    float p = KP * error;
    integral += KI * error * dt;
    integral = constrain(integral, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);
    float d = (dt > 0) ? KD * (error - prev_error) / dt : 0;
    float raw = p + integral + d;

    // Servo rate limiting
    float max_delta = SERVO_RATE_LIM * dt;
    float output = constrain(raw, prev_output - max_delta, prev_output + max_delta);

    // Hard stop
    output = constrain(output, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);

    prev_error = error;
    prev_output = output;
    return output;
  }
};

// Explicit values: logged to CSV and matched by compute_hardware_iae.py
enum FlightState { PAD_IDLE = 0, POWERED_FLIGHT = 1, BURNOUT = 2 };

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
KalmanAxis  kalman_pitch, kalman_yaw;
PIDAxis     pid_pitch, pid_yaw;
PWMServo    servo_pitch, servo_yaw;
FlightState flight_state = PAD_IDLE;

IntervalTimer control_timer;
volatile bool control_flag = false;   // set by timer ISR, consumed in loop()

// MPU-6050 derived data
float gyro_pitch_dps, gyro_yaw_dps;
float accel_pitch_deg, accel_yaw_deg;
float accel_mag_g;

// Kalman estimates (deg)
float est_pitch = 0;
float est_yaw   = 0;

// Gimbal commands (degrees, +/- relative to center)
float gimbal_pitch_deg = 0;
float gimbal_yaw_deg   = 0;

// Flight timing
uint32_t launch_time_ms  = 0;
uint32_t burnout_time_ms = 0;
int      burnout_sample_count = 0;

// SD logging
File     log_file;
bool     sd_ok = false;
char     log_filename[16] = "flight01.csv";
uint32_t log_row_count = 0;
bool     log_close_pending = false;   // set on burnout, handled after final row

// IAE accumulation (computed on-board, rectangle rule; the post-processing
// script recomputes with trapz from the log and is the authoritative value)
float iae_pitch = 0;
float iae_yaw   = 0;

// ---------------------------------------------------------------------------
// MPU-6050
// ---------------------------------------------------------------------------
void mpu_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpu_init() {
  Wire.begin();
  Wire.setClock(400000);
  mpu_write_reg(0x6B, 0x00);   // PWR_MGMT_1: wake from sleep
  mpu_write_reg(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s
  mpu_write_reg(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
  delay(100);
}

void mpu_read() {
  // Burst-read 14 bytes starting at ACCEL_XOUT_H (0x3B):
  // accel XYZ, temp, gyro XYZ, each as big-endian int16
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  // Read into a buffer first: combining two Wire.read() calls in one
  // expression has unspecified evaluation order and can swap bytes
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();

  int16_t ax_raw = (int16_t)((b[0] << 8) | b[1]);
  int16_t ay_raw = (int16_t)((b[2] << 8) | b[3]);
  int16_t az_raw = (int16_t)((b[4] << 8) | b[5]);
  // b[6], b[7]: temperature, unused
  int16_t gx_raw = (int16_t)((b[8] << 8) | b[9]);
  int16_t gy_raw = (int16_t)((b[10] << 8) | b[11]);
  // b[12], b[13]: gyro Z (roll rate), unused in 2-DOF pitch/yaw control

  float ax = ax_raw / 8192.0f;   // +/-4 g range sensitivity
  float ay = ay_raw / 8192.0f;
  float az = az_raw / 8192.0f;
  float gx = gx_raw / 65.5f;     // +/-500 dps range sensitivity
  float gy = gy_raw / 65.5f;

  // TODO: verify axis mapping matches your IMU mounting orientation.
  // The assignment below assumes gyro X = pitch, gyro Y = yaw with Z along
  // the rocket's long axis. A 90 deg mounting rotation is easy to introduce;
  // confirm on the bench (tilt each axis, watch Serial) before live fire.
  gyro_pitch_dps  = gx;
  gyro_yaw_dps    = gy;
  accel_pitch_deg = atan2f(ax, az) * 180.0f / PI;   // tilt angle from accel
  accel_yaw_deg   = atan2f(ay, az) * 180.0f / PI;
  accel_mag_g     = sqrtf(ax * ax + ay * ay + az * az);
}

// ---------------------------------------------------------------------------
// Servo output
// ---------------------------------------------------------------------------
void write_servo_deg(PWMServo& servo, float deflection_deg) {
  // deflection_deg is relative to center, range -GIMBAL_LIMIT to +GIMBAL_LIMIT.
  // The target pulse is computed in microseconds so the calibration constants
  // stay in us. PWMServo has no writeMicroseconds(); its write() takes a
  // servo angle 0-180 mapped onto the attach(pin, min, max) pulse range, so
  // convert the pulse to that scale. Resolution is 1 servo degree, about
  // 11 us here, roughly 0.22 deg of gimbal: below the SG90 deadband.
  float pulse_us = SERVO_CENTER_US + deflection_deg / DEG_PER_US;
  pulse_us = constrain(pulse_us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  int angle = (int)roundf((pulse_us - SERVO_MIN_US) * 180.0f
                          / (float)(SERVO_MAX_US - SERVO_MIN_US));
  servo.write(angle);
}

// ---------------------------------------------------------------------------
// SD logging
// ---------------------------------------------------------------------------
void sd_init() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init FAILED, flying without log");
    sd_ok = false;
    return;
  }
  // Find next available filename: flight01.csv, flight02.csv, ...
  for (int i = 1; i <= 99; i++) {
    snprintf(log_filename, sizeof(log_filename), "flight%02d.csv", i);
    if (!SD.exists(log_filename)) break;
  }
  log_file = SD.open(log_filename, FILE_WRITE);
  if (!log_file) {
    Serial.println("SD open FAILED, flying without log");
    sd_ok = false;
    return;
  }
  // Fixed header, consumed by compute_hardware_iae.py. Do not change
  // column names or order.
  log_file.println(
      "timestamp_ms,accel_pitch_deg,accel_yaw_deg,gyro_pitch_dps,"
      "gyro_yaw_dps,accel_mag_g,kalman_pitch_deg,kalman_yaw_deg,"
      "gimbal_pitch_cmd_deg,gimbal_yaw_cmd_deg,flight_state,"
      "iae_pitch,iae_yaw");
  log_file.flush();
  sd_ok = true;
  Serial.print("Logging to ");
  Serial.println(log_filename);
}

void sd_log(uint32_t t_ms) {
  char row[200];
  snprintf(row, sizeof(row),
           "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.4f,%.4f",
           (unsigned long)t_ms,
           accel_pitch_deg, accel_yaw_deg,
           gyro_pitch_dps, gyro_yaw_dps,
           accel_mag_g,
           est_pitch, est_yaw,
           gimbal_pitch_deg, gimbal_yaw_deg,
           (int)flight_state,
           iae_pitch, iae_yaw);
  log_file.println(row);
  log_row_count++;
  // Buffered writes: flush every 50 rows (in loop(), never in the ISR) so
  // SD write latency cannot disrupt the control path on every cycle.
  if (log_row_count % 50 == 0) {
    log_file.flush();
  }
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------
void enter_burnout(uint32_t now_ms, bool timed_out) {
  flight_state = BURNOUT;
  burnout_time_ms = now_ms;
  gimbal_pitch_deg = 0;
  gimbal_yaw_deg   = 0;
  write_servo_deg(servo_pitch, 0);
  write_servo_deg(servo_yaw,   0);
  Serial.print(timed_out ? "BURNOUT (safety timeout) at " : "BURNOUT at ");
  Serial.print((now_ms - launch_time_ms) / 1000.0f, 3);
  Serial.print("s  |  IAE pitch=");
  Serial.print(iae_pitch, 4);
  Serial.print(" yaw=");
  Serial.println(iae_yaw, 4);
  // Close the log after the final BURNOUT row is written in loop(). The
  // rocket does not leave the stand, so data after burnout has no value;
  // closing immediately guarantees the file is intact when power is cut.
  log_close_pending = true;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void control_isr() {
  // Minimal ISR: set the flag only. No I2C, no SD, no float math here.
  control_flag = true;
}

void setup() {
  Serial.begin(115200);
  // Wait briefly for USB serial when tethered; never block on the pad
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  mpu_init();
  sd_init();

  kalman_pitch.init(); kalman_yaw.init();
  pid_pitch.init();    pid_yaw.init();

  // Attach servos (only if pins are set)
  if (SERVO_PITCH_PIN >= 0) servo_pitch.attach(SERVO_PITCH_PIN, SERVO_MIN_US, SERVO_MAX_US);
  if (SERVO_YAW_PIN   >= 0) servo_yaw.attach(SERVO_YAW_PIN,   SERVO_MIN_US, SERVO_MAX_US);
  if (SERVO_PITCH_PIN < 0 || SERVO_YAW_PIN < 0) {
    Serial.println("WARNING: servo pin(s) not set, see TODOs at top of file");
  }

  // Center servos
  write_servo_deg(servo_pitch, 0);
  write_servo_deg(servo_yaw,   0);

  // Start control timer: the ISR only sets a flag, all real work in loop()
  control_timer.begin(control_isr, LOOP_INTERVAL_US);

  Serial.println("TVC ready. Waiting for launch...");
}

// ---------------------------------------------------------------------------
// Main loop, paced to 150 Hz by the IntervalTimer flag
// ---------------------------------------------------------------------------
void loop() {
  if (!control_flag) return;
  control_flag = false;

  uint32_t now_ms = millis();
  float dt = LOOP_INTERVAL_US / 1e6f;  // 0.006667 s, fixed control period

  // 1. Read IMU
  mpu_read();

  // 2. Kalman update
  est_pitch = kalman_pitch.update(gyro_pitch_dps, accel_pitch_deg, dt);
  est_yaw   = kalman_yaw.update(gyro_yaw_dps,   accel_yaw_deg,   dt);

  // 3. State machine
  switch (flight_state) {
    case PAD_IDLE:
      if (accel_mag_g > LAUNCH_ACCEL_G) {
        flight_state = POWERED_FLIGHT;
        launch_time_ms = now_ms;
        Serial.println("LAUNCH DETECTED");
      }
      break;

    case POWERED_FLIGHT:
      if (accel_mag_g < BURNOUT_ACCEL_G) {
        burnout_sample_count++;
        if (burnout_sample_count >= BURNOUT_SAMPLES) {
          enter_burnout(now_ms, false);
        }
      } else {
        burnout_sample_count = 0;
      }

      // Safety timeout (primary burnout path on a clamped static stand,
      // where accel never drops below BURNOUT_ACCEL_G; see caveat above)
      if (flight_state == POWERED_FLIGHT &&
          (now_ms - launch_time_ms) > (uint32_t)(BURN_TIME_MAX_S * 1000)) {
        enter_burnout(now_ms, true);
      }
      break;

    case BURNOUT:
      // Do nothing: servos centered, log already closed
      break;
  }

  // 4. PID control (only during powered flight).
  // Sign convention: setpoint is 0 deg, so error = -estimate. Positive pitch
  // error produces a positive gimbal command that tilts the motor to push
  // the nose back toward vertical. If the rocket overcorrects or diverges on
  // the bench tilt test, the IMU axis sign is flipped: fix the mapping in
  // mpu_read(), do not change this call (it matches simulation.py exactly).
  if (flight_state == POWERED_FLIGHT) {
    gimbal_pitch_deg = pid_pitch.update(-est_pitch, dt);
    gimbal_yaw_deg   = pid_yaw.update(-est_yaw,   dt);

    write_servo_deg(servo_pitch, gimbal_pitch_deg);
    write_servo_deg(servo_yaw,   gimbal_yaw_deg);

    // Accumulate IAE on-board (rectangle rule, debug readout only)
    iae_pitch += fabsf(est_pitch) * dt;
    iae_yaw   += fabsf(est_yaw)   * dt;
  }

  // 5. Log to SD
  if (sd_ok) {
    sd_log(now_ms);
    if (log_close_pending) {
      // Final row (flight_state == BURNOUT) is written; seal the file
      log_file.flush();
      log_file.close();
      sd_ok = false;
      log_close_pending = false;
      Serial.print("Log closed: ");
      Serial.println(log_filename);
    }
  }
}
