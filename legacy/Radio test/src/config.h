#pragma once

// ───────────────────────── pins ─────────────────────────
// Reserved elsewhere — do not reuse: 8,9,10,11,12,13 (RFM95W), 18,19 (I2C), 24 (RFM95 RST)
#define SERVO_X_PIN       2
#define SERVO_Y_PIN       3
#define HX_DOUT_PIN       4
#define HX_SCK_PIN        5
#define PYRO_GATE_PIN     6
#define ARM_SENSE_PIN     7
#define BUZZER_PIN        14
#define LED_PIN           LED_BUILTIN
#define PYRO_CONT_PIN     A0   // analog

// ───────────────────────── IMU ─────────────────────────
#define MPU_ADDR          0x68

// TODO: derive empirically per your gravity-based procedure (see HANDOFF.md).
// Identity placeholder — replace before trusting any logged attitude data.
static const int8_t R_BS[3][3] = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 }
};

// ───────────────────────── load cell (HX711) ─────────────────────────
// TODO: run 'z' tare command with known dead weight to derive these.
#define HX_OFFSET         0L
#define HX_SCALE          1.0f    // raw counts per Newton

// ───────────────────────── gimbal / servo ─────────────────────────
#define GIMBAL_MAX_DEG        12.0f
#define SERVO_US_MIN          1000
#define SERVO_US_MAX          2000
#define SERVO_X_CENTER        1500
#define SERVO_Y_CENTER        1500
#define SERVO_X_SIGN          1.0f
#define SERVO_Y_SIGN          1.0f
#define SERVO_X_US_PER_DEG    8.0f   // (SERVO_US_MAX-SERVO_US_MIN)/2 / GIMBAL_MAX_DEG, tune to your linkage
#define SERVO_Y_US_PER_DEG    8.0f
#define SERVO_SLEW_DEG_S      120.0f

// ───────────────────────── profiles ─────────────────────────
enum Profile : uint8_t { PROF_HOLD = 0, PROF_STEP_X, PROF_STEP_Y, PROF_RAMP_X, PROF_CHIRP_X, PROF_COUNT };

#define PROF_AMP_DEG      5.0f
#define STEP_PERIOD_S     1.0f
#define RAMP_PERIOD_S     2.0f
#define CHIRP_F0_HZ       0.5f
#define CHIRP_F1_HZ       10.0f
#define CHIRP_DUR_S       8.0f
#define DRY_RUN_S         10.0f

// ───────────────────────── sequencing ─────────────────────────
#define COUNTDOWN_MS          5000
#define PYRO_FIRE_MS          500
#define THRUST_DETECT_N       2.0f
#define NO_IGNITION_MS        1500
#define BURNOUT_N             1.0f
#define BURNOUT_HOLD_MS       300
#define POST_BURN_LOG_MS      3000
#define CONT_ADC_MIN          100   // tune to your pyro continuity resistor divider

// ───────────────────────── timing / telemetry ─────────────────────────
#define CTRL_PERIOD_US        4000    // 250 Hz
#define TELEM_PERIOD_US       50000   // 20 Hz