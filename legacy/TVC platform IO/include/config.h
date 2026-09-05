#pragma once
#include <Arduino.h>

// ═══════════════════════ SERVO / GIMBAL ═══════════════════════
// Values marked FILL come from tvc_servo_ident 'r' output.
constexpr uint8_t  SERVO_X_PIN = 5;   // unchanged
constexpr uint8_t  SERVO_Y_PIN = 3;   // was 2

constexpr float DRY_RUN_S = 6.0f;   // dry run profile duration

constexpr uint16_t SERVO_X_CENTER     = 1500;   // FILL, us
constexpr uint16_t SERVO_Y_CENTER     = 1500;   // FILL, us
constexpr int8_t   SERVO_X_SIGN       = +1;     // FILL
constexpr int8_t   SERVO_Y_SIGN       = +1;     // FILL
constexpr float    SERVO_X_US_PER_DEG = 13.5f;  // FILL
constexpr float    SERVO_Y_US_PER_DEG = 13.5f;  // FILL

constexpr float    GIMBAL_MAX_DEG     = 5.0f;   // soft limit, both axes
constexpr uint16_t SERVO_US_MIN       = 1150;   // hard clamp
constexpr uint16_t SERVO_US_MAX       = 1850;
constexpr float    SERVO_SLEW_DEG_S   = 400.0f; // command ramp ceiling

// ═══════════════════════ LOAD CELL (HX711) ════════════════════
constexpr uint8_t  HX_DOUT_PIN        = 20;
constexpr uint8_t  HX_SCK_PIN         = 21;
constexpr int32_t  HX_OFFSET          = 0;        // FILL: raw reading at zero load
constexpr float    HX_SCALE           = 21500.0f; // FILL: raw counts per newton
// Tie the HX711 RATE pin HIGH for 80 SPS. At 10 SPS this data is useless.

// ═══════════════════════ PYRO / IGNITION ══════════════════════
constexpr uint8_t  PYRO_GATE_PIN      = 6;   // low-side logic-level MOSFET gate
constexpr uint8_t  PYRO_CONT_PIN      = A0;  // continuity sense divider
constexpr uint8_t  ARM_SENSE_PIN      = 7;   // reads state of physical arm switch
constexpr uint16_t PYRO_FIRE_MS       = 1500; // hard ceiling on gate-high time
constexpr uint16_t CONT_ADC_MIN       = 120;  // below this = open igniter

// ═══════════════════════ INDICATORS ═══════════════════════════
constexpr uint8_t  BUZZER_PIN         = 4;
constexpr uint8_t  LED_PIN            = 13;

// ═══════════════════════ IMU ══════════════════════════════════
constexpr uint8_t  MPU_ADDR           = 0x68;
// Signed permutation, sensor frame -> body frame. Derive with the gravity
// procedure and verify det = +1 before trusting this.
constexpr int8_t   R_BS[3][3] = {{1,0,0},
                                 {0,1,0},
                                 {0,0,1}};   // FILL

// ═══════════════════════ SEQUENCE ═════════════════════════════
constexpr uint32_t COUNTDOWN_MS       = 10000;
constexpr float    THRUST_DETECT_N    = 5.0f;   // ignition confirmed above this
constexpr float    BURNOUT_N          = 2.0f;
constexpr uint32_t BURNOUT_HOLD_MS    = 250;    // sustained below BURNOUT_N
constexpr uint32_t POST_BURN_LOG_MS   = 3000;
constexpr uint32_t NO_IGNITION_MS     = 4000;   // abort if no thrust by T+this

// ═══════════════════════ RATES ════════════════════════════════
constexpr uint32_t CTRL_PERIOD_US     = 4000;   // 250 Hz servo + log
constexpr uint32_t TELEM_PERIOD_US    = 50000;  // 20 Hz

// ═══════════════════════ PROFILES ═════════════════════════════
enum Profile : uint8_t {
  PROF_HOLD = 0,   // both axes centered: clean thrust curve baseline
  PROF_STEP_X,     // square wave on X
  PROF_STEP_Y,
  PROF_RAMP_X,     // slow triangle, checks linearity + hysteresis
  PROF_CHIRP_X,    // log sweep, full frequency response in one burn
  PROF_COUNT
};

constexpr float    PROF_AMP_DEG       = 3.0f;
constexpr float    STEP_PERIOD_S      = 0.60f;
constexpr float    RAMP_PERIOD_S      = 2.00f;
constexpr float    CHIRP_F0_HZ        = 0.30f;
constexpr float    CHIRP_F1_HZ        = 8.00f;  // stay under HX711 usable BW
constexpr float    CHIRP_DUR_S        = 3.00f;  // set to your motor burn time
