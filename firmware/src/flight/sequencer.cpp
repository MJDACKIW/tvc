#include "sequencer.h"

#include <Arduino.h>
#include <math.h>

#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "loadcell.h"
#include "logging_flash.h"
#include "logging_sd.h"
#include "params.h"
#include "pins.h"
#include "profiles.h"
#include "servo_drive.h"

namespace tvc::flight::sequencer {
namespace {

// ---- timing constants -------------------------------------------------------------------
// SPEC.md 4.3 gives these directly (not physical calibration, so not params.yaml MEASURE):
constexpr uint32_t kCountdownMs = 10000;       // "10 s count"
constexpr uint32_t kFireConfirmWindowMs = 2000; // "waits up to 2 s for ... FIRE_CONFIRM"
constexpr uint32_t kTailoffMs = 1500;            // "TAILOFF (1.5 s)"
constexpr uint32_t kBurnTimeoutMs = 6000;         // "6 s timeout"
// Carried over unchanged from legacy/Radio test/src/static_fire.cpp (config.h); SPEC.md does
// not give replacement numbers for these, only the structure of the checks that use them.
constexpr float kThrustDetectN = 2.0f;
constexpr uint32_t kNoIgnitionMs = 1500;
constexpr float kBurnoutThrustN = 1.0f;
constexpr uint32_t kGyroBiasCalMs = 2000;

// ---- state --------------------------------------------------------------------------------
State g_state = State::kBoot;
constexpr Mode kFixedMode = Mode::kStandOpen;  // this build implements STAND_OPEN only
Profile g_profile = Profile::kHold;
bool g_dry_run = false;
uint16_t g_fault_bits = 0;

uint32_t g_t_state_ms = 0;
uint32_t g_t_ignite_us = 0;
uint32_t g_t_below_ms = 0;   // 0 = "not currently below threshold"
uint32_t g_countdown_start_ms = 0;
uint16_t g_fire_token = 0;
bool g_fire_confirmed = false;

uint32_t g_loop_overrun_count = 0;
uint32_t g_tick_count = 0;  // for the 300 Hz / 150 Hz split

float g_ol_pitch_delta = 0.0f, g_ol_yaw_delta = 0.0f;  // open-loop actuator state (applied)
estimator::ShadowOut g_shadow_p{}, g_shadow_y{};        // last shadow controller output

// NOT IMPLEMENTED: SPEC 4.4's 100 ms hardware watchdog ("the ISR sets the pyro pin LOW before
// reset"). The Teensyduino version PlatformIO resolved here (1.60 / core 5.1.0) does not
// bundle a watchdog library, and no verified one exists in the PlatformIO registry under
// this name. The MCU's RTWDOG peripheral is reachable directly via imxrt.h registers
// (WDOG3_CS/CNT/TOVAL/WIN + an unlock/refresh magic-value sequence), but hand-deriving that
// sequence from memory with no way to verify it against real silicon in this session is
// exactly the kind of unverified safety-relevant guess CLAUDE.md/SPEC.md ask to be flagged
// rather than shipped quietly -- a subtly wrong unlock sequence typically fails silently
// (watchdog just never arms), which is worse than an honest gap. Flagged prominently in the
// session summary; the more critical property SPEC 4.4 actually needs -- the pyro pulse
// width staying bounded even if this loop stalls -- is still independently covered below by
// g_pyro_off_timer, a real hardware timer (IntervalTimer) that does not depend on this.
IntervalTimer g_pyro_off_timer;

// ---- small helpers --------------------------------------------------------------------
bool armSwitchClosed() { return digitalReadFast(pins::kArmSense) == LOW; }

int continuityRaw12() { return analogRead(pins::kPyroCont); }

bool continuityOk() {
    int v = continuityRaw12();
    return v >= params::pyro::continuity_ok_min_adc && v <= params::pyro::continuity_ok_max_adc;
}

void pyroOff() { digitalWriteFast(pins::kPyroGate, LOW); }

void pyroOffISR() {
    digitalWriteFast(pins::kPyroGate, LOW);
    g_pyro_off_timer.end();
}

// Hardware-timer-driven, not loop-driven (SPEC 4.4): the gate comes back down on schedule
// even if the main loop stalls.
void pyroFire() {
    digitalWriteFast(pins::kPyroGate, HIGH);
    g_pyro_off_timer.begin(pyroOffISR, params::pyro::pulse_ms * 1000UL);
}

void setLeds() {
    digitalWriteFast(pins::kLedArmed,
                      (g_state >= State::kArmed && g_state <= State::kBurn) ? ((millis() >> 7) & 1) : 0);
    digitalWriteFast(pins::kLedTvcActive, g_state == State::kBurn ? HIGH : LOW);
}

void enter(State s) {
    g_state = s;
    g_t_state_ms = millis();
    Serial.printf(">> %s\n", stateName(s));
}

// STAND_OPEN's own required-MEASURE set: only what this mode actually needs to operate
// correctly (servo calibration to move to a real angle; load-cell scale so thrust_N and the
// ignition/burnout thrust gates mean something). kp/kd/kalman.q/r are deliberately excluded --
// STAND_OPEN never closes the loop, and gating a plant-ID run on the very numbers that run is
// meant to help derive would be circular. See the session summary for the full reasoning.
bool measureGateOk(bool print_missing) {
    struct Item { const char* name; float value; };
    const Item items[] = {
        {"servo.us_per_deg_x", params::servo::us_per_deg_x},
        {"servo.us_per_deg_y", params::servo::us_per_deg_y},
        {"servo.neutral_us_x", params::servo::neutral_us_x},
        {"servo.neutral_us_y", params::servo::neutral_us_y},
        {"loadcell.hx_scale_counts_per_n", params::loadcell::hx_scale_counts_per_n},
    };
    bool ok = true;
    for (const auto& it : items) {
        if (isnan(it.value)) {
            ok = false;
            if (print_missing) Serial.printf("  MISSING (still MEASURE): %s\n", it.name);
        }
    }
    return ok;
}

void buildAndLogRecord(uint32_t loop_us) {
    Record r{};
    r.t_us = micros();
    r.state = static_cast<uint8_t>(g_state);
    r.mode = static_cast<uint8_t>(kFixedMode);
    r.fault_bits = g_fault_bits;
    r.thrust_N = loadcell::thrustN();
    const int16_t* gy = imu::gyroRawCounts();
    const int16_t* ac = imu::accelRawCounts();
    for (int i = 0; i < 3; i++) { r.gyro[i] = gy[i]; r.accel[i] = ac[i]; }
    r.xhat_p = g_shadow_p.x_hat;
    r.xhat_y = g_shadow_y.x_hat;
    r.u_p = g_shadow_p.u_cmd;
    r.u_y = g_shadow_y.u_cmd;
    r.delta_p = g_ol_pitch_delta;  // actually-applied, open-loop command (SPEC 3.3 role)
    r.delta_y = g_ol_yaw_delta;
    r.enc_p = encoders::pitchDeg();
    r.enc_y = encoders::yawDeg();
    r.K_p = g_shadow_p.K;
    r.K_y = g_shadow_y.K;
    r.servo_us_p = servo_drive::lastUsPitch();
    r.servo_us_y = servo_drive::lastUsYaw();
    r.pyro = digitalReadFast(pins::kPyroGate);
    r.cont_adc8 = static_cast<uint8_t>(continuityRaw12() >> 4);  // 12-bit -> 8-bit
    r.loop_us = static_cast<uint16_t>(loop_us > 0xFFFFu ? 0xFFFFu : loop_us);
    finalizeRecordCrc(r);

    logging_sd::push(r);
    logging_flash::push(r);
}

void abortInternal(const char* why) {
    pyroOff();
    g_pyro_off_timer.end();
    g_ol_pitch_delta = g_ol_yaw_delta = 0.0f;
    servo_drive::center();
    noTone(pins::kBuzzer);
    Serial.printf("!! ABORT: %s\n", why);
    enter(State::kSafe);
}

void runSelftest() {
    enter(State::kSelftest);
    g_fault_bits = 0;
    bool hard_fault = false;

    Serial.println(F("--- SELFTEST ---"));

    if (imu::frameDeterminant() != 1 && imu::frameDeterminant() != -1) {
        g_fault_bits |= fault_bits::kFrameDetInvalid;
        hard_fault = true;
        Serial.println(F("[FAIL] R_B<-S determinant"));
    } else {
        Serial.println(F("[PASS] R_B<-S determinant"));
    }

    imu::begin();
    if (!imu::selftestWhoAmI()) {
        g_fault_bits |= fault_bits::kImu;
        hard_fault = true;
        Serial.println(F("[FAIL] IMU WHO_AM_I"));
    } else {
        Serial.println(F("[PASS] IMU WHO_AM_I"));
        Serial.println(F("       sampling gyro bias, keep the stand still..."));
        imu::calibrateGyroBias(kGyroBiasCalMs);
        digitalWriteFast(pins::kLedPowerImuOk, HIGH);
    }

    if (!logging_sd::begin(kFixedMode, g_profile)) {
        g_fault_bits |= fault_bits::kSd;
        hard_fault = true;
        Serial.println(F("[FAIL] SD card (mount/open/preallocate)"));
    } else {
        Serial.printf("[PASS] SD -> %s\n", logging_sd::filename());
    }

    if (!logging_flash::begin()) {
        g_fault_bits |= fault_bits::kFlash;
        hard_fault = true;
        Serial.println(F("[FAIL] LittleFS flash mirror mount"));
    } else {
        Serial.println(F("[PASS] LittleFS flash mirror mounted"));
    }

    if (!loadcell::tare()) {
        g_fault_bits |= fault_bits::kHx711;
        hard_fault = true;
        Serial.println(F("[FAIL] HX711 tare (never asserted DOUT ready)"));
    } else {
        Serial.println(F("[PASS] HX711 tare"));
    }

    if (!encoders::selftestPresence()) {
        g_fault_bits |= fault_bits::kEncoder;
        hard_fault = true;
        Serial.println(F("[FAIL] encoder I2C presence"));
    } else {
        encoders::zero();
        Serial.println(F("[PASS] encoders present, neutral captured"));
    }

    // Servo sweep to +-5 deg with encoder confirmation (SPEC 4.3). Direction is checked
    // against params::servo::sign_x/y, since that is what defines "commanded direction".
    if (!hard_fault) {
        bool servo_ok = true;
        auto sweepCheck = [&](float target_deg, bool is_pitch) {
            servo_drive::writeDeg(is_pitch ? target_deg : 0.0f, is_pitch ? 0.0f : target_deg);
            delay(400);
            encoders::read();
            float measured = is_pitch ? encoders::pitchDeg() : encoders::yawDeg();
            float sign = is_pitch ? static_cast<float>(params::servo::sign_x)
                                   : static_cast<float>(params::servo::sign_y);
            float expected_dir = sign * target_deg;
            bool moved_right_way = (expected_dir > 0) ? (measured > 1.0f)
                                                        : (expected_dir < 0 ? (measured < -1.0f) : true);
            if (!moved_right_way) servo_ok = false;
        };
        sweepCheck(5.0f, true);
        sweepCheck(-5.0f, true);
        servo_drive::writeDeg(0.0f, 0.0f);
        sweepCheck(5.0f, false);
        sweepCheck(-5.0f, false);
        servo_drive::center();

        if (!servo_ok) {
            g_fault_bits |= fault_bits::kServoSelftest;
            hard_fault = true;
            Serial.println(F("[FAIL] servo sweep: encoder did not confirm commanded direction"));
        } else {
            Serial.println(F("[PASS] servo sweep +-5 deg, both axes"));
        }
    } else {
        Serial.println(F("[SKIP] servo sweep (earlier hard fault)"));
    }

    bool measure_ok = measureGateOk(/*print_missing=*/true);
    if (!measure_ok) {
        g_fault_bits |= fault_bits::kMeasureMissing;
        Serial.println(F("[WARN] MEASURE params above are missing -- ARM will be refused until"));
        Serial.println(F("       params.yaml is filled in and gen_params.py is re-run. SELFTEST"));
        Serial.println(F("       otherwise continues so the stand can still be exercised by hand."));
    } else {
        Serial.println(F("[PASS] all STAND_OPEN MEASURE params present"));
    }

    if (hard_fault) {
        Serial.println(F("--- SELFTEST: FAULT (see failures above) ---"));
        enter(State::kFault);
        return;
    }

    Serial.println(F("--- SELFTEST: complete ---"));
    enter(State::kIdle);
}

}  // namespace

void begin() {
    analogReadResolution(12);

    pinMode(pins::kPyroGate, OUTPUT);
    pyroOff();  // before anything else touches this pin (SPEC 4.4)
    pinMode(pins::kArmSense, INPUT_PULLUP);
    pinMode(pins::kBuzzer, OUTPUT);
    pinMode(pins::kLedPowerImuOk, OUTPUT);
    pinMode(pins::kLedArmed, OUTPUT);
    pinMode(pins::kLedTvcActive, OUTPUT);
    pinMode(pins::kModeJumperA, INPUT_PULLUP);
    pinMode(pins::kModeJumperB, INPUT_PULLUP);

    loadcell::begin();
    encoders::begin();
    servo_drive::begin();
    estimator::init();

    Serial.printf("mode jumpers read %d%d -- mode is fixed to STAND_OPEN this build\n",
                  digitalRead(pins::kModeJumperA), digitalRead(pins::kModeJumperB));

    runSelftest();
}

void tick(uint32_t loop_us) {
    if (loop_us > 3300) {
        g_fault_bits |= fault_bits::kLoopOverrun;
        g_loop_overrun_count++;
    }

    imu::read();
    loadcell::service();
    encoders::read();

    const uint32_t now = millis();

    switch (g_state) {
        case State::kIdle:
            break;

        case State::kArmed: {
            if (!armSwitchClosed()) { g_fault_bits |= fault_bits::kArmSwitch; abortInternal("arm switch opened"); break; }
            float tilt_p = imu::pitchTiltDeg(), tilt_y = imu::yawTiltDeg();
            if (fabsf(tilt_p) > params::staging::prelaunch_tilt_abort_deg ||
                fabsf(tilt_y) > params::staging::prelaunch_tilt_abort_deg) {
                abortInternal("pre-launch tilt exceeded");
            }
            break;
        }

        case State::kCountdown: {
            if (!armSwitchClosed()) { g_fault_bits |= fault_bits::kArmSwitch; abortInternal("arm switch opened"); break; }
            uint32_t el = now - g_countdown_start_ms;
            if (el < kCountdownMs) {
                static uint32_t last_beep = 0;
                uint32_t rem_s = (kCountdownMs - el) / 1000;
                if (now - last_beep > (rem_s < 3 ? 250u : 1000u)) {
                    last_beep = now;
                    tone(pins::kBuzzer, rem_s < 3 ? 2200 : 1400, 80);
                    Serial.printf("T-%lu\n", static_cast<unsigned long>(rem_s) + 1);
                }
                break;
            }
            // T-0 reached: wait up to kFireConfirmWindowMs for FIRE_CONFIRM(token).
            if (g_fire_confirmed) {
                if (!continuityOk()) { g_fault_bits |= fault_bits::kContinuity; abortInternal("igniter continuity lost"); break; }
                pyroFire();
                g_t_ignite_us = micros();
                enter(State::kIgnite);
            } else if (el - kCountdownMs >= kFireConfirmWindowMs) {
                abortInternal("no FIRE_CONFIRM within window");
            }
            break;
        }

        case State::kIgnite: {
            if (g_dry_run) {
                enter(State::kBurn);  // no real thrust to confirm ignition against
                break;
            }
            if (loadcell::thrustN() >= kThrustDetectN) {
                pyroOff();
                g_pyro_off_timer.end();
                enter(State::kBurn);
            } else if (micros() - g_t_ignite_us >= kNoIgnitionMs * 1000UL) {
                g_fault_bits |= fault_bits::kNoIgnition;
                abortInternal("no thrust detected -- assume misfire, WAIT 60 s before approach");
            }
            break;
        }

        case State::kBurn: {
            float t = (micros() - g_t_ignite_us) * 1e-6f;
            // Every exit path below jumps the actuator straight to center (not a slewed
            // return) and resets the open-loop state to match, so the next 150 Hz sub-step
            // doesn't spend several ticks computing a stale, unapplied slew-toward-zero from
            // whatever g_ol_*_delta held mid-profile (harmless -- writeDeg() is only called in
            // BURN/IGNITE -- but it would otherwise show up as a brief, misleading blip in the
            // logged delta_p/delta_y right at the TAILOFF transition).
            if (g_dry_run) {
                if (t >= params::motor::burn_time_s) {
                    servo_drive::center();
                    g_ol_pitch_delta = g_ol_yaw_delta = 0.0f;
                    enter(State::kTailoff);
                }
                break;
            }
            if ((now - g_t_state_ms) >= kBurnTimeoutMs) {
                servo_drive::center();
                g_ol_pitch_delta = g_ol_yaw_delta = 0.0f;
                enter(State::kTailoff);
                break;
            }
            if (loadcell::thrustN() < kBurnoutThrustN) {
                if (g_t_below_ms == 0) g_t_below_ms = now;
                else if (now - g_t_below_ms >= static_cast<uint32_t>(params::staging::burnout_window_s * 1000)) {
                    servo_drive::center();
                    g_ol_pitch_delta = g_ol_yaw_delta = 0.0f;
                    enter(State::kTailoff);
                }
            } else {
                g_t_below_ms = 0;
            }
            break;
        }

        case State::kTailoff:
            if (now - g_t_state_ms >= kTailoffMs) {
                logging_sd::close();
                logging_flash::close();
                Serial.printf("peak %.2f N\n", loadcell::peakThrustN());
                enter(State::kSafe);
            }
            break;

        case State::kSafe:
        case State::kFault:
            pyroOff();
            break;

        default:
            break;
    }

    setLeds();

    // Control-rate (150 Hz) sub-step: shadow estimator + open-loop profile + servo write.
    g_tick_count++;
    if (g_tick_count % 2 == 0) {
        const float dt = 1.0f / params::control::rate_hz;

        g_shadow_p = estimator::stepPitch(imu::gyroDegS(0), imu::pitchTiltDeg(), imu::accelGateOk());
        g_shadow_y = estimator::stepYaw(imu::gyroDegS(1), imu::yawTiltDeg(), imu::accelGateOk());

        float target_p = 0.0f, target_y = 0.0f;
        if (g_state == State::kBurn) profiles::targetAngles(g_profile, (micros() - g_t_ignite_us) * 1e-6f, target_p, target_y);

        const float step = params::servo::slew_deg_per_s * dt;
        float dp = target_p - g_ol_pitch_delta, dyaw = target_y - g_ol_yaw_delta;
        g_ol_pitch_delta += (fabsf(dp) <= step) ? dp : (dp > 0 ? step : -step);
        g_ol_yaw_delta += (fabsf(dyaw) <= step) ? dyaw : (dyaw > 0 ? step : -step);

        if (g_state == State::kBurn || g_state == State::kIgnite) {
            servo_drive::writeDeg(g_ol_pitch_delta, g_ol_yaw_delta);
        }
    }

    buildAndLogRecord(loop_us);
}

// ---- console-invoked commands --------------------------------------------------------
void cmdSelftest() {
    if (g_state != State::kIdle && g_state != State::kFault) {
        Serial.println(F("SELFTEST only from IDLE or FAULT"));
        return;
    }
    runSelftest();
}

void cmdArm() {
    if (g_state != State::kIdle) { Serial.println(F("ARM only from IDLE")); return; }
    if (!measureGateOk(/*print_missing=*/true)) {
        Serial.println(F("ARM refused: MEASURE params above are still unset"));
        return;
    }
    if (!armSwitchClosed()) { Serial.println(F("ARM refused: close the physical arm switch first")); return; }
    if (!continuityOk()) { Serial.println(F("ARM refused: no igniter continuity")); return; }
    loadcell::resetPeak();
    enter(State::kArmed);
}

void cmdCountdown() {
    if (g_state != State::kArmed) { Serial.println(F("COUNTDOWN only from ARMED")); return; }
    g_fire_confirmed = false;
    g_fire_token = static_cast<uint16_t>(random(1, 65536));
    g_countdown_start_ms = millis();
    enter(State::kCountdown);
    Serial.printf("fire_token = %u  (send: FIRE_CONFIRM %u  within %lu ms of T-0)\n", g_fire_token,
                  g_fire_token, static_cast<unsigned long>(kFireConfirmWindowMs));
}

void cmdFireConfirm(uint16_t token) {
    if (g_state != State::kCountdown) { Serial.println(F("FIRE_CONFIRM only during COUNTDOWN")); return; }
    if (token != g_fire_token) { Serial.println(F("FIRE_CONFIRM: token mismatch")); return; }
    g_fire_confirmed = true;
    Serial.println(F("fire token confirmed"));
}

void cmdAbort(const char* why) { abortInternal(why); }

void cmdSetProfile(uint8_t index) {
    if (g_state != State::kIdle) { Serial.println(F("PROFILE only from IDLE")); return; }
    if (index >= static_cast<uint8_t>(Profile::kCount)) { Serial.println(F("PROFILE index out of range")); return; }
    g_profile = static_cast<Profile>(index);
    Serial.printf("profile = %s\n", profileName(g_profile));
}

void cmdSetDryRun(bool on) {
    if (g_state != State::kIdle) { Serial.println(F("DRY_RUN only from IDLE")); return; }
    g_dry_run = on;
    Serial.printf("dry_run = %s\n", on ? "ON (igniter output must be an LED, not a real igniter)" : "off");
}

void cmdTare() {
    if (g_state != State::kIdle) { Serial.println(F("TARE only from IDLE")); return; }
    bool ok = loadcell::tare();
    encoders::zero();
    Serial.printf("tare: load cell %s, encoders zeroed\n", ok ? "ok" : "FAILED (check HX711 wiring)");
}

void cmdResetToIdle() {
    if (g_state != State::kSafe) { Serial.println(F("reset only from SAFE")); return; }
    enter(State::kIdle);
}

State state() { return g_state; }
Mode mode() { return kFixedMode; }
Profile profile() { return g_profile; }
bool dryRun() { return g_dry_run; }
uint16_t faultBits() { return g_fault_bits; }

void printStatus() {
    Serial.printf(
        "[%s] mode=STAND_OPEN profile=%s dry_run=%d thrust=%.2fN enc=(%.2f,%.2f)deg "
        "arm_sw=%s cont=%s fault_bits=0x%04X overruns=%lu\n",
        stateName(g_state), profileName(g_profile), g_dry_run, loadcell::thrustN(),
        encoders::pitchDeg(), encoders::yawDeg(), armSwitchClosed() ? "CLOSED" : "open",
        continuityOk() ? "OK" : "OPEN", g_fault_bits,
        static_cast<unsigned long>(g_loop_overrun_count));
}

}  // namespace tvc::flight::sequencer
