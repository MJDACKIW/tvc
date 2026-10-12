// Writes a SYNTHETIC SD-primary log (SPEC.md Section 4.6: LogHeader + 300 Hz Records) of one
// complete STAND_OPEN / STEP_X dry sequence, IDLE -> ARMED -> COUNTDOWN -> IGNITE -> BURN ->
// TAILOFF, for exercising tools/decode_log.py and ground/gs.py --replay before a real bench log
// exists. Same approach as the decoder-verification harness from the STAND_OPEN firmware
// session: a native build against the REAL firmware/src/flight/record.h (Record, crc16), so
// the bytes on disk are the firmware's own layout. It also links the real core/ controller and
// the real firmware profiles.cpp, so the shadow estimator/PID and the open-loop profile are the
// flight code's, not re-implementations. Built and run by tools/synth_sd_log.py.
//
// Everything physical here (stand tilt, noise levels, vibration, igniter delay, servo lag) is
// made up to look plausible. It is not measured and not the sim. Values params.yaml still has
// as MEASURE are replaced by the kSynth* placeholders below, for this file only.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "attitude_from_accel.h"
#include "controller.h"
#include "params.h"
#include "profiles.h"
#include "record.h"

using namespace tvc;
using namespace tvc::flight;

#pragma pack(push, 1)
struct LogHeader {  // mirrors firmware/src/flight/logging_sd.cpp's LogHeader exactly
    uint32_t magic;
    uint32_t schema;
    char params_hash[65];
    char git_hash[20];
    uint8_t mode;
    uint8_t profile;
    uint8_t reserved0[2];
    uint32_t boot_count;
    uint32_t record_size;
};
#pragma pack(pop)
static_assert(sizeof(LogHeader) == 105, "LogHeader drifted from logging_sd.cpp");

namespace {

// Placeholders for params.yaml MEASURE values, synthetic file only. Kp/Kd are the paper
// simulation's (simulation/config.py KP, KD); q/r and the servo scale are picked for this file.
constexpr float kSynthKp = 8.5f;
constexpr float kSynthKd = 1.2f;
constexpr float kSynthQ = 0.005f;
constexpr float kSynthR = 0.5f;
constexpr float kSynthUsPerDeg = 10.0f;
constexpr float kSynthNeutralUs = 1500.0f;

// Test-procedure timeline, seconds since the first record.
constexpr double kArmAt = 4.0;
constexpr double kCountdownAt = 7.0;
constexpr double kFireConfirmAt = 15.0;  // T-2 s
// sequencer.cpp constants (SPEC 4.3 and the legacy static_fire.cpp thresholds it carries).
constexpr double kCountdownS = 10.0;
constexpr double kTailoffS = 1.5;
constexpr double kBurnTimeoutS = 6.0;
constexpr float kThrustDetectN = 2.0f;
constexpr float kBurnoutThrustN = 1.0f;

// Made-up plant/sensor behaviour.
constexpr double kIgniterDelayS = 0.12;
constexpr float kStandPitchDeg = 0.6f, kStandYawDeg = -0.4f;  // imperfectly levelled stand
constexpr float kServoTauS = 0.03f;
constexpr float kEncLsbDeg = 360.0f / 4096.0f;  // AS5600 12-bit
constexpr float kGyroLsbPerDps = 16.4f, kAccelLsbPerG = 2048.0f;  // imu.cpp
constexpr double kHx711Hz = 80.0;

struct Curve {
    std::vector<double> t, f;
    double at(double x) const {
        if (x <= 0.0 || t.empty() || x >= t.back()) return 0.0;
        for (size_t i = 1; i < t.size(); i++) {
            if (x <= t[i]) return f[i - 1] + (f[i] - f[i - 1]) * (x - t[i - 1]) / (t[i] - t[i - 1]);
        }
        return 0.0;
    }
};

Curve loadEng(const char* path) {
    Curve c;
    c.t.push_back(0.0);
    c.f.push_back(0.0);
    std::ifstream in(path);
    std::string line;
    bool header_seen = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == ';') continue;
        if (!header_seen) { header_seen = true; continue; }
        std::istringstream ss(line);
        double t, f;
        if (ss >> t >> f) { c.t.push_back(t); c.f.push_back(f); }
    }
    return c;
}

int16_t sat16(double v) {
    v = std::round(v);
    return static_cast<int16_t>(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <out.BIN> <estes_e12.eng>\n", argv[0]);
        return 2;
    }
    Curve thrust = loadEng(argv[2]);
    if (thrust.t.size() < 3) {
        std::fprintf(stderr, "could not read thrust curve %s\n", argv[2]);
        return 1;
    }
    FILE* f = std::fopen(argv[1], "wb");
    if (!f) { std::perror(argv[1]); return 1; }

    const Profile profile = Profile::kStepX;
    LogHeader h{};
    h.magic = 0x4C435654;  // "TVCL"
    h.schema = 1;
    std::memcpy(h.params_hash, params::kParamsHash, sizeof(h.params_hash) - 1);
    std::strncpy(h.git_hash, "0000000-synthetic", sizeof(h.git_hash) - 1);
    h.mode = static_cast<uint8_t>(Mode::kStandOpen);
    h.profile = static_cast<uint8_t>(profile);
    h.boot_count = 7;
    h.record_size = sizeof(Record);
    std::fwrite(&h, sizeof(h), 1, f);

    std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);
    auto N = [&](double sigma) { return sigma * n01(rng); };

    const ControlParams cp{1.0f / params::control::rate_hz, kSynthKp, params::control::ki, kSynthKd,
                           params::control::integral_clamp_deg_s, params::control::max_deflection_deg,
                           kSynthQ, kSynthR, params::servo::slew_deg_per_s};
    AxisState ax_p{}, ax_y{};
    ax_p.P = ax_y.P = params::kalman::p0;
    AxisOut sh_p{}, sh_y{};

    State state = State::kIdle;
    double t_state = 0, t_countdown = 0, t_ignite = -1, t_below = -1;
    bool fire_confirmed = false, pyro = false;
    uint16_t fault = 0;
    float ol_p = 0, ol_y = 0;        // open-loop applied command (sequencer's g_ol_*_delta)
    float mech_p = 0, mech_y = 0;    // gimbal's physical angle (lags the command)
    double thrust_meas = 0, next_hx = 0;
    const uint32_t t0_us = 1250000;
    uint32_t prev_us = t0_us - 3333;
    int n = 0;
    const double dt_loop = 1.0 / params::control::loop_hz;

    for (int i = 0;; i++) {
        const double t = i * dt_loop;
        const double t_motor = (t_ignite >= 0) ? t - t_ignite - kIgniterDelayS : -1.0;
        const double f_true = thrust.at(t_motor);

        if (t >= next_hx) {  // HX711 ~80 SPS; loadcell::thrustN() holds the last conversion
            next_hx += 1.0 / kHx711Hz;
            thrust_meas = f_true + N(0.12);
        }

        // ---- state machine, same transitions and thresholds as sequencer.cpp tick() ----
        bool stop = false;
        switch (state) {
            case State::kIdle:
                if (t >= kArmAt) { state = State::kArmed; t_state = t; }
                break;
            case State::kArmed:
                if (t >= kCountdownAt) { state = State::kCountdown; t_state = t_countdown = t; }
                break;
            case State::kCountdown:
                if (t >= kFireConfirmAt) fire_confirmed = true;
                if (t - t_countdown >= kCountdownS && fire_confirmed) {
                    pyro = true;
                    t_ignite = t;
                    state = State::kIgnite;
                    t_state = t;
                }
                break;
            case State::kIgnite:
                if (thrust_meas >= kThrustDetectN) { pyro = false; state = State::kBurn; t_state = t; }
                break;
            case State::kBurn:
                if (t - t_state >= kBurnTimeoutS) { ol_p = ol_y = 0; state = State::kTailoff; t_state = t; break; }
                if (thrust_meas < kBurnoutThrustN) {
                    if (t_below < 0) t_below = t;
                    else if (t - t_below >= params::staging::burnout_window_s) {
                        ol_p = ol_y = 0;
                        state = State::kTailoff;
                        t_state = t;
                    }
                } else {
                    t_below = -1;
                }
                break;
            case State::kTailoff:
                // sequencer.cpp closes the SD file on this transition, so SAFE is never logged.
                if (t - t_state >= kTailoffS) stop = true;
                break;
            default:
                break;
        }
        if (stop) break;
        if (pyro && t - t_ignite >= params::pyro::pulse_ms / 1000.0) pyro = false;  // IntervalTimer

        // ---- sensors: fixed stand, vibration proportional to thrust, mostly axial (Z_b) ----
        const double vib = std::max(0.0, f_true);
        const double th = kStandPitchDeg * M_PI / 180.0, ps = kStandYawDeg * M_PI / 180.0;
        const double a_g[3] = {-std::sin(ps) + N(0.004 + 0.002 * vib),
                               std::sin(th) * std::cos(ps) + N(0.004 + 0.002 * vib),
                               std::cos(th) * std::cos(ps) + N(0.004 + 0.012 * vib)};
        const double w_dps[3] = {N(0.12 + 0.06 * vib) + 0.03, N(0.12 + 0.06 * vib) - 0.02,
                                 N(0.10 + 0.08 * vib)};
        Record r{};
        for (int k = 0; k < 3; k++) {
            r.gyro[k] = sat16(w_dps[k] * kGyroLsbPerDps);
            r.accel[k] = sat16(a_g[k] * kAccelLsbPerG);
        }
        const float ax = r.accel[0] / kAccelLsbPerG, ay = r.accel[1] / kAccelLsbPerG,
                    az = r.accel[2] / kAccelLsbPerG;

        // ---- 150 Hz sub-step: shadow estimator + open-loop profile (sequencer.cpp) ----
        if (i % 2 == 1) {
            const bool gate = accel_gate_ok(ax, ay, az, params::kalman::accel_gate_g[0],
                                            params::kalman::accel_gate_g[1]);
            sh_p = controller_step(ax_p, r.gyro[0] / kGyroLsbPerDps, accel_tilt_pitch_deg(ay, az), gate, cp);
            sh_y = controller_step(ax_y, r.gyro[1] / kGyroLsbPerDps, accel_tilt_yaw_deg(ax, az), gate, cp);
            float target_p = 0, target_y = 0;
            if (state == State::kBurn)
                profiles::targetAngles(profile, static_cast<float>(t - t_ignite), target_p, target_y);
            const float step = params::servo::slew_deg_per_s * cp.dt;
            const float dp = target_p - ol_p, dy = target_y - ol_y;
            ol_p += std::fabs(dp) <= step ? dp : (dp > 0 ? step : -step);
            ol_y += std::fabs(dy) <= step ? dy : (dy > 0 ? step : -step);
        }
        const bool driving = (state == State::kBurn || state == State::kIgnite);
        const float cmd_p = driving ? ol_p : 0.0f, cmd_y = driving ? ol_y : 0.0f;
        mech_p += (cmd_p - mech_p) * static_cast<float>(dt_loop / kServoTauS);
        mech_y += (cmd_y - mech_y) * static_cast<float>(dt_loop / kServoTauS);

        // ---- the Record, field by field as buildAndLogRecord() fills it ----
        uint32_t t_us = t0_us + static_cast<uint32_t>(std::lround(t * 1e6 + N(12.0)));
        const bool overrun = (state == State::kBurn && t - t_state < dt_loop);  // one SD-flush stall
        if (overrun) t_us += 85;
        const uint32_t loop_us = t_us - prev_us;
        if (loop_us > 3300) fault |= fault_bits::kLoopOverrun;
        prev_us = t_us;

        r.t_us = t_us;
        r.state = static_cast<uint8_t>(state);
        r.mode = h.mode;
        r.fault_bits = fault;
        r.thrust_N = static_cast<float>(thrust_meas);
        r.xhat_p = sh_p.x_hat;
        r.xhat_y = sh_y.x_hat;
        r.u_p = sh_p.u_cmd;
        r.u_y = sh_y.u_cmd;
        r.delta_p = ol_p;
        r.delta_y = ol_y;
        r.enc_p = std::round((mech_p + N(0.03)) / kEncLsbDeg) * kEncLsbDeg;
        r.enc_y = std::round((mech_y + N(0.03)) / kEncLsbDeg) * kEncLsbDeg;
        r.K_p = sh_p.K;
        r.K_y = sh_y.K;
        r.servo_us_p = static_cast<uint16_t>(std::lround(kSynthNeutralUs + kSynthUsPerDeg * cmd_p));
        r.servo_us_y = static_cast<uint16_t>(std::lround(kSynthNeutralUs + kSynthUsPerDeg * cmd_y));
        r.pyro = pyro ? 1 : 0;
        // Igniter bridge intact until it burns through shortly after the gate goes high.
        const bool igniter_open = t_ignite >= 0 && t - t_ignite > 0.25;
        r.cont_adc8 = static_cast<uint8_t>(igniter_open ? 0 : 115 + static_cast<int>(N(1.0)));
        r.loop_us = static_cast<uint16_t>(loop_us > 0xFFFF ? 0xFFFF : loop_us);
        finalizeRecordCrc(r);
        if (i == 500) r.crc16 ^= 0xFFFF;  // one deliberately corrupt record (IDLE), as before
        std::fwrite(&r, sizeof(r), 1, f);
        n++;
    }

    std::fclose(f);
    std::printf("wrote %s: %zu-byte header + %d records x %zu bytes (%.2f s)\n", argv[1],
                sizeof(h), n, sizeof(Record), n * dt_loop);
    return 0;
}
