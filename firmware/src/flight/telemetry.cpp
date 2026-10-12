#include "telemetry.h"

#include <Arduino.h>
#include <math.h>

#include "encoders.h"
#include "git_hash.h"
#include "imu.h"
#include "loadcell.h"
#include "logging_flash.h"
#include "logging_sd.h"
#include "params.h"
#include "pins.h"
#include "protocol.h"
#include "radio.h"
#include "sequencer.h"

namespace tvc::flight::telemetry {
namespace {

constexpr uint32_t kTelemPeriodMs = 1000u / params::telemetry::telem_rate_hz;
constexpr uint32_t kStatusPeriodMs = 1000u / params::telemetry::status_rate_hz;
constexpr int kMaxPendingEvents = 6;  // headroom for a few EVENTs' retransmits overlapping

// ---- latest control-tick snapshot (150 Hz), read by the next 10 Hz TELEM send ----
float g_xhat_p = 0, g_xhat_y = 0, g_u_p = 0, g_u_y = 0;
uint16_t g_k_p_x1000 = 0, g_k_y_x1000 = 0;
uint8_t g_gate_sat_flags = 0;  // protocol::telem_flags bits 0-3

// ---- STATUS cumulative accumulators, since boot (SPEC 4.7: correct despite packet loss) ----
double g_iae_p = 0, g_iae_y = 0;  // integral of |x_hat| dt, deg*s
float g_peak_tilt_p = 0, g_peak_tilt_y = 0;
uint32_t g_control_ticks = 0, g_sat_ticks_p = 0, g_sat_ticks_y = 0;

uint32_t g_next_telem_ms = 0, g_next_status_ms = 0;

struct PendingEvent {
    uint32_t due_ms;
    protocol::EventPayload payload;
    bool live;
};
PendingEvent g_pending[kMaxPendingEvents];

int16_t clampToI16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return static_cast<int16_t>(v);
}

uint16_t clampToU16(double v) {
    if (v > 65535.0) return 65535;
    if (v < 0.0) return 0;
    return static_cast<uint16_t>(v);
}

void sendEventPayload(const protocol::EventPayload& payload) {
    radio::send(protocol::PacketType::kEvent, reinterpret_cast<const uint8_t*>(&payload),
                sizeof(payload));
}

// bits[i] matches core/measure_bits.json's entry for the same params::measure_bits::* index,
// both generated from the same measure_list in tools/gen_params.py (SPEC.md Section 2/4.7).
uint32_t missingMeasureBits() {
    uint32_t bits = 0;
    auto mark = [&bits](int bit, float v) {
        if (isnan(v)) bits |= (1u << bit);
    };
    mark(params::measure_bits::control_kp, params::control::kp);
    mark(params::measure_bits::control_kd, params::control::kd);
    mark(params::measure_bits::kalman_q, params::kalman::q);
    mark(params::measure_bits::kalman_r, params::kalman::r);
    mark(params::measure_bits::servo_us_per_deg_x, params::servo::us_per_deg_x);
    mark(params::measure_bits::servo_us_per_deg_y, params::servo::us_per_deg_y);
    mark(params::measure_bits::servo_neutral_us_x, params::servo::neutral_us_x);
    mark(params::measure_bits::servo_neutral_us_y, params::servo::neutral_us_y);
    mark(params::measure_bits::vehicle_mass_kg, params::vehicle::mass_kg);
    mark(params::measure_bits::vehicle_inertia_pitch_kg_m2, params::vehicle::inertia_pitch_kg_m2);
    mark(params::measure_bits::vehicle_inertia_yaw_kg_m2, params::vehicle::inertia_yaw_kg_m2);
    mark(params::measure_bits::vehicle_r_gimbal_to_com_m, params::vehicle::r_gimbal_to_com_m);
    mark(params::measure_bits::vehicle_cn_alpha_per_rad, params::vehicle::cn_alpha_per_rad);
    mark(params::measure_bits::loadcell_hx_scale_counts_per_n,
         params::loadcell::hx_scale_counts_per_n);
    mark(params::measure_bits::power_batt_adc_to_mv, params::power::batt_adc_to_mv);
    return bits;
}

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;  // non-hex (e.g. a truncated/garbled hash) decodes as 0, not a crash
}

void hexPrefixToBytes(const char* hex, uint8_t* out, int n_bytes) {
    for (int i = 0; i < n_bytes; i++) {
        out[i] = static_cast<uint8_t>((hexNibble(hex[2 * i]) << 4) | hexNibble(hex[2 * i + 1]));
    }
}

uint16_t battMv() {
    float scale = params::power::batt_adc_to_mv;
    if (isnan(scale)) return 0;  // uncalibrated; missing_measure_bits already flags this
    return clampToU16(analogRead(pins::kBattSense) * scale);
}

uint8_t continuityAdc8() { return static_cast<uint8_t>(analogRead(pins::kPyroCont) >> 4); }

void sendTelem() {
    State state = sequencer::state();
    protocol::TelemPayload tp{};
    tp.t_ms = clampToI16(sequencer::telemetryTMs());
    tp.state = static_cast<uint8_t>(state);
    tp.thrust_x10 = clampToI16(loadcell::thrustN() * 10.0);
    tp.xhat_p_x100 = clampToI16(g_xhat_p * 100.0);
    tp.xhat_y_x100 = clampToI16(g_xhat_y * 100.0);
    tp.u_p_x100 = clampToI16(g_u_p * 100.0);
    tp.u_y_x100 = clampToI16(g_u_y * 100.0);
    tp.enc_p_x100 = clampToI16(encoders::pitchDeg() * 100.0);
    tp.enc_y_x100 = clampToI16(encoders::yawDeg() * 100.0);
    tp.gyro_z_x10 = clampToI16(imu::gyroDegS(2) * 10.0);
    tp.k_p_x1000 = g_k_p_x1000;
    tp.k_y_x1000 = g_k_y_x1000;
    tp.flags = g_gate_sat_flags;
    if (state == State::kBurn) tp.flags |= protocol::telem_flags::kTvcActive;
    if (sequencer::dryRun()) tp.flags |= protocol::telem_flags::kDryRun;
    if (sequencer::armSwitchClosed()) tp.flags |= protocol::telem_flags::kArmSwitchClosed;
    tp.cont_adc8 = continuityAdc8();
    tp.batt_mv = battMv();
    radio::send(protocol::PacketType::kTelem, reinterpret_cast<const uint8_t*>(&tp), sizeof(tp));
}

void sendStatus() {
    protocol::StatusPayload sp{};
    sp.mode = static_cast<uint8_t>(sequencer::mode());
    sp.fault_bits = sequencer::faultBits();
    hexPrefixToBytes(params::kParamsHash, sp.params_hash6, sizeof(sp.params_hash6));
    sp.sd_bytes_written = logging_sd::bytesWritten();
    sp.flash_bytes_written = logging_flash::bytesWritten();
    sp.rssi_last_uplink_dbm = radio::lastUplinkRssiDbm();
    sp.boot_count = static_cast<uint16_t>(logging_sd::bootCount());
    sp.missing_measure_bits = missingMeasureBits();
    sp.loop_overrun_count = static_cast<uint16_t>(sequencer::loopOverrunCount());
    sp.sat_pct_p = static_cast<uint8_t>(
        g_control_ticks ? (100u * g_sat_ticks_p / g_control_ticks) : 0);
    sp.sat_pct_y = static_cast<uint8_t>(
        g_control_ticks ? (100u * g_sat_ticks_y / g_control_ticks) : 0);
    sp.running_iae_p_x100 = clampToU16(g_iae_p * 100.0);
    sp.running_iae_y_x100 = clampToU16(g_iae_y * 100.0);
    sp.peak_thrust_x10 = clampToU16(loadcell::peakThrustN() * 10.0);
    sp.peak_tilt_p_x100 = clampToU16(g_peak_tilt_p * 100.0);
    sp.peak_tilt_y_x100 = clampToU16(g_peak_tilt_y * 100.0);
    hexPrefixToBytes(kFirmwareGitHash, sp.fw_git_hash, sizeof(sp.fw_git_hash));
    radio::send(protocol::PacketType::kStatus, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

}  // namespace

void init() {
    radio::begin();
    g_xhat_p = g_xhat_y = g_u_p = g_u_y = 0;
    g_k_p_x1000 = g_k_y_x1000 = 0;
    g_gate_sat_flags = 0;
    g_iae_p = g_iae_y = 0;
    g_peak_tilt_p = g_peak_tilt_y = 0;
    g_control_ticks = g_sat_ticks_p = g_sat_ticks_y = 0;
    g_next_telem_ms = g_next_status_ms = millis();
    for (auto& p : g_pending) p.live = false;
}

void onControlTick(const estimator::ShadowOut& pitch, const estimator::ShadowOut& yaw) {
    constexpr float dt = 1.0f / params::control::rate_hz;

    g_xhat_p = pitch.x_hat;
    g_xhat_y = yaw.x_hat;
    g_u_p = pitch.u_cmd;
    g_u_y = yaw.u_cmd;
    g_k_p_x1000 = clampToU16(pitch.K * 1000.0);
    g_k_y_x1000 = clampToU16(yaw.K * 1000.0);

    // Same condition the core PID uses for its own saturated_last (paper Appendix A.3): the
    // clamp changed the value. No core/ changes needed to expose this.
    bool sat_p = (pitch.u_raw != pitch.u_cmd);
    bool sat_y = (yaw.u_raw != yaw.u_cmd);
    g_gate_sat_flags = 0;
    if (pitch.accel_used) g_gate_sat_flags |= protocol::telem_flags::kPitchGateOk;
    if (yaw.accel_used) g_gate_sat_flags |= protocol::telem_flags::kYawGateOk;
    if (sat_p) g_gate_sat_flags |= protocol::telem_flags::kPitchSaturated;
    if (sat_y) g_gate_sat_flags |= protocol::telem_flags::kYawSaturated;

    g_control_ticks++;
    if (sat_p) g_sat_ticks_p++;
    if (sat_y) g_sat_ticks_y++;
    g_iae_p += fabsf(pitch.x_hat) * dt;
    g_iae_y += fabsf(yaw.x_hat) * dt;
    if (fabsf(pitch.x_hat) > g_peak_tilt_p) g_peak_tilt_p = fabsf(pitch.x_hat);
    if (fabsf(yaw.x_hat) > g_peak_tilt_y) g_peak_tilt_y = fabsf(yaw.x_hat);
}

void onLoopTick() {
    radio::pollUplink();

    uint32_t now = millis();

    for (auto& p : g_pending) {
        if (p.live && static_cast<int32_t>(now - p.due_ms) >= 0) {
            sendEventPayload(p.payload);
            p.live = false;
        }
    }

    if (static_cast<int32_t>(now - g_next_telem_ms) >= 0) {
        g_next_telem_ms += kTelemPeriodMs;
        sendTelem();
    }

    if (static_cast<int32_t>(now - g_next_status_ms) >= 0) {
        g_next_status_ms += kStatusPeriodMs;
        sendStatus();
    }
}

void onEvent(uint8_t event_code, int16_t aux) {
    protocol::EventPayload payload{};
    payload.t_ms = clampToI16(sequencer::telemetryTMs());
    payload.event_code = event_code;
    payload.aux = aux;

    sendEventPayload(payload);  // send immediately

    uint32_t now = millis();
    int scheduled = 0;
    for (auto& p : g_pending) {
        if (!p.live && scheduled < 2) {  // 2 retransmits -> 3 total, per SPEC.md Section 4.7
            p.live = true;
            p.payload = payload;
            p.due_ms = now + 150u * (scheduled + 1);
            scheduled++;
        }
    }
}

}  // namespace tvc::flight::telemetry
