#pragma once

#include <stddef.h>
#include <stdint.h>

// Telemetry wire protocol, shared by firmware/src/flight/ (vehicle radio + USB console are
// separate; only the radio side speaks this), firmware/src/feather_bridge/ (relays raw frames,
// does not need the structs below -- see kSyncByte0/1, kHeaderLen, kCrcLen), and the one
// Python-side codec, protocol/telem_codec.py (used by tools/telem_cli.py, ground/, and
// sim/run_sim.py --serve; ground/tests/test_telem_codec.py checks it against this header).
// SPEC.md Section 4.7 is authoritative; this header must match it exactly, including field
// order.
//
// Frame: 0xAA 0x55 | len | type | payload[len] | crc16, little-endian throughout. `len` is
// payload bytes only (max kMaxPayload). `crc16` is CRC-16/CCITT-FALSE (poly 0x1021, init
// 0xFFFF -- same as firmware/src/flight/record.h's crc16(), which protects Record instead)
// computed over len++type++payload, so a corrupted length or type is caught too, not just a
// corrupted payload.
namespace tvc::protocol {

constexpr uint8_t kSyncByte0 = 0xAA;
constexpr uint8_t kSyncByte1 = 0x55;
constexpr uint8_t kMaxPayload = 48;
constexpr size_t kHeaderLen = 4;  // sync0, sync1, len, type
constexpr size_t kCrcLen = 2;
// Total on-wire frame size for a given payload length (does not include the Feather bridge's
// own 2 trailing RSSI/SNR bytes -- see feather_bridge/main.cpp -- which are outside this frame,
// appended only on the USB side of a downlink relay).
constexpr size_t frameLen(uint8_t payload_len) { return kHeaderLen + payload_len + kCrcLen; }

enum class PacketType : uint8_t {
    // Downlink
    kTelem = 1,
    kStatus = 2,
    kEvent = 3,
    kAck = 4,
    // Uplink
    kHeartbeat = 0x10,
    kSetMode = 0x11,
    kSetProfile = 0x12,
    kTare = 0x13,
    kArm = 0x14,
    kDisarm = 0x15,
    kCountdown = 0x16,
    kFireConfirm = 0x17,
    kAbort = 0x18,
    kDryRun = 0x19,
    kPing = 0x1A,
    kDumpFlash = 0x1B,  // USB only, per SPEC 4.7
};

// CRC-16/CCITT-FALSE, identical algorithm to record.h's crc16() (deliberately not shared code
// across the two headers -- protocol/ and firmware/src/flight/ stay independent of each other
// so feather_bridge, which only needs protocol.h, never has to pull in flight/ headers).
inline uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

#pragma pack(push, 1)

// TELEM (SPEC.md Section 4.7). 27 bytes.
struct TelemPayload {
    int16_t t_ms;        // ms since ignition; negative pre-launch
    uint8_t state;       // tvc::flight::State (record.h)
    int16_t thrust_x10;  // N * 10
    int16_t xhat_p_x100; // deg * 100
    int16_t xhat_y_x100;
    int16_t u_p_x100;    // deg * 100
    int16_t u_y_x100;
    int16_t enc_p_x100;  // deg * 100
    int16_t enc_y_x100;
    int16_t gyro_z_x10;  // deg/s * 10
    uint16_t k_p_x1000;  // Kalman gain * 1000, 0..1 -> 0..1000
    uint16_t k_y_x1000;
    uint8_t flags;       // see telem_flags below
    uint8_t cont_adc8;   // 12-bit continuity ADC >> 4
    uint16_t batt_mv;
};
static_assert(sizeof(TelemPayload) == 27, "TelemPayload size changed -- update SPEC.md 4.7");

namespace telem_flags {
constexpr uint8_t kPitchGateOk = 1u << 0;
constexpr uint8_t kYawGateOk = 1u << 1;
constexpr uint8_t kPitchSaturated = 1u << 2;
constexpr uint8_t kYawSaturated = 1u << 3;
constexpr uint8_t kTvcActive = 1u << 4;
constexpr uint8_t kDryRun = 1u << 5;
// Physical arm switch (ARM_SENSE_PIN) reads closed. Added for the ground GUI (SPEC.md Section
// 5: ARM "requires vehicle to report arm switch closed"), which had no field to read it from;
// took a previously reserved bit, so the payload size and every other field are unchanged.
constexpr uint8_t kArmSwitchClosed = 1u << 6;
}  // namespace telem_flags

// STATUS (SPEC.md Section 4.7). 42 bytes. Every cumulative field here is computed and
// integrated on the vehicle, not the ground station, so it stays correct across packet loss.
struct StatusPayload {
    uint8_t mode;
    uint16_t fault_bits;
    uint8_t params_hash6[6];       // first 6 bytes of SHA-256(params.yaml)
    uint32_t sd_bytes_written;
    uint32_t flash_bytes_written;
    int8_t rssi_last_uplink_dbm;
    uint16_t boot_count;
    uint32_t missing_measure_bits; // bit i <-> params::measure_bits::* index i (core/measure_bits.json)
    uint16_t loop_overrun_count;
    uint8_t sat_pct_p;              // 0..100
    uint8_t sat_pct_y;
    uint16_t running_iae_p_x100;    // integral of |x_hat_p| dt since boot, deg*s * 100
    uint16_t running_iae_y_x100;
    uint16_t peak_thrust_x10;       // N * 10
    uint16_t peak_tilt_p_x100;      // deg * 100
    uint16_t peak_tilt_y_x100;
    uint8_t fw_git_hash[4];         // first 4 bytes of kFirmwareGitHash's hex prefix
};
static_assert(sizeof(StatusPayload) == 42, "StatusPayload size changed -- update SPEC.md 4.7");

// EVENT (SPEC.md Section 4.7). 5 bytes. event_code values: protocol/event_codes.h/.json
// (generated by tools/gen_protocol.py from protocol/event_codes.yaml).
struct EventPayload {
    int16_t t_ms;
    uint8_t event_code;
    int16_t aux;
};
static_assert(sizeof(EventPayload) == 5, "EventPayload size changed -- update SPEC.md 4.7");

// ---- Uplink (SPEC.md Section 4.7) ----------------------------------------------------------
// Every uplink payload starts with a uint16 `seq`, chosen by the ground station (incrementing
// per command, wrapping), and every uplink is answered with one ACK carrying that seq back.
//   HEARTBEAT, TARE, ARM, DISARM, COUNTDOWN, ABORT, PING, DUMP_FLASH -> UplinkPayload (2 B)
//   SET_MODE (Mode), SET_PROFILE (Profile), DRY_RUN (0/1)            -> UplinkU8Payload (3 B)
//   FIRE_CONFIRM                                                      -> FireConfirmPayload (4 B)
// DISARM returns ARMED -> IDLE, and SAFE -> IDLE (the USB console's RESET).
struct UplinkPayload {
    uint16_t seq;
};
static_assert(sizeof(UplinkPayload) == 2, "UplinkPayload size changed -- update SPEC.md 4.7");

struct UplinkU8Payload {
    uint16_t seq;
    uint8_t value;
};
static_assert(sizeof(UplinkU8Payload) == 3, "UplinkU8Payload size changed -- update SPEC.md 4.7");

struct FireConfirmPayload {
    uint16_t seq;
    uint16_t token;  // the fire_token returned in COUNTDOWN's ACK.aux
};
static_assert(sizeof(FireConfirmPayload) == 4, "FireConfirmPayload size changed -- update SPEC.md 4.7");

enum class AckResult : uint8_t {
    kOk = 0,
    kRejectedState = 1,       // not allowed in the current State
    kRejectedArmSwitch = 2,   // physical arm switch open
    kRejectedMeasure = 3,     // a MEASURE param this mode needs is still NaN
    kRejectedToken = 4,       // FIRE_CONFIRM token mismatch
    kRejectedArg = 5,         // out-of-range mode/profile/etc.
    kRejectedContinuity = 6,  // igniter continuity out of range
    kUnsupported = 7,         // command (or selected mode) not implemented by this vehicle build
    kRejectedHeartbeat = 8,   // ARM with no HEARTBEAT in the last pyro.heartbeat_timeout_ms
};

// ACK (downlink, 6 bytes). aux: COUNTDOWN -> fire_token (uint16 bit pattern); 0 otherwise.
struct AckPayload {
    uint16_t seq;
    uint8_t cmd_type;  // the PacketType being acknowledged
    uint8_t result;    // AckResult
    uint16_t aux;
};
static_assert(sizeof(AckPayload) == 6, "AckPayload size changed -- update SPEC.md 4.7");

#pragma pack(pop)

}  // namespace tvc::protocol
