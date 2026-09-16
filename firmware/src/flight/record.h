#pragma once

#include <stddef.h>
#include <stdint.h>

// Shared types for the vehicle firmware: sequencer states, modes, open-loop profiles, the
// on-disk Record layout (SPEC.md Section 4.6), and fault bit assignments (not spec'd by
// number in SPEC.md; assigned here and must stay in sync with tools/decode_log.py).
namespace tvc::flight {

enum class State : uint8_t {
    kBoot = 0,
    kSelftest,
    kIdle,
    kArmed,
    kCountdown,
    kIgnite,
    kBurn,
    kTailoff,
    kSafe,
    kFault,
};

inline const char* stateName(State s) {
    static const char* kNames[] = {"BOOT", "SELFTEST", "IDLE",    "ARMED", "COUNTDOWN",
                                    "IGNITE", "BURN",   "TAILOFF", "SAFE",  "FAULT"};
    uint8_t i = static_cast<uint8_t>(s);
    return i < (sizeof(kNames) / sizeof(kNames[0])) ? kNames[i] : "?";
}

// SPEC.md Section 4.1. Only kStandOpen is implemented; the others fault SELFTEST if selected.
enum class Mode : uint8_t {
    kSimOnboard = 0,
    kHil = 1,
    kStandOpen = 2,
    kStandClosed = 3,
    kFlight = 4,
};

enum class Profile : uint8_t {
    kHold = 0,
    kStepX,
    kStepY,
    kRampX,
    kChirpX,
    kCount,
};

inline const char* profileName(Profile p) {
    static const char* kNames[] = {"HOLD", "STEP_X", "STEP_Y", "RAMP_X", "CHIRP_X"};
    uint8_t i = static_cast<uint8_t>(p);
    return i < (sizeof(kNames) / sizeof(kNames[0])) ? kNames[i] : "?";
}

// Bit assignments are a firmware-internal decision (SPEC.md does not enumerate them).
// tools/decode_log.py's FAULT_BITS must be kept in sync with this list.
namespace fault_bits {
constexpr uint16_t kMeasureMissing = 1u << 0;
constexpr uint16_t kFrameDetInvalid = 1u << 1;
constexpr uint16_t kImu = 1u << 2;
constexpr uint16_t kSd = 1u << 3;
constexpr uint16_t kFlash = 1u << 4;
constexpr uint16_t kEncoder = 1u << 5;
constexpr uint16_t kServoSelftest = 1u << 6;
constexpr uint16_t kLoopOverrun = 1u << 7;
constexpr uint16_t kServoRate = 1u << 8;
constexpr uint16_t kArmSwitch = 1u << 9;
constexpr uint16_t kContinuity = 1u << 10;
constexpr uint16_t kNoIgnition = 1u << 11;
constexpr uint16_t kHx711 = 1u << 12;
}  // namespace fault_bits

// SPEC.md Section 4.6. Field list as specified there sums to 74 bytes, not the "64 bytes"
// stated in the same section's prose -- flagged to the user rather than silently dropping
// fields (see the summary at the end of this session). Packed, no padding.
#pragma pack(push, 1)
struct Record {
    uint32_t t_us;
    uint8_t state;
    uint8_t mode;
    uint16_t fault_bits;
    float thrust_N;
    int16_t gyro[3];
    int16_t accel[3];
    float xhat_p, xhat_y;
    float u_p, u_y;
    float delta_p, delta_y;
    float enc_p, enc_y;
    float K_p, K_y;
    uint16_t servo_us_p, servo_us_y;
    uint8_t pyro;
    uint8_t cont_adc8;
    uint16_t loop_us;
    uint16_t crc16;
};
#pragma pack(pop)
static_assert(sizeof(Record) == 74, "Record size changed -- update tools/decode_log.py too");

// CRC-16/CCITT-FALSE over every Record field except crc16 itself.
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

inline void finalizeRecordCrc(Record& r) {
    r.crc16 = 0;
    r.crc16 = crc16(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc16));
}

}  // namespace tvc::flight
