#pragma once

#include <stdint.h>

// Byte layout of the MPU-6050 burst read that imu.cpp issues from ACCEL_XOUT_H (0x3B): 14 bytes,
// seven big-endian int16 words: accel x, y, z, temperature, gyro x, y, z. Pure, no Arduino, so
// firmware/test/test_imu_decode checks it on the host (the bench is the only other place a
// byte-order slip would show).
namespace tvc::flight::imu {

constexpr int kBurstBytes = 14;

inline int16_t beWord(const uint8_t* raw, int word_index) {
    return static_cast<int16_t>((static_cast<uint16_t>(raw[2 * word_index]) << 8) |
                                raw[2 * word_index + 1]);
}

// Splits one burst into the sensor-frame accel and gyro counts; the temperature is dropped.
inline void decodeBurst(const uint8_t raw[kBurstBytes], int16_t accel[3], int16_t gyro[3]) {
    for (int i = 0; i < 3; i++) accel[i] = beWord(raw, i);
    for (int i = 0; i < 3; i++) gyro[i] = beWord(raw, 4 + i);  // word 3 is the temperature
}

}  // namespace tvc::flight::imu
