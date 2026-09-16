#pragma once

#include <stdint.h>

// MPU-6050 driver: wake/configure, WHO_AM_I, gyro bias cal, burst read with the R_(B<-S)
// body-frame remap (SPEC.md Section 4.5) applied to raw counts before anything else touches
// them, per attitude_from_accel.h's header comment ("that remap is a firmware/sim concern").
namespace tvc::flight::imu {

// TODO(HANDOFF.md): identity is a placeholder. Derive the real signed permutation from the
// gravity-based procedure in HANDOFF.md before trusting logged attitude data (SPEC 4.5, 10).
// Rows/cols order is body (x,y,z) <- sensor (x,y,z).
inline constexpr int8_t kFrameBodyFromSensor[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};

// 3x3 signed-permutation determinant; SELFTEST refuses ARMED unless this is exactly +-1.
int frameDeterminant();

void begin();                 // Wire.begin() + wake/configure registers
uint8_t whoAmI();              // reads register 0x75; MPU-6050 returns 0x68
bool selftestWhoAmI();          // whoAmI() == kMpuAddr

// Blocking: averages raw gyro for duration_ms (vehicle must be still) and stores the bias
// subtracted by every subsequent read(). Call once during SELFTEST.
void calibrateGyroBias(uint32_t duration_ms);

// Burst-reads accel+gyro, applies kFrameBodyFromSensor, subtracts gyro bias. Updates the
// values returned by the accessors below.
void read();

const int16_t* gyroRawCounts();   // body frame, post-remap, bias-corrected, for Record
const int16_t* accelRawCounts();  // body frame, post-remap, for Record

float gyroDegS(int axis);   // 0=X_b, 1=Y_b, 2=Z_b
float accelG(int axis);

float pitchTiltDeg();  // atan2(ay, az), core/attitude_from_accel.h
float yawTiltDeg();    // atan2(-ax, az)
bool accelGateOk();     // |a| within params::kalman::accel_gate_g

}  // namespace tvc::flight::imu
