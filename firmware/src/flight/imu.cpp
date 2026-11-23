#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "attitude_from_accel.h"
#include "i2c_bus.h"
#include "imu_decode.h"
#include "params.h"
#include "pins.h"

namespace tvc::flight::imu {
namespace {

constexpr float kGyroLsbPerDps = 16.4f;    // +-2000 dps full scale
constexpr float kAccelLsbPerG = 2048.0f;   // +-16 g full scale

int16_t g_gyro_raw[3] = {0, 0, 0};
int16_t g_accel_raw[3] = {0, 0, 0};
float g_gyro_bias_raw[3] = {0.0f, 0.0f, 0.0f};

void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(pins::kMpuAddr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void applyFrame(const int16_t in[3], int16_t out[3]) {
    for (int i = 0; i < 3; i++) {
        int32_t s = 0;
        for (int j = 0; j < 3; j++) s += static_cast<int32_t>(kFrameBodyFromSensor[i][j]) * in[j];
        out[i] = static_cast<int16_t>(constrain(s, -32768, 32767));
    }
}

}  // namespace

int frameDeterminant() {
    const auto& m = kFrameBodyFromSensor;
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

void begin() {
    Wire.begin();
    Wire.setClock(400000);
    writeReg(0x6B, 0x00);  // PWR_MGMT_1: wake
    writeReg(0x1B, 0x18);  // GYRO_CONFIG: +-2000 dps
    writeReg(0x1C, 0x18);  // ACCEL_CONFIG: +-16 g
    writeReg(0x1A, 0x02);  // CONFIG: DLPF 94 Hz
}

uint8_t whoAmI() {
    Wire.beginTransmission(pins::kMpuAddr);
    Wire.write(0x75);
    Wire.endTransmission(false);
    if (Wire.requestFrom(static_cast<int>(pins::kMpuAddr), 1) != 1) return 0x00;
    return Wire.read();
}

bool selftestWhoAmI() { return whoAmI() == pins::kMpuAddr; }

void calibrateGyroBias(uint32_t duration_ms) {
    double sum[3] = {0.0, 0.0, 0.0};
    uint32_t n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < duration_ms) {
        if (read()) {
            sum[0] += g_gyro_raw[0] + g_gyro_bias_raw[0];  // undo the bias read() just applied
            sum[1] += g_gyro_raw[1] + g_gyro_bias_raw[1];
            sum[2] += g_gyro_raw[2] + g_gyro_bias_raw[2];
            n++;
        }
        delay(2);
    }
    if (n == 0) return;
    for (int i = 0; i < 3; i++) g_gyro_bias_raw[i] = static_cast<float>(sum[i] / n);
}

bool read() {
    // MPU-6050 burst from ACCEL_XOUT_H (0x3B): imu_decode.h has the byte layout and its test.
    uint8_t raw[kBurstBytes];
    if (i2c::readRegisters(i2c::Bus::kImu, pins::kMpuAddr, 0x3B, raw, sizeof(raw)) !=
        i2c::Result::kOk) {
        return false;
    }
    int16_t a_sensor[3], g_sensor[3];
    decodeBurst(raw, a_sensor, g_sensor);
    applyFrame(a_sensor, g_accel_raw);
    int16_t g_body[3];
    applyFrame(g_sensor, g_body);
    for (int i = 0; i < 3; i++) {
        g_gyro_raw[i] =
            static_cast<int16_t>(constrain(g_body[i] - g_gyro_bias_raw[i], -32768, 32767));
    }
    return true;
}

const int16_t* gyroRawCounts() { return g_gyro_raw; }
const int16_t* accelRawCounts() { return g_accel_raw; }

float gyroDegS(int axis) { return g_gyro_raw[axis] / kGyroLsbPerDps; }
float accelG(int axis) { return g_accel_raw[axis] / kAccelLsbPerG; }

float pitchTiltDeg() { return tvc::accel_tilt_pitch_deg(accelG(1), accelG(2)); }
float yawTiltDeg() { return tvc::accel_tilt_yaw_deg(accelG(0), accelG(2)); }

bool accelGateOk() {
    return tvc::accel_gate_ok(accelG(0), accelG(1), accelG(2), params::kalman::accel_gate_g[0],
                               params::kalman::accel_gate_g[1]);
}

}  // namespace tvc::flight::imu
