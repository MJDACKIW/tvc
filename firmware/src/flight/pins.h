#pragma once

// Teensy 4.1 pin map. SPEC.md Section 4.5 is authoritative; legacy/ configs under an older
// map are superseded (SPEC.md Section 10). Do not hand-edit without updating SPEC.md too.
namespace tvc::pins {

constexpr int kServoX = 3;    // channel B -> moment about X_b (pitch)
constexpr int kServoY = 5;    // channel A -> moment about Y_b (yaw)
constexpr int kBuzzer = 4;

constexpr int kHxDout = 6;    // single HX711, summed across the four load cells (two diagonal
constexpr int kHxSck = 7;     // pairs) upstream in analog hardware -- one logical channel here.

// Reserved for the radio phase; not driven by this build.
constexpr int kRfm95Irq = 8;
constexpr int kRfm95Rst = 9;    // moved from 24 to free Wire2 for the second encoder
constexpr int kRfm95Cs = 10;
// SPI0 MOSI/MISO/SCK = 11/12/13, fixed by silicon, reserved alongside the above.

constexpr int kPyroGate = 14;
constexpr int kArmSense = 15;   // INPUT_PULLUP, switch to GND
constexpr int kPyroCont = A14;  // = pin 38. Final resolution of the A2/Wire1 conflict (SPEC 4.5).

// Wire  (MPU-6050, 0x68):  SDA 18 / SCL 19 (fixed default bus)
// Wire1 (AS5600 #1, 0x36): SDA 17 / SCL 16
// Wire2 (AS5600 #2, 0x36): SDA 25 / SCL 24
constexpr uint8_t kMpuAddr = 0x68;
constexpr uint8_t kAs5600Addr = 0x36;

constexpr int kLedPowerImuOk = 20;
constexpr int kLedArmed = 21;
constexpr int kLedTvcActive = 22;

constexpr int kModeJumperA = 26;  // INPUT_PULLUP; read-only this phase, mode is fixed STAND_OPEN
constexpr int kModeJumperB = 27;

}  // namespace tvc::pins
