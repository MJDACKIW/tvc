#pragma once

#include <stdint.h>

// The bookkeeping around a bounded I2C transaction (i2c_bus.cpp does the hardware part). Pure
// logic with no Arduino in it, so firmware/test/test_i2c_guard runs the very code the flight
// loop uses.
//
// Why this exists: the Teensy 4 Wire library cannot be given a timeout. It waits up to 16 ms
// for the bus to go idle and up to 50 ms for a transaction to finish (WireIMXRT.cpp lines 83,
// 139, 196), per call, with no way to change either. A bus that is stuck, or whose clock pin
// is not connected to the I2C peripheral, therefore costs one tick tens of milliseconds. The
// rules here keep the whole I2C part of one 300 Hz tick under 1 ms whatever the buses do:
//  - one shared budget per tick (kTickBudgetUs): a transaction may run until the budget's
//    deadline and not past it, and is not started at all if too little is left;
//  - a bus that failed is left alone for kBackoffUs, so a dead bus costs at most one budget
//    in every 20 ms, not one in every tick;
//  - failures are counted per bus, so a bus that hangs shows up in STATUS instead of silently
//    returning stale data.
namespace tvc::flight::i2c {

enum class Bus : uint8_t {
    kImu = 0,       // Wire,  LPI2C1, SDA 18 / SCL 19, MPU-6050
    kEncPitch = 1,  // Wire1, LPI2C3, SDA 17 / SCL 16, AS5600 #1
    kEncYaw = 2,    // Wire2, LPI2C4, SDA 25 / SCL 24, AS5600 #2
};
constexpr int kBusCount = 3;

enum class Result : uint8_t {
    kOk,
    kNack,      // the device did not acknowledge: absent, or not at that address
    kTimeout,   // did not finish before the tick's deadline
    kError,     // arbitration lost, FIFO error, or the hardware pin-low timeout
    kBackoff,   // not tried: this bus failed recently and is being left alone
    kNoBudget,  // not tried: too little of this tick's I2C time was left
};

constexpr uint32_t kTickBudgetUs = 1000;  // all I2C in one tick, together
constexpr uint32_t kMinStartUs = 150;      // do not start a transaction with less than this left
constexpr uint32_t kBackoffUs = 20000;     // leave a failed bus alone for this long

// The time one tick may spend on I2C: a shared deadline.
class TickBudget {
public:
    void begin(uint32_t now_us) { deadline_us_ = now_us + kTickBudgetUs; }
    int32_t remainingUs(uint32_t now_us) const {
        return static_cast<int32_t>(deadline_us_ - now_us);
    }
    uint32_t deadlineUs() const { return deadline_us_; }
    bool mayStart(uint32_t now_us) const {
        return remainingUs(now_us) >= static_cast<int32_t>(kMinStartUs);
    }

private:
    uint32_t deadline_us_ = 0;
};

// One per bus: back-off after a failure, and the failure counts.
class Guard {
public:
    bool mayStart(uint32_t now_us) const {
        return !backing_off_ || static_cast<uint32_t>(now_us - failed_at_us_) >= kBackoffUs;
    }

    // Folds a finished transaction into the state. kBackoff and kNoBudget (never tried) are
    // counted as skips and change nothing else.
    void record(Result r, uint32_t now_us) {
        switch (r) {
            case Result::kOk:
                backing_off_ = false;
                break;
            case Result::kTimeout:
                timeouts_++;
                fail(now_us);
                break;
            case Result::kNack:
            case Result::kError:
                errors_++;
                fail(now_us);
                break;
            case Result::kBackoff:
            case Result::kNoBudget:
                skipped_++;
                break;
        }
    }

    uint32_t timeouts() const { return timeouts_; }
    uint32_t errors() const { return errors_; }  // NACKs and hardware errors
    uint32_t skipped() const { return skipped_; }
    void clearCounts() { timeouts_ = errors_ = skipped_ = 0; }

private:
    void fail(uint32_t now_us) {
        backing_off_ = true;
        failed_at_us_ = now_us;
    }

    bool backing_off_ = false;
    uint32_t failed_at_us_ = 0;
    uint32_t timeouts_ = 0, errors_ = 0, skipped_ = 0;
};

}  // namespace tvc::flight::i2c
