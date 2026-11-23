#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"

namespace tvc::flight::i2c {
namespace {

TickBudget g_budget;
Guard g_guard[kBusCount];
bool g_in_tick = false;

constexpr int idx(Bus b) { return static_cast<int>(b); }

#ifndef TVC_I2C_LIBRARY_WIRE
// Teensy 4.1: Wire = LPI2C1, Wire1 = LPI2C3, Wire2 = LPI2C4 (WireIMXRT.cpp lines 430-457).
IMXRT_LPI2C_t* portOf(Bus b) {
    switch (b) {
        case Bus::kImu: return reinterpret_cast<IMXRT_LPI2C_t*>(IMXRT_LPI2C1_ADDRESS);
        case Bus::kEncPitch: return reinterpret_cast<IMXRT_LPI2C_t*>(IMXRT_LPI2C3_ADDRESS);
        default: return reinterpret_cast<IMXRT_LPI2C_t*>(IMXRT_LPI2C4_ADDRESS);
    }
}
#endif

TwoWire& wireOf(Bus b) {
    switch (b) {
        case Bus::kImu: return Wire;
        case Bus::kEncPitch: return Wire1;
        default: return Wire2;
    }
}

// With the radio's RST wire on pin 24 (pins.h) the yaw encoder's SCL pin is the radio's reset
// line. Touching Wire2 would hand that pin to the I2C peripheral, whose clock pulses would then
// reset the radio, so that bus is never used and reads on it report "no device".
constexpr bool busUsable(Bus b) { return b != Bus::kEncYaw || !pins::kWire2SharedWithRadioRst; }

#ifndef TVC_I2C_LIBRARY_WIRE
// Stops a transaction that failed or lost the device: empty the FIFOs and ask for a STOP.
void abandon(IMXRT_LPI2C_t* port) {
    port->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;
    port->MTDR = LPI2C_MTDR_CMD_STOP;
}

// A master that timed out may be wedged mid-transaction. Re-running the library's own setup
// resets it (MCR RST, clock, pins) in a few microseconds. This does not free a slave that is
// holding SDA low: that needs the nine-clock recovery Wire does in wait_idle(), which is too
// slow for the tick path, so such a bus just keeps failing, counted, one try per 20 ms.
void resetBus(Bus b) {
    if (!busUsable(b)) return;
    TwoWire& w = wireOf(b);
    w.begin();
    w.setClock(400000);
}
#endif  // !TVC_I2C_LIBRARY_WIRE

#ifdef TVC_I2C_LIBRARY_WIRE
// The library's own blocking transaction: the escape hatch. No deadline.
Result transact(Bus bus, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n, uint32_t) {
    TwoWire& w = wireOf(bus);
    w.beginTransmission(addr);
    w.write(reg);
    const uint8_t e = w.endTransmission(false);
    if (e == 2 || e == 3) return Result::kNack;
    if (e != 0) return Result::kError;
    if (w.requestFrom(static_cast<int>(addr), static_cast<int>(n)) != n) return Result::kError;
    for (uint8_t i = 0; i < n; i++) out[i] = static_cast<uint8_t>(w.read());
    return Result::kOk;
}
#else
// One transaction on the LPI2C master, START, addr+W, reg, repeated START, addr+R, n bytes,
// STOP, queued the way WireIMXRT.cpp queues it (endTransmission(false) then requestFrom) but
// polled against a deadline. Flag handling follows the library: NDF = no ACK, ALF/FEF/PLTF =
// arbitration lost, FIFO error, pin-low timeout; done = every byte read, the command FIFO
// empty and the STOP detected.
Result transact(Bus bus, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n,
                uint32_t deadline_us) {
    IMXRT_LPI2C_t* port = portOf(bus);
    auto expired = [deadline_us]() { return static_cast<int32_t>(deadline_us - micros()) <= 0; };

    // The bus must be idle, or already ours (a previous NACK leaves the master holding it).
    for (;;) {
        const uint32_t s = port->MSR;
        if (!(s & LPI2C_MSR_BBF) || (s & LPI2C_MSR_MBF)) break;
        if (expired()) return Result::kTimeout;
    }
    port->MSR = 0x00007F00;                      // clear stale flags, as the library does
    port->MCR |= LPI2C_MCR_RTF | LPI2C_MCR_RRF;  // start from empty FIFOs

    const uint32_t wr = static_cast<uint32_t>(addr & 0x7F) << 1;
    uint8_t queued = 0, got = 0;
    for (;;) {
        uint32_t used = port->MFSR & 0x07;
        while (queued < 5 && used < 4) {
            switch (queued) {
                case 0: port->MTDR = LPI2C_MTDR_CMD_START | wr; break;
                case 1: port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | reg; break;
                case 2: port->MTDR = LPI2C_MTDR_CMD_START | wr | 1; break;
                case 3: port->MTDR = LPI2C_MTDR_CMD_RECEIVE | static_cast<uint32_t>(n - 1); break;
                default: port->MTDR = LPI2C_MTDR_CMD_STOP; break;
            }
            queued++;
            used++;
        }

        uint32_t rx = (port->MFSR >> 16) & 0x07;
        while (rx > 0 && got < n) {
            out[got++] = static_cast<uint8_t>(port->MRDR);
            rx--;
        }

        const uint32_t s = port->MSR;
        if (s & LPI2C_MSR_NDF) {
            abandon(port);
            return Result::kNack;
        }
        if (s & (LPI2C_MSR_ALF | LPI2C_MSR_FEF | LPI2C_MSR_PLTF)) {
            abandon(port);
            return Result::kError;
        }
        if (got >= n && queued >= 5 && (port->MFSR & 0x07) == 0 && (s & LPI2C_MSR_SDF)) {
            port->MSR = LPI2C_MSR_SDF;
            return Result::kOk;
        }
        if (expired()) return Result::kTimeout;
    }
}
#endif

}  // namespace

void begin() { clearStats(); }

void beginTick() {
    g_budget.begin(micros());
    g_in_tick = true;
}

void endTick() { g_in_tick = false; }

Result readRegisters(Bus bus, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) {
    Guard& guard = g_guard[idx(bus)];
    const uint32_t now = micros();
    if (!g_in_tick) g_budget.begin(now);  // outside the tick every read has its own budget
    Result r;
    if (n == 0) {
        r = Result::kError;
    } else if (!busUsable(bus)) {
        r = Result::kNack;  // no hardware access: see busUsable()
    } else if (!guard.mayStart(now)) {
        r = Result::kBackoff;
    } else if (!g_budget.mayStart(now)) {
        r = Result::kNoBudget;
    } else {
        r = transact(bus, addr, reg, out, n, g_budget.deadlineUs());
#ifndef TVC_I2C_LIBRARY_WIRE
        if (r == Result::kTimeout || r == Result::kError) resetBus(bus);
#endif
    }
    guard.record(r, micros());
    return r;
}

uint32_t timeouts(Bus bus) { return g_guard[idx(bus)].timeouts(); }
uint32_t errors(Bus bus) { return g_guard[idx(bus)].errors(); }

uint32_t skipped() {
    uint32_t n = 0;
    for (const auto& g : g_guard) n += g.skipped();
    return n;
}

void clearStats() {
    for (auto& g : g_guard) g.clearCounts();
}

}  // namespace tvc::flight::i2c
