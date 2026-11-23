#pragma once

#include <stddef.h>
#include <stdint.h>

#include "params.h"
#include "protocol.h"

// The LoRa link, in one place, for both ends (SPEC.md Section 4.7 and 4.8).
//
//  - configure() applies the radio settings from the generated params.h. The vehicle
//    (firmware/src/flight/radio.cpp) and the Feather bridge (firmware/src/feather_bridge/)
//    both call it, so the two sides cannot disagree about frequency, SF, BW, CR, power or
//    preamble: there is no second copy of those calls to edit.
//  - timeOnAirUs() is the Semtech SX127x time-on-air formula (datasheet Section 4.1.1.7) on
//    the same numbers. radio.cpp derives its stuck-TX timeout from it and SELFTEST prints the
//    airtime budget from it; tools/lora_airtime.py is the same formula for the Mac, and both
//    are pinned to the same hand-checked values by tests.
//
// It takes the driver as a template parameter so this header needs no RadioHead include and
// the native tests can compile it.
namespace tvc::protocol::lora {

// RadioHead prepends 4 bytes (to, from, id, flags) to every payload it sends, so the PHY
// payload is the protocol frame plus this, not the frame alone.
constexpr uint8_t kRadioHeadHeaderBytes = 4;

// RH_RF95 initialises 8 preamble symbols, explicit header, payload CRC on. configure() sets
// the preamble explicitly so the airtime formula below is not relying on a library default.
constexpr uint16_t kPreambleSymbols = 8;

constexpr int kSf = params::telemetry::lora_sf;
constexpr long kBwHz = static_cast<long>(params::telemetry::lora_bw_khz) * 1000L;
constexpr int kCrDenominator = params::telemetry::lora_cr;  // 5..8 means coding rate 4/5..4/8

static_assert(kSf >= 7 && kSf <= 12, "telemetry.lora_sf must be 7..12");
static_assert(kBwHz == 125000L || kBwHz == 250000L || kBwHz == 500000L,
              "telemetry.lora_bw_khz must be 125, 250 or 500");
static_assert(kCrDenominator >= 5 && kCrDenominator <= 8, "telemetry.lora_cr must be 5..8");

// Applies the shared settings after the driver's own init(). Works on any RadioHead RH_RF95.
template <typename Rf95>
void configure(Rf95& rf95) {
    rf95.setFrequency(params::telemetry::lora_freq_mhz);
    rf95.setTxPower(static_cast<int8_t>(params::telemetry::tx_power_dbm), false);
    rf95.setSpreadingFactor(static_cast<uint8_t>(kSf));
    rf95.setSignalBandwidth(kBwHz);
    rf95.setCodingRate4(static_cast<uint8_t>(kCrDenominator));
    rf95.setPreambleLength(kPreambleSymbols);
}

// Time on air in microseconds for a PHY payload of phy_bytes, integer math so it is exact and
// usable in constant expressions. Explicit header, payload CRC on, as RH_RF95 configures it.
// Low-data-rate optimisation is on when a symbol lasts more than 16 ms, which is what
// RadioHead's own setLowDatarate() decides.
//
// Written as single-expression constexpr functions on purpose: the Feather bridge env may
// compile this header as C++11, which has no local variables in constexpr bodies.
constexpr uint64_t symbolNs(int sf, long bw_hz) {
    return (static_cast<uint64_t>(1) << sf) * 1000000000ULL / static_cast<uint64_t>(bw_hz);
}

constexpr bool lowDataRate(int sf, long bw_hz) { return symbolNs(sf, bw_hz) > 16000000ULL; }

constexpr int64_t ceilDivPositive(int64_t n, int64_t d) { return n > 0 ? (n + d - 1) / d : 0; }

constexpr uint64_t payloadSymbols(uint32_t phy_bytes, int sf, long bw_hz, int cr_denominator) {
    return 8 + static_cast<uint64_t>(ceilDivPositive(
                   8 * static_cast<int64_t>(phy_bytes) - 4 * sf + 28 + 16,
                   4 * (sf - (lowDataRate(sf, bw_hz) ? 2 : 0)))) *
                   static_cast<uint64_t>(cr_denominator);
}

constexpr uint32_t timeOnAirUs(uint32_t phy_bytes, int sf, long bw_hz, int cr_denominator,
                               uint32_t preamble_symbols = kPreambleSymbols) {
    // The preamble lasts (n + 4.25) symbols, i.e. (4n + 17) / 4.
    return static_cast<uint32_t>(
        ((4 * static_cast<uint64_t>(preamble_symbols) + 17) * symbolNs(sf, bw_hz) / 4 +
         payloadSymbols(phy_bytes, sf, bw_hz, cr_denominator) * symbolNs(sf, bw_hz) + 500) /
        1000);
}

constexpr uint32_t timeOnAirUs(uint32_t phy_bytes) {
    return timeOnAirUs(phy_bytes, kSf, kBwHz, kCrDenominator);
}

// Airtime of one protocol frame (sync, len, type, payload, crc16) as it goes out through
// RadioHead, i.e. including RadioHead's own header.
constexpr uint32_t frameTimeOnAirUs(uint8_t payload_len) {
    return timeOnAirUs(kRadioHeadHeaderBytes + tvc::protocol::frameLen(payload_len));
}

// The downlink budget (SPEC.md Section 4.7): TELEM at telem_rate_hz plus STATUS at
// status_rate_hz, as thousandths of each second the vehicle transmitter is on the air. EVENT
// frames (3 copies each) and the 2 Hz HEARTBEAT uplink come on top.
constexpr uint32_t kTelemAirUs = frameTimeOnAirUs(sizeof(tvc::protocol::TelemPayload));
constexpr uint32_t kStatusAirUs = frameTimeOnAirUs(sizeof(tvc::protocol::StatusPayload));
constexpr uint32_t kEventAirUs = frameTimeOnAirUs(sizeof(tvc::protocol::EventPayload));
constexpr uint32_t kDownlinkDutyPermille =
    (static_cast<uint32_t>(params::telemetry::telem_rate_hz) * kTelemAirUs +
     static_cast<uint32_t>(params::telemetry::status_rate_hz) * kStatusAirUs) / 1000;

// ---- how the downlink is laid out in time (firmware/src/flight/tx_slots.h) ------------------
//
// TELEM owns a fixed grid, one slot per 1/telem_rate_hz. STATUS and EVENT frames go in the gap
// after a TELEM frame, never across the next TELEM slot. The sizes below are what that
// scheduler needs, all derived from the same airtime numbers.

static_assert(params::telemetry::status_rate_hz >= 1 &&
                  params::telemetry::telem_rate_hz % params::telemetry::status_rate_hz == 0,
              "telemetry.telem_rate_hz must be a whole multiple of status_rate_hz: STATUS rides "
              "on every Nth TELEM slot");

constexpr uint32_t kTelemPeriodMs = 1000u / params::telemetry::telem_rate_hz;
constexpr uint32_t kStatusEveryNTelem =
    static_cast<uint32_t>(params::telemetry::telem_rate_hz / params::telemetry::status_rate_hz);

// One tick of the 300 Hz loop: a frame can be noticed as finished, or started, only on a tick.
constexpr uint32_t kLoopTickUs = 1000000u / params::control::loop_hz;

// Loading RadioHead's FIFO before the transmitter starts: about 40 bytes over SPI at the
// library's default 1 MHz, so roughly half a millisecond. An estimate from the bus speed, not a
// measurement; it only widens the margins below.
constexpr uint32_t kTxStartCostUs = 500;

constexpr uint32_t ceilMs(uint32_t us) { return (us + 999) / 1000; }

// STATUS falls due this long after its TELEM slot, i.e. once a TELEM that started on time would
// have finished.
constexpr uint32_t kStatusOffsetMs = ceilMs(kTxStartCostUs + kTelemAirUs);
// What a STATUS or EVENT frame occupies the transmitter for, rounded up and with 1 ms spare
// because the scheduler counts in whole milliseconds.
constexpr uint32_t kStatusCostMs = ceilMs(kTxStartCostUs + kStatusAirUs) + 1;
constexpr uint32_t kEventCostMs = ceilMs(kTxStartCostUs + kEventAirUs) + 1;

}  // namespace tvc::protocol::lora
