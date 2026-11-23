#pragma once

#include <stdint.h>

// Which frame, if any, the vehicle starts transmitting on a given loop tick (SPEC.md Section
// 4.7). Pure logic with no Arduino or RadioHead in it, so firmware/test/test_tx_slots runs the
// very code telemetry.cpp calls against a simulated radio.
//
// The rules, in priority order, with at most one frame started per tick:
//  1. TELEM. Its slot is used up the moment it falls due. If the radio is still sending, that
//     TELEM is skipped and counted, never queued: the next one is a period later.
//  2. EVENT copy, if one is due and the radio is free.
//  3. STATUS, if its slot is due and the radio is free. A STATUS that finds the radio busy does
//     not lose its slot, it stays due and goes on the first free tick. Without that rule it
//     would never be sent: TELEM and STATUS fall due in the same millisecond every whole
//     second, TELEM wins, and the radio is then busy for the rest of the tick.
// Timers advance on a fixed grid, so a late send does not shift the ones after it. After a
// stall longer than a period (SELFTEST holds the loop for seconds at boot) the grid restarts
// from now instead of replaying every missed slot as a burst of frames.
namespace tvc::flight {

enum class TxChoice : uint8_t { kNone, kTelem, kEvent, kStatus };

class TxSlots {
public:
    TxSlots(uint32_t telem_period_ms, uint32_t status_period_ms)
        : telem_period_ms_(telem_period_ms), status_period_ms_(status_period_ms) {}

    // Both timers due immediately; also clears the skipped count.
    void reset(uint32_t now_ms) {
        next_telem_ms_ = now_ms;
        next_status_ms_ = now_ms;
        telem_skipped_ = 0;
    }

    TxChoice choose(uint32_t now_ms, bool radio_busy, bool event_due) {
        if (due(now_ms, next_telem_ms_)) {
            advance(next_telem_ms_, telem_period_ms_, now_ms);
            if (!radio_busy) return TxChoice::kTelem;
            telem_skipped_++;
        }
        if (radio_busy) return TxChoice::kNone;
        if (event_due) return TxChoice::kEvent;
        if (due(now_ms, next_status_ms_)) {
            advance(next_status_ms_, status_period_ms_, now_ms);
            return TxChoice::kStatus;
        }
        return TxChoice::kNone;
    }

    uint32_t telemSkipped() const { return telem_skipped_; }
    void clearSkipped() { telem_skipped_ = 0; }

private:
    static bool due(uint32_t now_ms, uint32_t slot_ms) {
        return static_cast<int32_t>(now_ms - slot_ms) >= 0;
    }

    static void advance(uint32_t& slot_ms, uint32_t period_ms, uint32_t now_ms) {
        slot_ms += period_ms;
        if (due(now_ms, slot_ms)) slot_ms = now_ms + period_ms;
    }

    uint32_t telem_period_ms_;
    uint32_t status_period_ms_;
    uint32_t next_telem_ms_ = 0;
    uint32_t next_status_ms_ = 0;
    uint32_t telem_skipped_ = 0;
};

}  // namespace tvc::flight
