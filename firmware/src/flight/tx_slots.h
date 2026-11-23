#pragma once

#include <stdint.h>

// Which frame, if any, the vehicle starts transmitting on a given loop tick (SPEC.md Section
// 4.7). Pure logic with no Arduino or RadioHead in it, so firmware/test/test_tx_slots runs the
// very code telemetry.cpp calls against a simulated radio.
//
// The layout, with at most one frame started per tick:
//  - TELEM owns a fixed grid of slots, one per period. A slot is used up the moment it falls
//    due. If the radio is still sending then, that TELEM is skipped and counted, never queued.
//  - STATUS rides on every Nth TELEM slot at a phase offset: it falls due `status_offset_ms`
//    after the slot, once a TELEM that started on time would have finished. So it starts in
//    the gap after a TELEM frame, not in the same millisecond as the next one.
//  - EVENT copies and STATUS only start when the radio is free AND the frame will have left the
//    air before the next TELEM slot falls due. Otherwise they wait for the next gap, so a
//    TELEM never falls due while one of them is still on the air. (If a frame is longer than
//    the gap can ever be, the check is dropped for it: the link is saturated and skipped
//    TELEMs are then the honest outcome.) A STATUS held back more than half its own period
//    goes out anyway, at the cost of one TELEM, so it cannot be starved indefinitely.
//  - Priority within a tick: TELEM, then an EVENT copy, then STATUS.
// Timers advance on a fixed grid, so a late send does not shift the ones after it. After a
// stall longer than a period (SELFTEST holds the loop for seconds at boot) the grid restarts
// from now instead of replaying every missed slot as a burst of frames, and STATUS keeps its
// phase because it is derived from the TELEM slot, not from a timer of its own.
namespace tvc::flight {

enum class TxChoice : uint8_t { kNone, kTelem, kEvent, kStatus };

struct TxPlan {
    uint32_t telem_period_ms;
    uint32_t status_every_n;    // a STATUS follows every Nth TELEM slot
    uint32_t status_offset_ms;  // STATUS falls due this long after its TELEM slot
    uint32_t status_cost_ms;    // how long a STATUS occupies the transmitter, start-up included
    uint32_t event_cost_ms;     // same, for one EVENT copy
};

class TxSlots {
public:
    explicit TxSlots(const TxPlan& plan) : plan_(plan) {}

    // The first TELEM slot is due immediately, and so is the STATUS behind it (plus its
    // offset). Also clears the skipped count.
    void reset(uint32_t now_ms) {
        next_telem_ms_ = now_ms;
        slot_count_ = plan_.status_every_n - 1;  // the first slot carries a STATUS
        status_pending_ = false;
        status_due_ms_ = now_ms;
        telem_skipped_ = 0;
    }

    TxChoice choose(uint32_t now_ms, bool radio_busy, bool event_due) {
        if (due(now_ms, next_telem_ms_)) {
            // A whole period or more behind: restart the grid from now, do not replay slots.
            const uint32_t slot_ms =
                due(now_ms, next_telem_ms_ + plan_.telem_period_ms) ? now_ms : next_telem_ms_;
            next_telem_ms_ = slot_ms + plan_.telem_period_ms;
            if (++slot_count_ >= plan_.status_every_n) {
                slot_count_ = 0;
                status_pending_ = true;  // a STATUS still waiting from last time is replaced
                status_due_ms_ = slot_ms + plan_.status_offset_ms;
            }
            if (!radio_busy) return TxChoice::kTelem;
            telem_skipped_++;
        }
        if (radio_busy) return TxChoice::kNone;

        // Milliseconds until the next TELEM slot falls due: how long a frame started now has.
        const int32_t room_ms = static_cast<int32_t>(next_telem_ms_ - now_ms);
        if (event_due && fits(plan_.event_cost_ms, room_ms)) return TxChoice::kEvent;
        if (status_pending_ && due(now_ms, status_due_ms_)) {
            const bool overdue = static_cast<int32_t>(now_ms - status_due_ms_) >
                                 static_cast<int32_t>(plan_.telem_period_ms *
                                                      plan_.status_every_n / 2);
            if (overdue || fits(plan_.status_cost_ms, room_ms)) {
                status_pending_ = false;
                return TxChoice::kStatus;
            }
        }
        return TxChoice::kNone;
    }

    uint32_t telemSkipped() const { return telem_skipped_; }
    void clearSkipped() { telem_skipped_ = 0; }

private:
    static bool due(uint32_t now_ms, uint32_t slot_ms) {
        return static_cast<int32_t>(now_ms - slot_ms) >= 0;
    }

    // A frame that can never fit in one period is let through whenever the radio is free.
    bool fits(uint32_t cost_ms, int32_t room_ms) const {
        return cost_ms >= plan_.telem_period_ms || room_ms >= static_cast<int32_t>(cost_ms);
    }

    TxPlan plan_;
    uint32_t next_telem_ms_ = 0;
    uint32_t slot_count_ = 0;
    bool status_pending_ = false;
    uint32_t status_due_ms_ = 0;
    uint32_t telem_skipped_ = 0;
};

}  // namespace tvc::flight
