#pragma once

#include <stdint.h>

// Timing of the 300 Hz loop, measured two separate ways per tick, plus the rule that the
// overrun counter has always used. Pure logic with no Arduino in it, so
// firmware/test/test_loop_stats runs this very code.
//
//   execution time  end of the tick body minus its start (micros() before and after
//                   sequencer::tick()). What one tick costs. Must stay under the tick period
//                   or the loop cannot keep its rate.
//   period          start of this tick minus start of the previous one, as micros() saw it.
//                   What the control loop actually runs at. About 3333 us when healthy, and
//                   about the execution time when the loop is overloaded.
//   behind          how far past its scheduled time the tick started (the loop_us the Record
//                   logs, minus one tick period). Zero-ish when healthy. When ticks cost more
//                   than a period it grows without bound, because the schedule only retires
//                   one period per tick: it is a backlog, not a one-off stall.
//
// The first tick after boot or after reset() is ignored for every maximum, the histogram and the
// count: it has no previous start to measure a period against, and its timestamps come from
// before the stats were cleared.
namespace tvc::flight {

// The overrun rule exactly as sequencer.cpp has always applied it (SPEC.md Section 4.2 says "any
// tick exceeds 3.3 ms"). loop_us here is the time since the tick's scheduled start, which is
// never less than one tick period (3333 us) for a tick that has just been started, so this is
// true for every tick; see BRINGUP.md "What loop_overrun_count counts". It lives here, unchanged,
// so a test can pin that down. The fault bit's meaning is not changed.
inline bool countsAsOverrun(uint32_t loop_us) { return loop_us > 3300; }

class LoopStats {
public:
    static constexpr int kBins = 6;
    // Upper edges (exclusive) of the first five execution-time bins, microseconds; the sixth bin
    // is everything at or above the last edge. 3333 is one tick period at 300 Hz.
    static constexpr uint32_t kEdgesUs[kBins - 1] = {500, 1000, 2000, 3333, 10000};

    LoopStats() { reset(); }

    void reset() {
        first_ = true;
        ticks_ = 0;
        exec_max_us_ = period_max_us_ = behind_max_us_ = 0;
        for (auto& h : hist_) h = 0;
    }

    // start_us/end_us: micros() at the start and end of the tick body (wraparound is fine).
    void record(uint32_t start_us, uint32_t end_us, uint32_t behind_us) {
        if (first_) {
            first_ = false;
            last_start_us_ = start_us;
            return;
        }
        const uint32_t exec_us = end_us - start_us;
        const uint32_t period_us = start_us - last_start_us_;
        last_start_us_ = start_us;
        ticks_++;
        if (exec_us > exec_max_us_) exec_max_us_ = exec_us;
        if (period_us > period_max_us_) period_max_us_ = period_us;
        if (behind_us > behind_max_us_) behind_max_us_ = behind_us;
        hist_[bin(exec_us)]++;
    }

    static int bin(uint32_t exec_us) {
        int b = 0;
        while (b < kBins - 1 && exec_us >= kEdgesUs[b]) b++;
        return b;
    }

    uint32_t ticks() const { return ticks_; }
    uint32_t execMaxUs() const { return exec_max_us_; }
    uint32_t periodMaxUs() const { return period_max_us_; }
    uint32_t behindMaxUs() const { return behind_max_us_; }
    uint32_t hist(int i) const { return hist_[i]; }

private:
    bool first_ = true;
    uint32_t last_start_us_ = 0;
    uint32_t ticks_ = 0;
    uint32_t exec_max_us_ = 0, period_max_us_ = 0, behind_max_us_ = 0;
    uint32_t hist_[kBins] = {};
};

}  // namespace tvc::flight
