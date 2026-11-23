#include <unity.h>

#include <stdint.h>

#include "../../src/flight/tx_slots.h"
#include "lora_link.h"

using tvc::flight::TxChoice;
using tvc::flight::TxPlan;
using tvc::flight::TxSlots;
namespace lora = tvc::protocol::lora;

void setUp(void) {}
void tearDown(void) {}

// ---- a pretend vehicle -----------------------------------------------------------------------
// The loop ticks every 3333 us (300 Hz) and asks TxSlots::choose() what to start, with a radio
// that stays busy for the start-up cost plus the airtime of whatever it was told to send. TELEM
// slots are on the 100 ms grid from t=0, STATUS slots on every 10th, as in the firmware. Sizes
// on the air: TELEM 33 + 4 RadioHead bytes, STATUS 49 + 4, EVENT 11 + 4.
constexpr uint32_t kPeriodMs = 100;
constexpr uint32_t kStartCostUs = lora::kTxStartCostUs;

struct Sim {
    TxPlan plan;
    TxSlots slots;
    uint32_t telem_air_us, status_air_us, event_air_us;
    uint64_t now_us;
    uint64_t tick_us = lora::kLoopTickUs;  // time between ticks: 3333 us when the loop is healthy
    uint64_t busy_until_us = 0;
    bool event_pending = false;
    int telem = 0, status = 0, event = 0;
    bool status_outside_gap = false;     // a STATUS started before its offset or ran past the slot
    bool telem_started_late = false;     // a TELEM started more than 4 ms after its slot

    Sim(const TxPlan& p, uint32_t telem_us, uint32_t status_us, uint32_t event_us,
        uint64_t first_tick_us)
        : plan(p), slots(p), telem_air_us(telem_us), status_air_us(status_us),
          event_air_us(event_us), now_us(first_tick_us) {
        slots.reset(0);
    }

    // Plan and airtimes for explicit radio settings, derived the way lora_link.h derives them.
    static Sim forBandwidth(long bw_hz, uint64_t first_tick_us) {
        const uint32_t ta = lora::timeOnAirUs(37, 7, bw_hz, 5);
        const uint32_t sa = lora::timeOnAirUs(53, 7, bw_hz, 5);
        const uint32_t ea = lora::timeOnAirUs(15, 7, bw_hz, 5);
        TxPlan plan{kPeriodMs, 10, lora::ceilMs(kStartCostUs + ta),
                    lora::ceilMs(kStartCostUs + sa) + 1, lora::ceilMs(kStartCostUs + ea) + 1};
        return Sim(plan, ta, sa, ea, first_tick_us);
    }

    // The settings in params.yaml, straight from lora_link.h's constants.
    static Sim configured(uint64_t first_tick_us) {
        TxPlan plan{lora::kTelemPeriodMs, lora::kStatusEveryNTelem, lora::kStatusOffsetMs,
                    lora::kStatusCostMs, lora::kEventCostMs};
        return Sim(plan, lora::kTelemAirUs, lora::kStatusAirUs, lora::kEventAirUs,
                   first_tick_us);
    }

    void tickOnce() {
        const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000);
        const bool busy = now_us < busy_until_us;
        switch (slots.choose(now_ms, busy, event_pending)) {
            case TxChoice::kTelem:
                telem++;
                busy_until_us = now_us + kStartCostUs + telem_air_us;
                if (now_ms % kPeriodMs > 4) telem_started_late = true;
                break;
            case TxChoice::kEvent:
                event++;
                event_pending = false;
                busy_until_us = now_us + kStartCostUs + event_air_us;
                break;
            case TxChoice::kStatus: {
                status++;
                busy_until_us = now_us + kStartCostUs + status_air_us;
                const uint32_t into_period_ms = now_ms % kPeriodMs;
                if (into_period_ms < plan.status_offset_ms ||
                    busy_until_us > (now_us / 1000 - into_period_ms + kPeriodMs) * 1000ULL) {
                    status_outside_gap = true;
                }
                break;
            }
            case TxChoice::kNone:
                break;
        }
        now_us += tick_us;
    }

    void run(uint32_t seconds) {
        const uint64_t end = now_us + static_cast<uint64_t>(seconds) * 1000000ULL;
        while (now_us < end) tickOnce();
    }
};

// ---- the requirement: STATUS starts in the gap after a TELEM and never over a TELEM slot -------

// At the settings in params.yaml, for every phase the 300 Hz ticks can have against the 100 ms
// grid: no TELEM is skipped, so none ever fell due with a STATUS on the air; every TELEM starts
// within a tick of its slot; every STATUS starts after its offset and finishes before the next
// slot; and all 20 STATUS frames of 20 s go out.
void test_configured_settings_keep_status_inside_the_gap(void) {
    for (uint64_t phase = 0; phase < lora::kLoopTickUs; phase += 61) {
        Sim sim = Sim::configured(phase);
        sim.run(20);
        TEST_ASSERT_EQUAL_UINT32(0, sim.slots.telemSkipped());
        TEST_ASSERT_FALSE(sim.telem_started_late);
        TEST_ASSERT_FALSE(sim.status_outside_gap);
        TEST_ASSERT_GREATER_OR_EQUAL(19, sim.status);
        TEST_ASSERT_GREATER_OR_EQUAL(199, sim.telem);
    }
}

// The bench loop was not at 300 Hz: with ticks about 52 ms apart the first scheduler skipped a
// TELEM nearly every second (a 52 ms STATUS started on one tick was still on the air at the
// next, which is where the TELEM slot is noticed). Frames are only started and noticed finished
// on ticks, so this holds the new scheduler to the same standard for slow ticks of every length
// from 20 to 60 ms: no TELEM is skipped and STATUS still goes out every second.
void test_a_slow_loop_does_not_make_status_skip_telem(void) {
    const uint64_t ticks_us[] = {20000, 40000, 51500, 51950, 52000, 52500, 60000};
    for (uint64_t tick : ticks_us) {
        for (uint64_t phase = 0; phase < tick; phase += tick / 25) {
            Sim sim = Sim::configured(phase);
            sim.tick_us = tick;
            sim.run(20);
            TEST_ASSERT_EQUAL_UINT32(0, sim.slots.telemSkipped());
            TEST_ASSERT_GREATER_OR_EQUAL(19, sim.status);
        }
    }
}

// The 250 kHz fallback (SPEC.md Section 4.7) holds to the same rule at 300 Hz ticks.
void test_the_250_khz_fallback_keeps_status_inside_the_gap(void) {
    for (uint64_t phase = 0; phase < lora::kLoopTickUs; phase += 61) {
        Sim sim = Sim::forBandwidth(250000, phase);
        sim.run(20);
        TEST_ASSERT_EQUAL_UINT32(0, sim.slots.telemSkipped());
        TEST_ASSERT_FALSE(sim.status_outside_gap);
        TEST_ASSERT_GREATER_OR_EQUAL(19, sim.status);
    }
}

// The check above must be able to fail. Give the scheduler a plan that understates how long a
// STATUS stays on the air (28 ms planned, 90 ms real): it admits the frame into gaps it does
// not fit, STATUS runs over the next TELEM slot, TELEMs are skipped and the gap check trips.
// This is also what guards the plan's cost numbers against drifting from the real airtime.
void test_the_check_catches_a_status_that_runs_over_a_telem_slot(void) {
    Sim sim = Sim::forBandwidth(500000, 0);
    sim.status_air_us = 90000;
    sim.run(20);
    TEST_ASSERT_GREATER_OR_EQUAL(10, sim.slots.telemSkipped());
    TEST_ASSERT_TRUE(sim.status_outside_gap);
}

// 125 kHz is over the channel's capacity: TELEM frames are skipped (and counted), every TELEM
// slot is either sent or counted, none is queued, and STATUS still goes out every second.
void test_saturated_link_skips_telem_but_still_sends_status(void) {
    Sim sim = Sim::forBandwidth(125000, 0);
    sim.run(60);
    TEST_ASSERT_TRUE(sim.telem < 600);
    TEST_ASSERT_GREATER_THAN_UINT32(30, sim.slots.telemSkipped());
    TEST_ASSERT_GREATER_OR_EQUAL(59, sim.status);
    TEST_ASSERT_INT_WITHIN(2, 600, sim.telem + static_cast<int>(sim.slots.telemSkipped()));
}

// ---- single decisions -------------------------------------------------------------------------

static TxPlan bw500Plan() {
    return TxPlan{100, 10, 22, 28, 14};  // offset, STATUS cost, EVENT cost in ms
}

// STATUS is not due with its TELEM: it waits for its offset, then goes once.
void test_status_waits_for_its_offset_after_the_telem_slot(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(3, true, false));    // TELEM on the air
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(21, false, false));  // free, but 1 ms early
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(22, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(26, false, false));
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

// A STATUS that would still be on the air when the next TELEM slot falls due is held for the
// next gap rather than started.
void test_status_that_would_cross_the_next_slot_is_held(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(80, false, false));  // 20 ms left, needs 28
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(100, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(122, false, false));  // the next gap
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

// A STATUS too long for any gap (95 ms into a 100 ms period with a TELEM in front of it) is
// held back; once it is more than half a STATUS period (500 ms) late it goes out regardless, at
// the cost of a TELEM, so a gap that never opens cannot starve it forever.
void test_overdue_status_goes_out_even_without_a_gap(void) {
    TxPlan plan = bw500Plan();
    plan.status_cost_ms = 95;
    TxSlots slots(plan);
    slots.reset(0);
    uint32_t status_at = 0, busy_until = 0;
    for (uint32_t t = 0; t < 700 && status_at == 0; t++) {
        const TxChoice c = slots.choose(t, t < busy_until, false);
        if (c == TxChoice::kTelem) busy_until = t + 21;  // a BW500 TELEM, start-up included
        if (c == TxChoice::kStatus) status_at = t;
    }
    TEST_ASSERT_TRUE(status_at > 0);
    TEST_ASSERT_EQUAL_UINT32(523, status_at);  // due at 22 ms, overdue once more than 500 ms late
}

// An EVENT copy goes before STATUS, after a TELEM that is due, and only with room before the
// next TELEM slot.
void test_event_copy_needs_room_and_goes_before_status(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kEvent, slots.choose(23, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(90, false, true));  // 10 ms left, needs 14
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(100, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kEvent, slots.choose(123, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(126, false, false));
}

// A frame that falls due while the radio is busy is dropped, not delayed: after the radio
// frees up there is one TELEM per period again, not a catch-up burst.
void test_busy_radio_skips_telem_and_does_not_queue(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(100, true, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(200, true, false));
    TEST_ASSERT_EQUAL_UINT32(2, slots.telemSkipped());
    // free again at 250: the two skipped TELEMs are not owed; the STATUS that has been waiting
    // since 22 ms is what goes, and the next TELEM is the 300 ms slot
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(250, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(300, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(301, false, false));
    TEST_ASSERT_EQUAL_UINT32(2, slots.telemSkipped());
}

// A long stall (boot SELFTEST) must not turn into a burst of frames or a pile of phantom skips:
// the TELEM grid restarts from the first tick after it, one frame goes out, and the next one is
// a full period later.
void test_a_stall_does_not_replay_missed_slots(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(22, false, false));
    // the loop does not run for 5 s, then resumes with a free radio
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(5050, false, false));
    for (uint32_t t = 5053; t < 5150; t += 3) {
        TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(t, false, false));
    }
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(5150, false, false));
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

void test_clear_skipped_zeroes_the_counter(void) {
    TxSlots slots(bw500Plan());
    slots.reset(0);
    slots.choose(0, true, false);
    TEST_ASSERT_EQUAL_UINT32(1, slots.telemSkipped());
    slots.clearSkipped();
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_configured_settings_keep_status_inside_the_gap);
    RUN_TEST(test_a_slow_loop_does_not_make_status_skip_telem);
    RUN_TEST(test_the_250_khz_fallback_keeps_status_inside_the_gap);
    RUN_TEST(test_the_check_catches_a_status_that_runs_over_a_telem_slot);
    RUN_TEST(test_saturated_link_skips_telem_but_still_sends_status);
    RUN_TEST(test_status_waits_for_its_offset_after_the_telem_slot);
    RUN_TEST(test_status_that_would_cross_the_next_slot_is_held);
    RUN_TEST(test_overdue_status_goes_out_even_without_a_gap);
    RUN_TEST(test_event_copy_needs_room_and_goes_before_status);
    RUN_TEST(test_busy_radio_skips_telem_and_does_not_queue);
    RUN_TEST(test_a_stall_does_not_replay_missed_slots);
    RUN_TEST(test_clear_skipped_zeroes_the_counter);
    return UNITY_END();
}
