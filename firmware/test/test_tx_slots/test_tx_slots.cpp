#include <unity.h>

#include <stdint.h>

#include "../../src/flight/tx_slots.h"
#include "lora_link.h"

using tvc::flight::TxChoice;
using tvc::flight::TxSlots;
using tvc::protocol::lora::timeOnAirUs;

void setUp(void) {}
void tearDown(void) {}

// The vehicle's loop, simulated: a tick every 3333 us (300 Hz) calling TxSlots::choose() with
// the state of a pretend radio that stays busy for the airtime of whatever it was told to send.
// Frame sizes on the air: TELEM 33 + 4 RadioHead bytes, STATUS 48 + 4, EVENT 11 + 4.
struct Sim {
    TxSlots slots{100, 1000};
    uint32_t telem_air_us, status_air_us, event_air_us;
    uint64_t now_us = 0;
    uint64_t busy_until_us = 0;
    bool event_pending = false;
    int telem = 0, status = 0, event = 0;
    uint64_t status_gap_max_us = 0, last_status_us = 0;

    explicit Sim(long bw_hz) {
        telem_air_us = timeOnAirUs(37, 7, bw_hz, 5);
        status_air_us = timeOnAirUs(52, 7, bw_hz, 5);
        event_air_us = timeOnAirUs(15, 7, bw_hz, 5);
        slots.reset(0);
    }

    void tickOnce() {
        const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000);
        const bool busy = now_us < busy_until_us;
        switch (slots.choose(now_ms, busy, event_pending)) {
            case TxChoice::kTelem:
                telem++;
                busy_until_us = now_us + telem_air_us;
                break;
            case TxChoice::kEvent:
                event++;
                event_pending = false;
                busy_until_us = now_us + event_air_us;
                break;
            case TxChoice::kStatus:
                status++;
                busy_until_us = now_us + status_air_us;
                if (last_status_us && now_us - last_status_us > status_gap_max_us) {
                    status_gap_max_us = now_us - last_status_us;
                }
                last_status_us = now_us;
                break;
            case TxChoice::kNone:
                break;
        }
        now_us += 3333;
    }

    void run(uint32_t seconds) {
        const uint64_t end = now_us + static_cast<uint64_t>(seconds) * 1000000ULL;
        while (now_us < end) tickOnce();
    }
};

// BW250: 41 ms + 51 ms every 100 ms fits, so nothing is skipped and STATUS keeps its 1 Hz.
void test_bw250_sends_every_telem_and_every_status(void) {
    Sim sim(250000);
    sim.run(60);
    TEST_ASSERT_GREATER_OR_EQUAL(599, sim.telem);
    TEST_ASSERT_EQUAL_UINT32(0, sim.slots.telemSkipped());
    TEST_ASSERT_GREATER_OR_EQUAL(59, sim.status);
    TEST_ASSERT_TRUE(sim.status_gap_max_us < 1010000);  // never later than a tick or two
}

// BW125 is over the channel's capacity: TELEM frames get skipped (and counted), but the
// STATUS frame still goes out every second instead of losing to TELEM forever.
void test_bw125_skips_telem_but_still_sends_status(void) {
    Sim sim(125000);
    sim.run(60);
    TEST_ASSERT_TRUE(sim.telem < 600);
    TEST_ASSERT_GREATER_THAN_UINT32(30, sim.slots.telemSkipped());
    TEST_ASSERT_GREATER_OR_EQUAL(59, sim.status);
    // every TELEM slot is either sent or counted as skipped, none is queued for later
    TEST_ASSERT_INT_WITHIN(2, 600, sim.telem + static_cast<int>(sim.slots.telemSkipped()));
}

// A frame that falls due while the radio is busy is dropped, not delayed: after the radio
// frees up there is one TELEM per period again, not a catch-up burst.
void test_busy_radio_skips_and_does_not_queue(void) {
    TxSlots slots(100, 1000);
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    // radio busy from 0 to 250 ms; TELEM slots at 100 and 200 fall due meanwhile
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(100, true, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(200, true, false));
    TEST_ASSERT_EQUAL_UINT32(2, slots.telemSkipped());
    // free again at 250: the two skipped TELEMs are not owed. STATUS, which has been due since
    // t=0, is what goes, and the next TELEM is the 300 ms slot.
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(250, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(300, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(301, false, false));
    TEST_ASSERT_EQUAL_UINT32(2, slots.telemSkipped());
}

// TELEM and STATUS are both due at t=0 with a free radio: TELEM goes first, STATUS on the
// next free tick, and STATUS keeps its place rather than being skipped.
void test_telem_first_then_status_on_the_next_free_tick(void) {
    TxSlots slots(100, 1000);
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(3, true, false));   // TELEM still on air
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(43, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(46, false, false));
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

// A pending EVENT copy goes before STATUS but never ahead of a TELEM that is due.
void test_event_copy_goes_before_status_and_after_telem(void) {
    TxSlots slots(100, 1000);
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kEvent, slots.choose(43, false, true));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(70, false, false));
}

// A long stall (boot SELFTEST) must not turn into a burst of frames or a pile of phantom skips.
void test_a_stall_does_not_replay_missed_slots(void) {
    TxSlots slots(100, 1000);
    slots.reset(0);
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(0, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(50, false, false));
    // the loop does not run for 5 s, then resumes with a free radio
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(5050, false, false));
    TEST_ASSERT_EQUAL(TxChoice::kStatus, slots.choose(5053, false, false));  // STATUS was due too
    for (uint32_t t = 5056; t < 5150; t += 3) {
        TEST_ASSERT_EQUAL(TxChoice::kNone, slots.choose(t, false, false));
    }
    TEST_ASSERT_EQUAL(TxChoice::kTelem, slots.choose(5150, false, false));
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

void test_clear_skipped_zeroes_the_counter(void) {
    TxSlots slots(100, 1000);
    slots.reset(0);
    slots.choose(0, true, false);
    TEST_ASSERT_EQUAL_UINT32(1, slots.telemSkipped());
    slots.clearSkipped();
    TEST_ASSERT_EQUAL_UINT32(0, slots.telemSkipped());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_bw250_sends_every_telem_and_every_status);
    RUN_TEST(test_bw125_skips_telem_but_still_sends_status);
    RUN_TEST(test_busy_radio_skips_and_does_not_queue);
    RUN_TEST(test_telem_first_then_status_on_the_next_free_tick);
    RUN_TEST(test_event_copy_goes_before_status_and_after_telem);
    RUN_TEST(test_a_stall_does_not_replay_missed_slots);
    RUN_TEST(test_clear_skipped_zeroes_the_counter);
    return UNITY_END();
}
