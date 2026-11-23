#include <unity.h>

#include <stdint.h>

#include "../../src/flight/i2c_guard.h"

using namespace tvc::flight::i2c;

void setUp(void) {}
void tearDown(void) {}

void test_tick_budget_is_one_ms_and_refuses_a_start_with_too_little_left(void) {
    TickBudget b;
    b.begin(5000);
    TEST_ASSERT_EQUAL_INT32(1000, b.remainingUs(5000));
    TEST_ASSERT_TRUE(b.mayStart(5000));
    TEST_ASSERT_TRUE(b.mayStart(5000 + 1000 - kMinStartUs));
    TEST_ASSERT_FALSE(b.mayStart(5000 + 1000 - kMinStartUs + 1));
    TEST_ASSERT_FALSE(b.mayStart(7000));
    TEST_ASSERT_TRUE(b.remainingUs(7000) < 0);
}

void test_tick_budget_survives_micros_wraparound(void) {
    TickBudget b;
    b.begin(0xFFFFFF00u);  // deadline = 0x2E8, past the wrap
    TEST_ASSERT_TRUE(b.mayStart(0xFFFFFF00u));
    TEST_ASSERT_EQUAL_INT32(1000 - 512, b.remainingUs(0x00000100u));  // 512 us after the start
    TEST_ASSERT_FALSE(b.mayStart(0x00000400u));
}

void test_a_failed_bus_is_left_alone_for_the_backoff_then_retried(void) {
    Guard g;
    TEST_ASSERT_TRUE(g.mayStart(0));
    g.record(Result::kTimeout, 1000);
    TEST_ASSERT_FALSE(g.mayStart(1000));
    TEST_ASSERT_FALSE(g.mayStart(1000 + kBackoffUs - 1));
    TEST_ASSERT_TRUE(g.mayStart(1000 + kBackoffUs));
    g.record(Result::kOk, 1000 + kBackoffUs + 400);
    TEST_ASSERT_TRUE(g.mayStart(1000 + kBackoffUs + 500));
}

void test_counts_are_kept_apart(void) {
    Guard g;
    g.record(Result::kTimeout, 0);
    g.record(Result::kTimeout, 100000);
    g.record(Result::kNack, 200000);
    g.record(Result::kError, 300000);
    g.record(Result::kBackoff, 300001);
    g.record(Result::kNoBudget, 300002);
    TEST_ASSERT_EQUAL_UINT32(2, g.timeouts());
    TEST_ASSERT_EQUAL_UINT32(2, g.errors());   // the NACK and the hardware error
    TEST_ASSERT_EQUAL_UINT32(2, g.skipped());
    g.clearCounts();
    TEST_ASSERT_EQUAL_UINT32(0, g.timeouts() + g.errors() + g.skipped());
}

// Skipped transactions never happened: they must not push the back-off further out.
void test_skips_do_not_extend_the_backoff(void) {
    Guard g;
    g.record(Result::kTimeout, 0);
    g.record(Result::kBackoff, 15000);
    g.record(Result::kNoBudget, 19000);
    TEST_ASSERT_TRUE(g.mayStart(kBackoffUs));
}

// The contract of the whole thing, as arithmetic: buses that hang, 300 ticks a second. A hung
// transaction runs until the tick's deadline and then stops, so however the buses fail, one tick
// never spends more than 1000 us on I2C in total, and each dead bus is tried at most once per
// 20 ms of back-off.
static void runDeadBuses(int dead_buses, uint32_t* worst_tick_us, uint32_t* tries_per_bus_max,
                         uint32_t* total_us) {
    TickBudget budget;
    Guard guards[kBusCount];
    uint32_t tries[kBusCount] = {};
    *worst_tick_us = *total_us = 0;
    uint32_t now = 123456;
    const uint32_t tick_us = 3333;
    for (int tick = 0; tick < 3000; tick++) {  // 10 s
        const uint32_t tick_start = now;
        budget.begin(now);
        for (int bus = 0; bus < dead_buses; bus++) {
            if (!guards[bus].mayStart(now)) { guards[bus].record(Result::kBackoff, now); continue; }
            if (!budget.mayStart(now)) { guards[bus].record(Result::kNoBudget, now); continue; }
            tries[bus]++;
            now = budget.deadlineUs();  // a hung transaction stops at the deadline
            guards[bus].record(Result::kTimeout, now);
        }
        const uint32_t spent = now - tick_start;
        if (spent > *worst_tick_us) *worst_tick_us = spent;
        *total_us += spent;
        now = tick_start + tick_us;
    }
    *tries_per_bus_max = 0;
    for (uint32_t t : tries) if (t > *tries_per_bus_max) *tries_per_bus_max = t;
}

void test_dead_buses_never_cost_a_tick_more_than_1_ms(void) {
    for (int dead = 1; dead <= kBusCount; dead++) {
        uint32_t worst, per_bus, total;
        runDeadBuses(dead, &worst, &per_bus, &total);
        TEST_ASSERT_TRUE(worst <= kTickBudgetUs);
        TEST_ASSERT_TRUE(per_bus <= 10u * 1000000u / kBackoffUs + 1);  // 10 s of back-off
    }
}

// One dead bus (the case on the bench: one encoder bus that never answers) costs the loop about
// 1 ms in every 23 ms, a little over 4%, instead of tens of milliseconds in every tick.
void test_one_dead_bus_costs_about_four_percent_of_the_loop(void) {
    uint32_t worst, per_bus, total;
    runDeadBuses(1, &worst, &per_bus, &total);
    TEST_ASSERT_TRUE(total < 10u * 1000000u / 20u);  // under 5% of 10 s
    TEST_ASSERT_TRUE(total > 10u * 1000000u / 40u);  // and it really is being retried
}

// A healthy IMU read (about 430 us at 400 kHz) plus two encoder reads (about 130 us each) fits
// the budget with room to spare, so the budget does not cut off a working tick.
void test_a_healthy_tick_fits_in_the_budget(void) {
    TickBudget budget;
    budget.begin(0);
    uint32_t now = 0;
    const uint32_t costs_us[] = {430, 130, 130};
    for (uint32_t c : costs_us) {
        TEST_ASSERT_TRUE(budget.mayStart(now));
        now += c;
    }
    TEST_ASSERT_TRUE(now < kTickBudgetUs);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_tick_budget_is_one_ms_and_refuses_a_start_with_too_little_left);
    RUN_TEST(test_tick_budget_survives_micros_wraparound);
    RUN_TEST(test_a_failed_bus_is_left_alone_for_the_backoff_then_retried);
    RUN_TEST(test_counts_are_kept_apart);
    RUN_TEST(test_skips_do_not_extend_the_backoff);
    RUN_TEST(test_dead_buses_never_cost_a_tick_more_than_1_ms);
    RUN_TEST(test_one_dead_bus_costs_about_four_percent_of_the_loop);
    RUN_TEST(test_a_healthy_tick_fits_in_the_budget);
    return UNITY_END();
}
