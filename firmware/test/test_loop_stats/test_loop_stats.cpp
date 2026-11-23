#include <unity.h>

#include <stdint.h>

#include "../../src/flight/loop_stats.h"

using tvc::flight::countsAsOverrun;
using tvc::flight::LoopStats;

void setUp(void) {}
void tearDown(void) {}

void test_first_tick_after_boot_is_ignored(void) {
    LoopStats s;
    s.record(1000, 1000 + 900000, 5000000);  // a huge first tick: must not show anywhere
    TEST_ASSERT_EQUAL_UINT32(0, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(0, s.execMaxUs());
    TEST_ASSERT_EQUAL_UINT32(0, s.periodMaxUs());
    TEST_ASSERT_EQUAL_UINT32(0, s.behindMaxUs());
    for (int i = 0; i < LoopStats::kBins; i++) TEST_ASSERT_EQUAL_UINT32(0, s.hist(i));
}

// Execution time is end minus start of one tick; the period is start minus the previous start.
// They are tracked separately: a 400 us tick that follows a 20 ms gap has a short exec time and
// a long period.
void test_exec_and_period_are_measured_separately(void) {
    LoopStats s;
    s.record(0, 800, 0);            // first tick, ignored
    s.record(3333, 3333 + 1200, 0);     // exec 1200, period 3333
    s.record(23333, 23333 + 400, 17000);  // exec 400, period 20000, 17 ms behind
    s.record(26666, 26666 + 3000, 0);     // exec 3000, period 3333
    TEST_ASSERT_EQUAL_UINT32(3, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(3000, s.execMaxUs());
    TEST_ASSERT_EQUAL_UINT32(20000, s.periodMaxUs());
    TEST_ASSERT_EQUAL_UINT32(17000, s.behindMaxUs());
}

void test_histogram_bin_edges(void) {
    TEST_ASSERT_EQUAL_INT(0, LoopStats::bin(0));
    TEST_ASSERT_EQUAL_INT(0, LoopStats::bin(499));
    TEST_ASSERT_EQUAL_INT(1, LoopStats::bin(500));
    TEST_ASSERT_EQUAL_INT(1, LoopStats::bin(999));
    TEST_ASSERT_EQUAL_INT(2, LoopStats::bin(1000));
    TEST_ASSERT_EQUAL_INT(3, LoopStats::bin(2000));
    TEST_ASSERT_EQUAL_INT(3, LoopStats::bin(3332));
    TEST_ASSERT_EQUAL_INT(4, LoopStats::bin(3333));
    TEST_ASSERT_EQUAL_INT(4, LoopStats::bin(9999));
    TEST_ASSERT_EQUAL_INT(5, LoopStats::bin(10000));
    TEST_ASSERT_EQUAL_INT(5, LoopStats::bin(4000000));
}

void test_histogram_counts_every_recorded_tick(void) {
    LoopStats s;
    uint32_t t = 0;
    s.record(t, t + 100, 0);  // ignored
    const uint32_t execs[] = {100, 600, 600, 1500, 2500, 5000, 20000};
    for (uint32_t e : execs) {
        t += 3333;
        s.record(t, t + e, 0);
    }
    TEST_ASSERT_EQUAL_UINT32(7, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(1, s.hist(0));  // 100
    TEST_ASSERT_EQUAL_UINT32(2, s.hist(1));  // 600, 600
    TEST_ASSERT_EQUAL_UINT32(1, s.hist(2));  // 1500
    TEST_ASSERT_EQUAL_UINT32(1, s.hist(3));  // 2500
    TEST_ASSERT_EQUAL_UINT32(1, s.hist(4));  // 5000
    TEST_ASSERT_EQUAL_UINT32(1, s.hist(5));  // 20000
    uint32_t total = 0;
    for (int i = 0; i < LoopStats::kBins; i++) total += s.hist(i);
    TEST_ASSERT_EQUAL_UINT32(s.ticks(), total);
}

// micros() wraps at 2^32; differences must still come out right.
void test_timer_wraparound(void) {
    LoopStats s;
    s.record(0xFFFFF000u, 0xFFFFF100u, 0);
    s.record(0xFFFFFFFFu - 99u, 0xFFFFFFFFu - 99u + 700u, 0);  // end wraps past zero
    TEST_ASSERT_EQUAL_UINT32(700, s.execMaxUs());
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu - 99u - 0xFFFFF000u, s.periodMaxUs());
}

// CLEARSTATS: everything back to zero, and the first tick afterwards is ignored again, so a
// maximum cannot carry a timestamp from before the clear.
void test_reset_clears_and_ignores_the_next_tick(void) {
    LoopStats s;
    s.record(0, 100, 0);
    s.record(3333, 3333 + 9000, 50000);
    TEST_ASSERT_EQUAL_UINT32(1, s.ticks());
    s.reset();
    TEST_ASSERT_EQUAL_UINT32(0, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(0, s.execMaxUs());
    s.record(5000000, 5000000 + 70000, 123000000);  // first tick after the clear
    TEST_ASSERT_EQUAL_UINT32(0, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(0, s.behindMaxUs());
    s.record(5003333, 5003333 + 800, 0);
    TEST_ASSERT_EQUAL_UINT32(1, s.ticks());
    TEST_ASSERT_EQUAL_UINT32(800, s.execMaxUs());
    TEST_ASSERT_EQUAL_UINT32(3333, s.periodMaxUs());
}

// Per-section maxima find what makes a slow tick slow. Like everything else, the first tick
// after a reset does not count.
void test_section_maxima_ignore_the_first_tick_and_clear_on_reset(void) {
    LoopStats s;
    s.section(0, 9000);  // first tick: ignored
    s.record(0, 100, 0);
    TEST_ASSERT_EQUAL_UINT32(0, s.sectionMaxUs(0));
    s.section(0, 400);
    s.section(2, 50000);
    s.section(0, 700);
    s.section(4, 90);
    s.record(3333, 3333 + 800, 0);
    TEST_ASSERT_EQUAL_UINT32(700, s.sectionMaxUs(0));
    TEST_ASSERT_EQUAL_UINT32(0, s.sectionMaxUs(1));
    TEST_ASSERT_EQUAL_UINT32(50000, s.sectionMaxUs(2));
    TEST_ASSERT_EQUAL_UINT32(90, s.sectionMaxUs(4));
    s.reset();
    s.section(2, 123456);  // first tick after the reset: ignored again
    s.record(10000, 10100, 0);
    for (int i = 0; i < LoopStats::kSections; i++) TEST_ASSERT_EQUAL_UINT32(0, s.sectionMaxUs(i));
}

// ---- what loop_overrun_count counts ------------------------------------------------------------
// The rule: a tick overruns when its body took more than 3300 us to execute, judged when it
// ends (sequencer.cpp). main.cpp starts a tick when its elapsedMicros timer reaches one period,
// then subtracts one period ("preserve phase"); this runs that arithmetic with a given tick cost.
struct LoopRun {
    uint32_t ticks = 0, overruns = 0;
    LoopStats stats;
    uint64_t last_behind_us = 0;
};

static LoopRun runLoop(uint32_t exec_us, uint32_t seconds) {
    constexpr uint32_t kPeriod = 3333;
    LoopRun r;
    uint64_t now = 0, epoch = 0;
    const uint64_t end = static_cast<uint64_t>(seconds) * 1000000ULL;
    while (now < end) {
        if (now - epoch >= kPeriod) {
            const uint32_t loop_us = static_cast<uint32_t>(now - epoch);
            epoch += kPeriod;
            r.ticks++;
            if (countsAsOverrun(exec_us)) r.overruns++;
            const uint32_t behind = loop_us > kPeriod ? loop_us - kPeriod : 0;
            r.stats.record(static_cast<uint32_t>(now), static_cast<uint32_t>(now + exec_us),
                           behind);
            r.last_behind_us = behind;
            now += exec_us;
        }
        now += 2;  // the rest of one pass through loop()
    }
    return r;
}

void test_overrun_threshold_is_3300_us_of_execution_time(void) {
    TEST_ASSERT_FALSE(countsAsOverrun(0));
    TEST_ASSERT_FALSE(countsAsOverrun(3300));
    TEST_ASSERT_TRUE(countsAsOverrun(3301));
    TEST_ASSERT_TRUE(countsAsOverrun(52000));
}

// Healthy loop: 1 ms per tick, 300 ticks a second, and the counter stays at zero. (Until the
// rule was moved to execution time it counted every one of these ticks.)
void test_a_healthy_loop_reads_zero_overruns(void) {
    LoopRun r = runLoop(1000, 60);
    TEST_ASSERT_INT_WITHIN(20, 18000, static_cast<int>(r.ticks));
    TEST_ASSERT_EQUAL_UINT32(0, r.overruns);
    TEST_ASSERT_TRUE(r.stats.behindMaxUs() < 100);    // on schedule
    TEST_ASSERT_TRUE(r.stats.periodMaxUs() < 3400);
    TEST_ASSERT_TRUE(r.stats.execMaxUs() == 1000);
}

// A loop that overruns now and then reads the number of slow ticks: here every 100th tick takes
// 5 ms. Also shows the loop recovers: the backlog from one slow tick drains.
void test_occasional_slow_ticks_are_counted_one_each(void) {
    constexpr uint32_t kPeriod = 3333;
    uint64_t now = 0, epoch = 0;
    uint32_t ticks = 0, overruns = 0;
    LoopStats stats;
    while (now < 60ULL * 1000000ULL) {
        if (now - epoch >= kPeriod) {
            const uint32_t loop_us = static_cast<uint32_t>(now - epoch);
            epoch += kPeriod;
            ticks++;
            const uint32_t exec_us = (ticks % 100 == 0) ? 5000 : 800;
            if (countsAsOverrun(exec_us)) overruns++;
            stats.record(static_cast<uint32_t>(now), static_cast<uint32_t>(now + exec_us),
                         loop_us > kPeriod ? loop_us - kPeriod : 0);
            now += exec_us;
        }
        now += 2;
    }
    TEST_ASSERT_INT_WITHIN(2, static_cast<int>(ticks / 100), static_cast<int>(overruns));
    TEST_ASSERT_TRUE(stats.behindMaxUs() < 2000);  // never more than the one slow tick's excess
}

// Overloaded loop: every tick costs 52 ms. Every tick overruns, so the counter rises once per
// tick that ran (about 19 a second, 1154 in 60 s, the bench's +1155), and "behind" grows by
// about (1 - 3333/52000) seconds per second without limit.
void test_overloaded_loop_overruns_once_per_tick_that_ran_and_falls_behind(void) {
    LoopRun r = runLoop(52000, 60);
    TEST_ASSERT_INT_WITHIN(5, 1153, static_cast<int>(r.ticks));
    TEST_ASSERT_EQUAL_UINT32(r.ticks, r.overruns);
    TEST_ASSERT_EQUAL_UINT32(52000, r.stats.execMaxUs());
    TEST_ASSERT_TRUE(r.stats.periodMaxUs() < 52100);

    LoopRun longer = runLoop(52000, 130);
    const double expect_s = 130.0 * (1.0 - 3333.0 / 52000.0);
    const double got_s = longer.last_behind_us / 1e6;
    TEST_ASSERT_TRUE(got_s > expect_s * 0.97 && got_s < expect_s * 1.03);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_first_tick_after_boot_is_ignored);
    RUN_TEST(test_exec_and_period_are_measured_separately);
    RUN_TEST(test_histogram_bin_edges);
    RUN_TEST(test_histogram_counts_every_recorded_tick);
    RUN_TEST(test_timer_wraparound);
    RUN_TEST(test_reset_clears_and_ignores_the_next_tick);
    RUN_TEST(test_section_maxima_ignore_the_first_tick_and_clear_on_reset);
    RUN_TEST(test_overrun_threshold_is_3300_us_of_execution_time);
    RUN_TEST(test_a_healthy_loop_reads_zero_overruns);
    RUN_TEST(test_occasional_slow_ticks_are_counted_one_each);
    RUN_TEST(test_overloaded_loop_overruns_once_per_tick_that_ran_and_falls_behind);
    return UNITY_END();
}
