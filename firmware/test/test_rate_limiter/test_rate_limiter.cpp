#include <unity.h>

#include <cmath>

#include "rate_limiter.h"

void setUp(void) {}
void tearDown(void) {}

void test_reaches_target_within_slew_budget(void) {
    float delta = 0.0f;
    // slew=500 deg/s, dt=0.01 -> max step 5 deg/tick; commanding 3 deg is within budget.
    float out = tvc::rate_limiter_step(delta, 3.0f, 0.01f, 500.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, delta);
}

void test_never_exceeds_slew_limit_per_step(void) {
    float delta = 0.0f;
    const float slew = 500.0f, dt = 0.01f;  // max step = 5 deg/tick
    const float target = 100.0f;

    for (int i = 0; i < 19; ++i) {
        float prev = delta;
        float out = tvc::rate_limiter_step(delta, target, dt, slew);
        TEST_ASSERT_TRUE(std::fabs(out - prev) <= 5.0f + 1e-6f);
        TEST_ASSERT_TRUE(out < target);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 95.0f, delta);

    tvc::rate_limiter_step(delta, target, dt, slew);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, delta);

    // Once caught up, further steps hold at the target (zero further motion needed).
    float out = tvc::rate_limiter_step(delta, target, dt, slew);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out);
}

void test_symmetric_in_negative_direction(void) {
    float delta = 0.0f;
    const float slew = 500.0f, dt = 0.01f;
    for (int i = 0; i < 20; ++i) {
        tvc::rate_limiter_step(delta, -100.0f, dt, slew);
    }
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, delta);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_reaches_target_within_slew_budget);
    RUN_TEST(test_never_exceeds_slew_limit_per_step);
    RUN_TEST(test_symmetric_in_negative_direction);
    return UNITY_END();
}
