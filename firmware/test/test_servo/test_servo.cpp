#include <unity.h>

#include <cmath>

#include "servo.h"

void setUp(void) {}
void tearDown(void) {}

void test_reaches_target_within_slew_budget(void) {
    float delta = 0.0f;
    // slew=500 deg/s, dt=0.01 -> max step 5 deg/tick; tau tiny so the lag step always
    // exceeds the slew step, i.e. slew alone decides (matches the pre-lag model).
    float out = tvc::servo_step(delta, 3.0f, 0.01f, 1e-6f, 500.0f);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, delta);
}

void test_never_exceeds_slew_limit_per_step(void) {
    float delta = 0.0f;
    const float slew = 500.0f, dt = 0.01f, tau = 1e-6f;  // max step = 5 deg/tick
    const float target = 100.0f;

    for (int i = 0; i < 19; ++i) {
        float prev = delta;
        float out = tvc::servo_step(delta, target, dt, tau, slew);
        TEST_ASSERT_TRUE(std::fabs(out - prev) <= 5.0f + 1e-6f);
        TEST_ASSERT_TRUE(out < target);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 95.0f, delta);

    tvc::servo_step(delta, target, dt, tau, slew);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, delta);

    // Once caught up, further steps hold at the target (zero further motion needed).
    float out = tvc::servo_step(delta, target, dt, tau, slew);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out);
}

void test_symmetric_in_negative_direction(void) {
    float delta = 0.0f;
    const float slew = 500.0f, dt = 0.01f, tau = 1e-6f;
    for (int i = 0; i < 20; ++i) {
        tvc::servo_step(delta, -100.0f, dt, tau, slew);
    }
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, delta);
}

void test_first_order_lag_matches_exponential_when_slew_not_binding(void) {
    // slew limit set high enough to never bind (50 deg/tick >> the lag's own implied
    // step), so the response should follow the exact first-order discretization:
    // delta_k = target * (1 - (1-alpha)^k), alpha = 1 - exp(-dt/tau).
    float delta = 0.0f;
    const float dt = 0.01f, tau = 0.05f, slew = 5000.0f;  // max step = 50 deg/tick
    const float target = 10.0f;
    const float alpha = 1.0f - std::exp(-dt / tau);

    float expected = 0.0f;
    for (int i = 0; i < 10; ++i) {
        tvc::servo_step(delta, target, dt, tau, slew);
        expected = expected + alpha * (target - expected);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, delta);
    }
    // Genuinely decaying toward, not slewing linearly to, the target: the step taken on
    // the very first tick must be smaller than a naive slew step would allow, proving
    // the lag (not the slew limit) governed it.
    TEST_ASSERT_TRUE(expected < target);
}

void test_small_error_decays_slower_than_pure_slew_would(void) {
    // A small residual error near the target should decay like a first-order system,
    // not slew at the full rate right up to zero error and then stop dead -- that
    // discontinuous behavior is exactly what adding tau_s is meant to remove.
    float delta = 9.0f;
    const float dt = 0.01f, tau = 0.05f, slew = 5000.0f;
    float step1 = tvc::servo_step(delta, 10.0f, dt, tau, slew) - 9.0f;
    TEST_ASSERT_TRUE(step1 > 0.0f);
    TEST_ASSERT_TRUE(step1 < 1.0f);  // did not jump straight to the target in one tick
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_reaches_target_within_slew_budget);
    RUN_TEST(test_never_exceeds_slew_limit_per_step);
    RUN_TEST(test_symmetric_in_negative_direction);
    RUN_TEST(test_first_order_lag_matches_exponential_when_slew_not_binding);
    RUN_TEST(test_small_error_decays_slower_than_pure_slew_would);
    return UNITY_END();
}
