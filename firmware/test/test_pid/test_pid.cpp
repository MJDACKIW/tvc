#include <unity.h>

#include "pid.h"

void setUp(void) {}
void tearDown(void) {}

// All fixture gains below are synthetic, chosen to make each behavior easy to hand-check.
// They are not the params.yaml flight values (those are MEASURE until the stand tuning session).

void test_pure_proportional(void) {
    float integral = 0.0f;
    bool saturated = false;
    tvc::PidResult r = tvc::pid_step(integral, saturated, /*x_hat=*/3.0f, /*d_error=*/0.0f,
                                      /*dt=*/0.01f, /*kp=*/2.0f, /*ki=*/0.0f, /*kd=*/0.0f,
                                      /*integral_clamp=*/100.0f, /*max_deflection=*/100.0f);
    // e = 0 - 3 = -3, u = kp*e = -6, well within the deflection limit.
    TEST_ASSERT_EQUAL_FLOAT(-6.0f, r.u_raw);
    TEST_ASSERT_EQUAL_FLOAT(-6.0f, r.u_cmd);
    TEST_ASSERT_FALSE(saturated);
}

void test_output_clamps_and_flags_saturated(void) {
    float integral = 0.0f;
    bool saturated = false;
    tvc::PidResult r = tvc::pid_step(integral, saturated, /*x_hat=*/50.0f, /*d_error=*/0.0f,
                                      /*dt=*/0.01f, /*kp=*/10.0f, /*ki=*/0.0f, /*kd=*/0.0f,
                                      /*integral_clamp=*/1000.0f, /*max_deflection=*/10.0f);
    // e = -50, u_raw = -500, clamped to -10.
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, r.u_raw);
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, r.u_cmd);
    TEST_ASSERT_TRUE(saturated);
}

void test_anti_windup_freezes_integral_while_saturated(void) {
    float integral = 0.0f;
    bool saturated = false;
    const float kp = 1.0f, ki = 1.0f, kd = 0.0f;
    const float integral_clamp = 1000.0f, max_deflection = 5.0f;

    tvc::pid_step(integral, saturated, /*x_hat=*/10.0f, /*d_error=*/0.0f, /*dt=*/1.0f,
                  kp, ki, kd, integral_clamp, max_deflection);
    TEST_ASSERT_TRUE(saturated);
    float integral_after_first = integral;

    tvc::pid_step(integral, saturated, /*x_hat=*/10.0f, /*d_error=*/0.0f, /*dt=*/1.0f,
                  kp, ki, kd, integral_clamp, max_deflection);
    TEST_ASSERT_TRUE(saturated);
    TEST_ASSERT_EQUAL_FLOAT(integral_after_first, integral);
}

void test_integral_clamp_bounds_state_even_when_not_saturated(void) {
    float integral = 0.0f;
    bool saturated = false;
    // kp=0 so u never saturates (max_deflection is huge); only the integral clamp is at play.
    for (int i = 0; i < 3; ++i) {
        tvc::pid_step(integral, saturated, /*x_hat=*/-10.0f, /*d_error=*/0.0f, /*dt=*/1.0f,
                      /*kp=*/0.0f, /*ki=*/1.0f, /*kd=*/0.0f,
                      /*integral_clamp=*/2.0f, /*max_deflection=*/1000.0f);
        TEST_ASSERT_FALSE(saturated);
        TEST_ASSERT_EQUAL_FLOAT(2.0f, integral);
    }
}

void test_derivative_uses_supplied_d_error_directly(void) {
    // Unlike Phase 1's core/, the derivative is not a finite difference computed here:
    // the caller (controller.cpp) supplies d_error directly (from kalman2d's
    // bias-corrected rate estimate). pid_step just multiplies it by kd.
    float integral = 0.0f;
    bool saturated = false;
    tvc::PidResult r = tvc::pid_step(integral, saturated, /*x_hat=*/0.0f, /*d_error=*/50.0f,
                                      /*dt=*/0.1f, /*kp=*/0.0f, /*ki=*/0.0f, /*kd=*/1.0f,
                                      /*integral_clamp=*/1000.0f, /*max_deflection=*/1000.0f);
    TEST_ASSERT_EQUAL_FLOAT(50.0f, r.u_raw);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pure_proportional);
    RUN_TEST(test_output_clamps_and_flags_saturated);
    RUN_TEST(test_anti_windup_freezes_integral_while_saturated);
    RUN_TEST(test_integral_clamp_bounds_state_even_when_not_saturated);
    RUN_TEST(test_derivative_uses_supplied_d_error_directly);
    return UNITY_END();
}
