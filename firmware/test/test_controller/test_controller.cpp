#include <unity.h>

#include <cmath>

#include "controller.h"

void setUp(void) {}
void tearDown(void) {}

// Fixture-only gains (fast, obvious convergence), not the params.yaml flight values.
static tvc::ControlParams fixture_params() {
    return tvc::ControlParams{
        /*dt=*/1.0f / 150.0f,
        /*kp=*/4.0f, /*ki=*/0.0f, /*kd=*/0.5f,
        /*integral_clamp=*/5.0f, /*max_deflection=*/10.0f,
        /*q_angle=*/1.0f, /*q_rate=*/1.0f, /*r=*/0.1f,
        /*slew_deg_per_s=*/500.0f, /*tau_s=*/0.0f,
    };
}

void test_step_wires_kalman_pid_and_limiter_together(void) {
    tvc::AxisState state;
    tvc::ControlParams params = fixture_params();

    tvc::AxisOut out{};
    for (int i = 0; i < 30; ++i) {
        out = tvc::controller_step(state, /*gyro_deg_s=*/0.0f, /*accel_tilt_deg=*/5.0f,
                                    /*accel_gate_ok=*/true, params);
        TEST_ASSERT_TRUE(out.accel_used);
        TEST_ASSERT_TRUE(std::fabs(out.u_cmd) <= params.max_deflection + 1e-6f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 5.0f, out.x_hat);
}

void test_gated_out_step_is_gyro_only(void) {
    tvc::AxisState state;
    tvc::ControlParams params = fixture_params();

    tvc::AxisOut out = tvc::controller_step(state, /*gyro_deg_s=*/50.0f,
                                             /*accel_tilt_deg=*/999.0f,
                                             /*accel_gate_ok=*/false, params);

    TEST_ASSERT_FALSE(out.accel_used);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.K);
    TEST_ASSERT_EQUAL_FLOAT(50.0f * params.dt, out.x_hat);
}

void test_controller_init_sets_asymmetric_p0_and_resets_everything(void) {
    tvc::AxisState state;
    state.x_hat = 3.0f;
    state.bias_hat = 2.0f;
    state.p00 = 7.0f;
    state.p01 = 6.0f;
    state.p10 = 5.0f;
    state.p11 = 4.0f;
    state.integral = 1.0f;
    state.delta = 8.0f;
    state.saturated = true;

    tvc::controller_init(state, /*p0_angle=*/25.0f, /*p0_bias=*/0.01f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.x_hat);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.bias_hat);
    TEST_ASSERT_EQUAL_FLOAT(25.0f, state.p00);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.p01);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.p10);
    TEST_ASSERT_EQUAL_FLOAT(0.01f, state.p11);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.delta);
    TEST_ASSERT_FALSE(state.saturated);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_step_wires_kalman_pid_and_limiter_together);
    RUN_TEST(test_gated_out_step_is_gyro_only);
    RUN_TEST(test_controller_init_sets_asymmetric_p0_and_resets_everything);
    return UNITY_END();
}
