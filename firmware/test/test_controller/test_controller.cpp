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
        /*q=*/1.0f, /*r=*/0.1f,
        /*slew_deg_per_s=*/500.0f,
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

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_step_wires_kalman_pid_and_limiter_together);
    RUN_TEST(test_gated_out_step_is_gyro_only);
    return UNITY_END();
}
