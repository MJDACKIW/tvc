#include <unity.h>

#include <cmath>

#include "kalman2d.h"

void setUp(void) {}
void tearDown(void) {}

void test_predict_advances_angle_only_when_gate_false(void) {
    tvc::Kalman2DState s;
    s.p00 = 1.0f;
    s.p11 = 1.0f;
    tvc::Kalman2DResult r = tvc::kalman2d_update(s, /*gyro=*/10.0f, /*accel=*/999.0f,
                                                  /*dt=*/0.1f, /*q_angle=*/0.001f,
                                                  /*q_rate=*/0.003f, /*r=*/0.03f,
                                                  /*gate_ok=*/false);
    // angle = angle + dt*(gyro - bias) = 0 + 0.1*(10 - 0) = 1.0; bias untouched.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s.angle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, s.bias);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.k0);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 10.0f, r.kal_rate);  // gyro - bias, bias still 0
}

void test_update_converges_to_measurement(void) {
    // Fixture-only Q/R (paper's originals, but fast/obvious convergence is what matters
    // here), zero true rate and zero true bias: angle should converge to z_k. Unlike a
    // 1-state filter, convergence need not be strictly monotonic: with a constant gyro
    // reading of 0 and a constant accel reading, the filter cannot yet distinguish
    // "zero true rate, zero bias" from "true rate equal to some nonzero bias," so
    // bias_hat can wander briefly and cause a small angle overshoot before settling
    // (see the trace this fixture was checked against: overshoots to ~10.017 by step 24,
    // still relaxing back toward 10.0 at step 39). Check eventual convergence instead.
    tvc::Kalman2DState s;
    s.p00 = 1.0f;
    s.p11 = 1.0f;
    const float z_k = 10.0f;

    for (int i = 0; i < 40; ++i) {
        tvc::kalman2d_update(s, 0.0f, z_k, 0.01f, 0.001f, 0.003f, 0.03f, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, z_k, s.angle);
}

void test_bias_estimation_tracks_constant_gyro_bias(void) {
    // True angle stays at 0 (accel always reads 0); gyro reads a constant offset that is
    // pure bias, not real rotation. A working 2-state filter should learn to attribute
    // that offset to bias_hat rather than integrating it into angle_hat forever -- this
    // is the capability a 1-state filter cannot have (see kalman2d.h header comment).
    tvc::Kalman2DState s;
    s.p00 = 1.0f;
    s.p11 = 1.0f;
    const float true_bias = 3.0f;
    const float dt = 0.01f;

    for (int i = 0; i < 2000; ++i) {
        tvc::kalman2d_update(s, /*gyro=*/true_bias, /*accel=*/0.0f, dt, 0.001f, 0.003f,
                              0.03f, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, true_bias, s.bias);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, s.angle);
}

void test_gate_false_holds_p_update_but_still_predicts_variance_growth(void) {
    tvc::Kalman2DState s;
    s.p00 = 0.5f;
    s.p01 = 0.0f;
    s.p10 = 0.0f;
    s.p11 = 0.5f;
    tvc::kalman2d_update(s, 0.0f, 999.0f, 0.1f, 0.001f, 0.003f, 0.03f, false);
    // n00 = p00 + dt*(dt*p11 - p01 - p10 + q_angle), with p01=p10=0:
    // = 0.5 + 0.1*(0.1*0.5 + 0.001) = 0.5 + 0.1*0.051 = 0.5051
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5051f, s.p00);
    // n11 = p11 + q_rate*dt = 0.5 + 0.003*0.1 = 0.5003
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5003f, s.p11);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_predict_advances_angle_only_when_gate_false);
    RUN_TEST(test_update_converges_to_measurement);
    RUN_TEST(test_bias_estimation_tracks_constant_gyro_bias);
    RUN_TEST(test_gate_false_holds_p_update_but_still_predicts_variance_growth);
    return UNITY_END();
}
