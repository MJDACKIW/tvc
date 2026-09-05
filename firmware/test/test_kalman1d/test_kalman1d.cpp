#include <unity.h>

#include <cmath>

#include "kalman1d.h"

void setUp(void) {}
void tearDown(void) {}

void test_predict_advances_state(void) {
    float x_hat = 0.0f;
    float P = 1.0f;
    tvc::kalman1d_predict(x_hat, P, 10.0f, 0.1f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, x_hat);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.02f, P);
}

void test_update_converges_to_measurement(void) {
    // Fixture-only Q/R (not params.yaml's MEASURE placeholders) chosen for fast, obvious
    // convergence: trust the accel reading heavily relative to the process model.
    float x_hat = 0.0f;
    float P = 1.0f;
    const float q = 1.0f;
    const float r = 0.1f;
    const float z_k = 10.0f;

    float prev_error = 1e9f;
    for (int i = 0; i < 20; ++i) {
        tvc::kalman1d_predict(x_hat, P, 0.0f, 0.01f, q);
        tvc::kalman1d_update(x_hat, P, z_k, r, true);
        float error = std::fabs(z_k - x_hat);
        TEST_ASSERT_TRUE(error <= prev_error);
        prev_error = error;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, z_k, x_hat);
}

void test_gate_false_skips_update(void) {
    float x_hat = 0.0f;
    float P = 1.0f;
    tvc::kalman1d_predict(x_hat, P, 50.0f, 0.1f, 0.02f);
    float x_hat_after_predict = x_hat;
    float P_after_predict = P;

    float K = tvc::kalman1d_update(x_hat, P, 999.0f, 0.5f, false);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, K);
    TEST_ASSERT_EQUAL_FLOAT(x_hat_after_predict, x_hat);
    TEST_ASSERT_EQUAL_FLOAT(P_after_predict, P);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_predict_advances_state);
    RUN_TEST(test_update_converges_to_measurement);
    RUN_TEST(test_gate_false_skips_update);
    return UNITY_END();
}
