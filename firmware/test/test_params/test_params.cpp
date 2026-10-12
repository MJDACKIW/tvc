#include <unity.h>

#include <cmath>
#include <cstring>

#include "params.h"

void setUp(void) {}
void tearDown(void) {}

void test_non_measure_values_match_yaml(void) {
    TEST_ASSERT_EQUAL_INT(1, tvc::params::schema);
    TEST_ASSERT_EQUAL_INT(150, tvc::params::control::rate_hz);
    TEST_ASSERT_EQUAL_INT(300, tvc::params::control::loop_hz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, tvc::params::control::ki);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, tvc::params::control::max_deflection_deg);
    TEST_ASSERT_EQUAL_FLOAT(0.6f, tvc::params::kalman::accel_gate_g[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.4f, tvc::params::kalman::accel_gate_g[1]);
    TEST_ASSERT_EQUAL_STRING("Estes E12-4", tvc::params::motor::name);
}

void test_measure_fields_are_nan(void) {
    TEST_ASSERT_TRUE(std::isnan(tvc::params::control::kp));
    TEST_ASSERT_TRUE(std::isnan(tvc::params::control::kd));
    TEST_ASSERT_TRUE(std::isnan(tvc::params::kalman::q));
    TEST_ASSERT_TRUE(std::isnan(tvc::params::kalman::r));
    TEST_ASSERT_TRUE(std::isnan(tvc::params::vehicle::mass_kg));
    TEST_ASSERT_TRUE(std::isnan(tvc::params::loadcell::hx_scale_counts_per_n));
}

void test_measure_list_names_every_measure_field(void) {
    TEST_ASSERT_EQUAL_INT(15, tvc::params::kMeasureCount);
    bool found_kp = false;
    bool found_hx_scale = false;
    bool found_batt = false;
    for (int i = 0; i < tvc::params::kMeasureCount; ++i) {
        if (std::strcmp(tvc::params::kMeasureList[i], "control.kp") == 0) {
            found_kp = true;
        }
        if (std::strcmp(tvc::params::kMeasureList[i], "loadcell.hx_scale_counts_per_n") == 0) {
            found_hx_scale = true;
        }
        if (std::strcmp(tvc::params::kMeasureList[i], "power.batt_adc_to_mv") == 0) {
            found_batt = true;
        }
    }
    TEST_ASSERT_TRUE(found_kp);
    TEST_ASSERT_TRUE(found_hx_scale);
    TEST_ASSERT_TRUE(found_batt);
}

// measure_bits::<name> must name the same slot kMeasureList puts that dotted path in --
// that's the whole point of generating both from one list (see gen_params.py).
void test_measure_bits_match_measure_list_order(void) {
    TEST_ASSERT_EQUAL_INT(0, tvc::params::measure_bits::control_kp);
    TEST_ASSERT_EQUAL_INT(13, tvc::params::measure_bits::loadcell_hx_scale_counts_per_n);
    TEST_ASSERT_EQUAL_INT(14, tvc::params::measure_bits::power_batt_adc_to_mv);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_non_measure_values_match_yaml);
    RUN_TEST(test_measure_fields_are_nan);
    RUN_TEST(test_measure_list_names_every_measure_field);
    RUN_TEST(test_measure_bits_match_measure_list_order);
    return UNITY_END();
}
