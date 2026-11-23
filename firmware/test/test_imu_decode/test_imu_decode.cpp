#include <unity.h>

#include <stdint.h>

#include "../../src/flight/imu_decode.h"

using namespace tvc::flight::imu;

void setUp(void) {}
void tearDown(void) {}

// Every word gets a distinct value whose two bytes differ, so a swapped byte order, a wrong
// word index or the temperature leaking into the gyro all change the answer.
void test_burst_layout_accel_temperature_gyro(void) {
    const uint8_t raw[kBurstBytes] = {
        0x12, 0x34,  // accel x = 0x1234
        0x56, 0x78,  // accel y = 0x5678
        0x9A, 0xBC,  // accel z = 0x9ABC (negative)
        0x0F, 0xED,  // temperature, dropped
        0xFF, 0x38,  // gyro x = -200
        0x00, 0x64,  // gyro y = 100
        0x80, 0x01,  // gyro z = -32767
    };
    int16_t a[3], g[3];
    decodeBurst(raw, a, g);
    TEST_ASSERT_EQUAL_INT16(0x1234, a[0]);
    TEST_ASSERT_EQUAL_INT16(0x5678, a[1]);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(0x9ABC), a[2]);
    TEST_ASSERT_EQUAL_INT16(-200, g[0]);
    TEST_ASSERT_EQUAL_INT16(100, g[1]);
    TEST_ASSERT_EQUAL_INT16(-32767, g[2]);
}

void test_big_endian_word(void) {
    const uint8_t raw[kBurstBytes] = {0x01, 0x02};
    TEST_ASSERT_EQUAL_INT16(0x0102, beWord(raw, 0));
    TEST_ASSERT_EQUAL_INT16(0, beWord(raw, 1));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_burst_layout_accel_temperature_gyro);
    RUN_TEST(test_big_endian_word);
    return UNITY_END();
}
