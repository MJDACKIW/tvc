#include <unity.h>

#include <algorithm>
#include <cmath>

#include "lora_link.h"

using namespace tvc::protocol::lora;

void setUp(void) {}
void tearDown(void) {}

// Worked by hand from the SX127x datasheet formula (Section 4.1.1.7), then compared with the
// integer constexpr implementation. 23 B at SF7/BW125 is the commonly quoted 61.7 ms.
void test_hand_checked_airtimes(void) {
    TEST_ASSERT_EQUAL_UINT32(61696, timeOnAirUs(23, 7, 125000, 5));
    TEST_ASSERT_EQUAL_UINT32(82176, timeOnAirUs(37, 7, 125000, 5));   // TELEM
    TEST_ASSERT_EQUAL_UINT32(102656, timeOnAirUs(52, 7, 125000, 5));  // STATUS
    TEST_ASSERT_EQUAL_UINT32(41088, timeOnAirUs(37, 7, 250000, 5));
    TEST_ASSERT_EQUAL_UINT32(51328, timeOnAirUs(52, 7, 250000, 5));
    TEST_ASSERT_EQUAL_UINT32(20544, timeOnAirUs(37, 7, 500000, 5));
    TEST_ASSERT_EQUAL_UINT32(25664, timeOnAirUs(52, 7, 500000, 5));
}

// The same formula written the datasheet's way, in doubles, as an independent check of the
// integer arithmetic (the ceiling, the low-data-rate switch, the preamble's 4.25 symbols).
static double referenceUs(int pl, int sf, double bw, int cr_denominator) {
    const double tsym = std::pow(2.0, sf) / bw;
    const int de = tsym > 0.016 ? 1 : 0;
    const double npre = 8 + 4.25;
    const double groups = std::ceil((8.0 * pl - 4.0 * sf + 28 + 16) / (4.0 * (sf - 2 * de)));
    const double npay = 8 + std::max(groups * cr_denominator, 0.0);
    return (npre + npay) * tsym * 1e6;
}

void test_integer_math_matches_the_datasheet_formula_over_a_grid(void) {
    const long bws[] = {125000, 250000, 500000};
    const unsigned pls[] = {1, 10, 23, 37, 52, 58, 100, 255};
    for (int sf = 7; sf <= 12; sf++) {
        for (long bw : bws) {
            for (int cr = 5; cr <= 8; cr++) {
                for (unsigned pl : pls) {
                    const double want = referenceUs(static_cast<int>(pl), sf, bw, cr);
                    const uint32_t got = timeOnAirUs(pl, sf, bw, cr);
                    TEST_ASSERT_TRUE_MESSAGE(std::fabs(want - got) <= 1.0, "airtime differs");
                }
            }
        }
    }
}

void test_low_data_rate_switches_on_above_16_ms_symbols(void) {
    TEST_ASSERT_FALSE(lowDataRate(11, 250000));  // 8.2 ms
    TEST_ASSERT_TRUE(lowDataRate(11, 125000));   // 16.4 ms
    TEST_ASSERT_TRUE(lowDataRate(12, 125000));   // 32.8 ms
    TEST_ASSERT_FALSE(lowDataRate(7, 125000));   // 1.0 ms
}

void test_wider_bandwidth_never_takes_longer(void) {
    for (unsigned pl : {10u, 37u, 52u}) {
        TEST_ASSERT_TRUE(timeOnAirUs(pl, 7, 250000, 5) < timeOnAirUs(pl, 7, 125000, 5));
        TEST_ASSERT_TRUE(timeOnAirUs(pl, 7, 500000, 5) < timeOnAirUs(pl, 7, 250000, 5));
    }
}

// RadioHead's own 4 header bytes are part of what goes on the air.
void test_frame_airtime_includes_the_radiohead_header(void) {
    TEST_ASSERT_EQUAL_UINT32(timeOnAirUs(33 + kRadioHeadHeaderBytes),
                             frameTimeOnAirUs(sizeof(tvc::protocol::TelemPayload)));
    TEST_ASSERT_EQUAL_UINT32(timeOnAirUs(48 + kRadioHeadHeaderBytes),
                             frameTimeOnAirUs(sizeof(tvc::protocol::StatusPayload)));
    TEST_ASSERT_EQUAL_UINT32(timeOnAirUs(37), kTelemAirUs);
    TEST_ASSERT_EQUAL_UINT32(timeOnAirUs(52), kStatusAirUs);
}

// Whatever params.yaml holds, the downlink must fit in a second, or TELEM cannot keep up.
void test_current_params_leave_the_channel_some_room(void) {
    TEST_ASSERT_TRUE(kDownlinkDutyPermille < 1000);
}

// STATUS rides on the TELEM grid (tx_slots.h): at the configured settings it must start after
// the TELEM frame and finish inside the period. Also pins the numbers the scheduler is built
// from at 500 kHz, the same ones tools/lora_airtime.py prints.
void test_schedule_constants_at_500_khz(void) {
    constexpr uint32_t telem = timeOnAirUs(37, 7, 500000, 5);
    constexpr uint32_t status = timeOnAirUs(52, 7, 500000, 5);
    TEST_ASSERT_EQUAL_UINT32(20544, telem);
    TEST_ASSERT_EQUAL_UINT32(25664, status);
    TEST_ASSERT_EQUAL_UINT32(22, ceilMs(kTxStartCostUs + telem));       // STATUS offset
    TEST_ASSERT_EQUAL_UINT32(28, ceilMs(kTxStartCostUs + status) + 1);  // STATUS cost
}

void test_status_fits_in_the_gap_at_the_configured_settings(void) {
    TEST_ASSERT_TRUE(kStatusOffsetMs + kStatusCostMs < kTelemPeriodMs);
    TEST_ASSERT_EQUAL_UINT32(10, kStatusEveryNTelem);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_hand_checked_airtimes);
    RUN_TEST(test_integer_math_matches_the_datasheet_formula_over_a_grid);
    RUN_TEST(test_low_data_rate_switches_on_above_16_ms_symbols);
    RUN_TEST(test_wider_bandwidth_never_takes_longer);
    RUN_TEST(test_frame_airtime_includes_the_radiohead_header);
    RUN_TEST(test_current_params_leave_the_channel_some_room);
    RUN_TEST(test_schedule_constants_at_500_khz);
    RUN_TEST(test_status_fits_in_the_gap_at_the_configured_settings);
    return UNITY_END();
}
