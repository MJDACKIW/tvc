#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <stddef.h>

#include "params.h"
#include "protocol.h"

// Ground radio <-> USB CDC bridge (SPEC.md Section 4.8). Stateless: no sequencing, no command
// dispatch, no state machine -- "no logic lives here". It only does two things:
//   downlink (vehicle -> LoRa -> here): forward the frame RadioHead handed back, verbatim,
//     with 2 raw bytes (RSSI, SNR) appended, over USB.
//   uplink (USB -> here -> vehicle): forward bytes read from USB over LoRa, once a complete
//     frame (protocol.h's sync/len/type/payload/crc16) has arrived -- this file only needs
//     frame *boundaries* to know how many bytes make one LoRa send(), it does not interpret
//     payloads or verify the CRC (that's the vehicle's and the ground tool's job).
// Pins per SPEC.md Section 4.8: CS 8 / INT 3 / RST 4 (Adafruit Feather M0 RFM95, #3178).
namespace {

constexpr int kRfm95Cs = 8;
constexpr int kRfm95Int = 3;
constexpr int kRfm95Rst = 4;

RH_RF95 g_rf95(kRfm95Cs, kRfm95Int);

constexpr size_t kUplinkBufSize =
    tvc::protocol::kHeaderLen + tvc::protocol::kMaxPayload + tvc::protocol::kCrcLen;
uint8_t g_uplink_buf[kUplinkBufSize];
size_t g_uplink_len = 0;

// Byte-at-a-time so a slow or fragmented USB write never stalls the LoRa side waiting for a
// full frame in one read(). Sends the instant a complete frame has arrived.
void feedUplinkByte(uint8_t b) {
    if (g_uplink_len == 0 && b != tvc::protocol::kSyncByte0) return;  // resync on sync0
    if (g_uplink_len == 1 && b != tvc::protocol::kSyncByte1) {
        g_uplink_len = 0;
        return;
    }
    g_uplink_buf[g_uplink_len++] = b;
    if (g_uplink_len < tvc::protocol::kHeaderLen) return;  // still need len+type

    uint8_t payload_len = g_uplink_buf[2];
    size_t total = tvc::protocol::frameLen(payload_len);
    if (total > kUplinkBufSize) {  // garbled length byte -- can't be a real frame, drop and resync
        g_uplink_len = 0;
        return;
    }
    if (g_uplink_len < total) return;

    g_rf95.send(g_uplink_buf, total);
    g_rf95.waitPacketSent();
    g_uplink_len = 0;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }

    pinMode(kRfm95Rst, OUTPUT);
    digitalWrite(kRfm95Rst, HIGH);
    digitalWrite(kRfm95Rst, LOW);
    delay(10);
    digitalWrite(kRfm95Rst, HIGH);
    delay(10);

    if (!g_rf95.init()) {
        Serial.println("RF95 init failed");
        while (1) {
        }
    }
    g_rf95.setFrequency(tvc::params::telemetry::lora_freq_mhz);
    g_rf95.setTxPower(static_cast<int8_t>(tvc::params::telemetry::tx_power_dbm), false);
    g_rf95.setSpreadingFactor(static_cast<uint8_t>(tvc::params::telemetry::lora_sf));
    g_rf95.setSignalBandwidth(static_cast<long>(tvc::params::telemetry::lora_bw_khz) * 1000L);
    g_rf95.setCodingRate4(static_cast<uint8_t>(tvc::params::telemetry::lora_cr));
}

void loop() {
    if (g_rf95.available()) {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 2];
        uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
        if (g_rf95.recv(buf, &len)) {
            buf[len] = static_cast<uint8_t>(static_cast<int8_t>(g_rf95.lastRssi()));
            buf[len + 1] = static_cast<uint8_t>(static_cast<int8_t>(g_rf95.lastSNR()));
            Serial.write(buf, len + 2);
        }
    }

    while (Serial.available()) {
        feedUplinkByte(static_cast<uint8_t>(Serial.read()));
    }
}
