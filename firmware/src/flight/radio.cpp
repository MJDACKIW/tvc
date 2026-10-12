#include "radio.h"

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <string.h>

#include "params.h"
#include "pins.h"

namespace tvc::flight::radio {
namespace {

RH_RF95 g_rf95(pins::kRfm95Cs, pins::kRfm95Irq);

int8_t g_last_uplink_rssi = -128;
uint32_t g_last_heartbeat_ms = 0;
bool g_any_heartbeat = false;

}  // namespace

void begin() {
    pinMode(pins::kRfm95Rst, OUTPUT);
    digitalWrite(pins::kRfm95Rst, HIGH);
    digitalWrite(pins::kRfm95Rst, LOW);
    delay(10);
    digitalWrite(pins::kRfm95Rst, HIGH);
    delay(10);

    g_rf95.init();
    g_rf95.setFrequency(params::telemetry::lora_freq_mhz);
    g_rf95.setTxPower(static_cast<int8_t>(params::telemetry::tx_power_dbm), false);
    g_rf95.setSpreadingFactor(static_cast<uint8_t>(params::telemetry::lora_sf));
    g_rf95.setSignalBandwidth(static_cast<long>(params::telemetry::lora_bw_khz) * 1000L);
    g_rf95.setCodingRate4(static_cast<uint8_t>(params::telemetry::lora_cr));
}

bool send(tvc::protocol::PacketType type, const uint8_t* payload, uint8_t payload_len) {
    if (payload_len > tvc::protocol::kMaxPayload) return false;

    uint8_t frame[tvc::protocol::kHeaderLen + tvc::protocol::kMaxPayload + tvc::protocol::kCrcLen];
    frame[0] = tvc::protocol::kSyncByte0;
    frame[1] = tvc::protocol::kSyncByte1;
    frame[2] = payload_len;
    frame[3] = static_cast<uint8_t>(type);
    memcpy(frame + 4, payload, payload_len);

    uint16_t crc = tvc::protocol::crc16(frame + 2, 2 + payload_len);  // len ++ type ++ payload
    frame[4 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
    frame[4 + payload_len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    size_t total = tvc::protocol::frameLen(payload_len);
    if (!g_rf95.send(frame, total)) return false;
    g_rf95.waitPacketSent();
    return true;
}

bool pollUplink() {
    if (!g_rf95.available()) return false;

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (!g_rf95.recv(buf, &len)) return false;
    if (len < tvc::protocol::kHeaderLen + tvc::protocol::kCrcLen) return false;
    if (buf[0] != tvc::protocol::kSyncByte0 || buf[1] != tvc::protocol::kSyncByte1) return false;

    uint8_t payload_len = buf[2];
    if (len != tvc::protocol::frameLen(payload_len)) return false;

    uint16_t crc_calc = tvc::protocol::crc16(buf + 2, 2 + payload_len);
    uint16_t crc_recv = static_cast<uint16_t>(buf[4 + payload_len]) |
                         (static_cast<uint16_t>(buf[4 + payload_len + 1]) << 8);
    if (crc_calc != crc_recv) return false;

    g_last_uplink_rssi = static_cast<int8_t>(g_rf95.lastRssi());
    auto type = static_cast<tvc::protocol::PacketType>(buf[3]);
    if (type == tvc::protocol::PacketType::kHeartbeat) {
        g_last_heartbeat_ms = millis();
        g_any_heartbeat = true;
    }
    return true;
}

int8_t lastUplinkRssiDbm() { return g_last_uplink_rssi; }

uint32_t msSinceLastHeartbeat() {
    return g_any_heartbeat ? (millis() - g_last_heartbeat_ms) : UINT32_MAX;
}

}  // namespace tvc::flight::radio
