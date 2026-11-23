#include "radio.h"

#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <string.h>

#include "lora_link.h"
#include "params.h"
#include "pins.h"

namespace tvc::flight::radio {
namespace {

RH_RF95 g_rf95(pins::kRfm95Cs, pins::kRfm95Irq);

int8_t g_last_uplink_rssi = -128;
uint32_t g_last_heartbeat_ms = 0;
bool g_any_heartbeat = false;
bool g_radio_ok = false;  // rf95.init() succeeded; every send/receive below is skipped if not

bool g_tx_in_flight = false;
uint32_t g_tx_start_us = 0;
uint32_t g_tx_timeout_us = 0;
uint32_t g_tx_timeouts = 0;

// A healthy TX ends within a few hundred microseconds of its airtime. Twice the airtime plus
// 10 ms leaves room for that without letting a dead IRQ line hold the radio for long.
constexpr uint32_t kTxTimeoutMarginUs = 10000;

constexpr uint32_t txTimeoutUs(uint8_t payload_len) {
    return 2 * protocol::lora::frameTimeOnAirUs(payload_len) + kTxTimeoutMarginUs;
}

}  // namespace

bool begin() {
    pinMode(pins::kRfm95Rst, OUTPUT);
    digitalWrite(pins::kRfm95Rst, HIGH);
    digitalWrite(pins::kRfm95Rst, LOW);
    delay(10);
    digitalWrite(pins::kRfm95Rst, HIGH);
    delay(10);

    g_tx_in_flight = false;
    g_radio_ok = g_rf95.init();
    if (!g_radio_ok) return false;
    protocol::lora::configure(g_rf95);
    return true;
}

bool busy() {
    if (!g_tx_in_flight) return false;
    // The TX-done interrupt runs RH_RF95::handleInterrupt(), which sets the mode back to idle.
    if (g_rf95.mode() != RHGenericDriver::RHModeTx) {
        g_tx_in_flight = false;
        return false;
    }
    if (micros() - g_tx_start_us > g_tx_timeout_us) {
        g_rf95.setModeIdle();  // abandon the stuck frame; the transmitter is switched off
        g_tx_in_flight = false;
        g_tx_timeouts++;
        return false;
    }
    return true;
}

SendResult send(tvc::protocol::PacketType type, const uint8_t* payload, uint8_t payload_len) {
    if (!g_radio_ok) return SendResult::kRadioDown;
    if (payload_len > tvc::protocol::kMaxPayload) return SendResult::kOversize;
    // RadioHead's own send() calls waitPacketSent(), which spins until the TX-done interrupt
    // and has no timeout. busy() is false here, so that wait is already over and never runs.
    if (busy()) return SendResult::kBusy;

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
    // RH_RF95::send() returns once the transmitter is started. It only fails on an oversize
    // frame, which the checks above rule out, so a false here means the module is not
    // answering: report it as down for this call and leave nothing marked in flight.
    if (!g_rf95.send(frame, total)) return SendResult::kRadioDown;
    g_tx_start_us = micros();
    g_tx_timeout_us = txTimeoutUs(payload_len);
    g_tx_in_flight = true;
    return SendResult::kStarted;
}

bool pollUplink() {
    // available() returns false while the radio is transmitting, so this cannot cut a TX short.
    if (!g_radio_ok || !g_rf95.available()) return false;

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

uint32_t txTimeouts() { return g_tx_timeouts; }

void clearStats() { g_tx_timeouts = 0; }

}  // namespace tvc::flight::radio
