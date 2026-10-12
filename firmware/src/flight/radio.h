#pragma once

#include <stdint.h>

#include "protocol.h"

// RFM95W driver for the vehicle (Teensy) side of the telemetry link (SPEC.md Section 4.5 for
// pins, Section 4.7 for the frame format, Section 4.8 for the Feather bridge on the other end).
// RadioHead RH_RF95, the same library the Feather bridge and the archived legacy/ radio tests
// use.
namespace tvc::flight::radio {

void begin();  // RST pulse + rf95.init() + frequency/SF/BW/CR/power from params::telemetry::*

// Wraps payload in the shared frame (sync/len/type/crc16, protocol.h) and transmits over LoRa.
// Blocks until the send completes (RadioHead's own waitPacketSent()); payload_len must be <=
// protocol::kMaxPayload. Returns false if the payload is oversized or the underlying send()
// call fails.
bool send(tvc::protocol::PacketType type, const uint8_t* payload, uint8_t payload_len);

// Non-blocking: checks for one waiting uplink frame, validates sync/len/crc16, and if valid
// records its RSSI (lastUplinkRssiDbm()) and, for HEARTBEAT specifically, the receive time
// (msSinceLastHeartbeat()). Does not dispatch other uplink command types onto the sequencer
// this phase -- SPEC.md Section 4.7 flags this explicitly rather than silently stubbing it;
// those commands already work from the USB console (console.cpp) in the meantime. Returns true
// if a valid frame was received (regardless of its type).
bool pollUplink();

int8_t lastUplinkRssiDbm();       // -128 ("unknown") until the first valid uplink frame arrives
uint32_t msSinceLastHeartbeat();  // UINT32_MAX if no HEARTBEAT has ever been received

}  // namespace tvc::flight::radio
