#pragma once

#include <stdint.h>

#include "estimator.h"
#include "record.h"

// Builds and sends TELEM/STATUS/EVENT frames (SPEC.md Section 4.7) over radio.h. Pulls
// everything it needs from the other flight/ modules itself (imu, encoders, loadcell, the
// logging modules, radio, and sequencer's own getters) rather than taking a long parameter
// list, the same way sequencer.cpp's buildAndLogRecord() already gathers Record's fields.
namespace tvc::flight::telemetry {

// radio::begin() + resets the STATUS cumulative accumulators (since boot). Returns radio::begin()'s
// result so SELFTEST can report it.
bool init();

// Call every control-rate (150 Hz) sub-step (SPEC 4.2), right where the shadow controller
// produces this tick's output for each axis. Updates running IAE, peak tilt, and saturation-%
// (STATUS) and caches K/gate/saturation (TELEM) for the next send.
void onControlTick(const estimator::ShadowOut& pitch, const estimator::ShadowOut& yaw);

// Call every loop-rate (300 Hz) tick (SPEC 4.2). Polls for uplink frames, then starts at most
// one transmission, chosen by tx_slots.h:
//  - TELEM on its fixed grid. One that falls due while a frame is still on the air is skipped
//    and counted (txSkipped()), never queued.
//  - an EVENT copy, then STATUS, but only in the gap after a TELEM frame and only if the frame
//    will have left the air before the next TELEM slot. STATUS falls due a fixed offset after
//    its TELEM slot (about 22 ms at BW500) instead of in the same millisecond as the next
//    TELEM. A frame that does not fit stays due and goes in the next gap; it is built from live
//    state when it is sent, so it is never stale.
// Nothing here waits for the radio. Most calls do no send at all; cheap to call unconditionally.
void onLoopTick();

// Schedules an EVENT frame for now and two retransmits at +150 ms and +300 ms (SPEC 4.7:
// "retransmitted 3x"). Nothing is sent from this call; onLoopTick() sends each copy in the
// first gap after a TELEM frame once the copy is due, normally within one TELEM period.
// event_code values: protocol/event_codes.h.
void onEvent(uint8_t event_code, int16_t aux);

// TELEM frames skipped because a frame was still in flight when they fell due, and stuck
// transmissions cleared by the radio's timeout, since boot or the last clearStats().
uint32_t txSkipped();
uint32_t txTimeouts();
void clearStats();

}  // namespace tvc::flight::telemetry
