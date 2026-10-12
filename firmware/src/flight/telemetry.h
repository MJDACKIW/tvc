#pragma once

#include <stdint.h>

#include "estimator.h"
#include "record.h"

// Builds and sends TELEM/STATUS/EVENT frames (SPEC.md Section 4.7) over radio.h. Pulls
// everything it needs from the other flight/ modules itself (imu, encoders, loadcell, the
// logging modules, radio, and sequencer's own getters) rather than taking a long parameter
// list, the same way sequencer.cpp's buildAndLogRecord() already gathers Record's fields.
namespace tvc::flight::telemetry {

void init();  // radio::begin() + resets the STATUS cumulative accumulators (since boot)

// Call every control-rate (150 Hz) sub-step (SPEC 4.2), right where the shadow controller
// produces this tick's output for each axis. Updates running IAE, peak tilt, and saturation-%
// (STATUS) and caches K/gate/saturation (TELEM) for the next send.
void onControlTick(const estimator::ShadowOut& pitch, const estimator::ShadowOut& yaw);

// Call every loop-rate (300 Hz) tick (SPEC 4.2). Polls for uplink frames, services pending
// EVENT retransmits, and sends TELEM/STATUS when their timers are due. Most calls do none of
// the sends; this is cheap to call unconditionally.
void onLoopTick();

// Sends an EVENT frame immediately and schedules two retransmits (SPEC 4.7: "retransmitted
// 3x"), serviced from onLoopTick(). event_code values: protocol/event_codes.h.
void onEvent(uint8_t event_code, int16_t aux);

}  // namespace tvc::flight::telemetry
