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
// one transmission: TELEM when its timer is due, otherwise a pending EVENT copy, otherwise
// STATUS when its timer is due. Nothing here waits for the radio, and nothing is queued behind
// a frame in flight:
//  - a TELEM that falls due while a frame is still on the air is skipped and counted
//    (txSkipped()); the next one comes a period later;
//  - an EVENT copy or a STATUS that finds the radio busy simply stays due and goes out on the
//    first tick after the radio is free (it is rebuilt from live state at that moment, so it
//    is never stale). Without that, STATUS would lose to the TELEM that falls due in the same
//    millisecond at every whole second and never be sent.
// Most calls do no send at all; this is cheap to call unconditionally.
void onLoopTick();

// Schedules an EVENT frame for now and two retransmits at +150 ms and +300 ms (SPEC 4.7:
// "retransmitted 3x"), all sent from onLoopTick() as the radio frees up; the first copy goes
// out inside this call if the radio is idle. event_code values: protocol/event_codes.h.
void onEvent(uint8_t event_code, int16_t aux);

// TELEM frames skipped because a frame was still in flight when they fell due, and stuck
// transmissions cleared by the radio's timeout, since boot or the last clearStats().
uint32_t txSkipped();
uint32_t txTimeouts();
void clearStats();

}  // namespace tvc::flight::telemetry
