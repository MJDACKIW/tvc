#pragma once

// Servo model: first-order lag (tau_s) plus a hard slew-rate limit. Paper Section 3.3,
// 3.6 (60 deg / 0.12 s) characterized the MG90S by slew rate alone; tau_s adds the
// actuator's own response lag, so a small correction settles with a first-order decay
// instead of slewing at full rate right up to the target and then instantly stopping.
// Same function models the physical actuator in the sim and shapes the command sent to
// the servo in firmware, so the logged command is exactly what the sim predicts.
namespace tvc {

// delta is persistent per-axis state (see controller.h AxisState), updated in place and
// returned. Each step moves delta toward u_cmd by whichever is smaller: the first-order
// lag's implied step (1 - exp(-dt/tau_s)) * (u_cmd - delta), or the slew limit
// slew_deg_per_s * dt. Large errors are slew-limited (matches the pre-lag model's
// large-signal behavior exactly); small errors near the target decay like a first-order
// system, which the slew-only model could not represent. tau_s -> 0 recovers the
// pre-lag model exactly, since the lag step then always exceeds the slew step.
float servo_step(float& delta, float u_cmd, float dt, float tau_s, float slew_deg_per_s);

}  // namespace tvc
