#pragma once

// Servo slew model applied to the commanded deflection. Paper Section 3.3, 3.6 (60 deg / 0.12 s).
// Same function models the physical actuator in the sim and shapes the command sent to the
// servo in firmware, so the logged command is exactly what the sim predicts.
namespace tvc {

// delta is persistent per-axis state (see controller.h AxisState), updated in place and returned.
float rate_limiter_step(float& delta, float u_cmd, float dt, float slew_deg_per_s);

}  // namespace tvc
