# TVC Project: CLAUDE.md

Custom Thrust Vector Control (TVC) model rocket. **SPEC.md is the authoritative spec.**
Read it in full before doing anything in this repo. It covers the repo layout, `params.yaml`,
the exact control equations, firmware modes, ground station, simulation, and the phased
build plan. This file is just pointers.

## Build and test (Phase 1: core/ only, no hardware)

```
python3 tools/gen_params.py                 # params.yaml -> core/params.h, sim/tvc_params.py
~/.platformio/penv/bin/pio test -e native   # runs firmware/test/ against core/
python3 tools/parity_test.py                # core/ via ctypes vs. core/ via pio native test
```

`pio` lives at `~/.platformio/penv/bin/pio`; use the full path (SPEC.md Section 10: an
unrelated `.venv` once broke bare `pio` on PATH). `pio test -e native` auto-regenerates
`core/params.h` via a pre-build script, but run `gen_params.py` by hand after editing
`params.yaml` if you need `sim/tvc_params.py` refreshed too.

`core/*.cpp` also builds standalone as a shared library for the Python side:
`make -C core native` (needs clang or another C++17 compiler).

## Execution environment

Claude Code sessions on this repo may run in an ephemeral remote container, not on the
Mac. `core/` and its native tests and parity check are container-safe (no hardware). But
Phases 3+ (uploading to the Teensy 4.1 or Feather M0, radio soak tests, bench runs, anything
that touches real hardware) need the actual Mac: the boards, the USB ports, and a local
`~/.platformio/penv/bin/pio` are not reachable from a remote session. Confirm you are on the
Mac, with `pio` installed at that path, before starting those phases.

## Current phase

Phase 1 (core/, native tests, parity) is complete. Phase 2 (`sim/`) is substantially
built: `run_sim.py`'s free-flight scenarios (baseline, open_loop, disturbance,
controllability_map, monte_carlo), `--legacy-physics` regression mode, and a
`diagnose_estimator` scenario. Ahead of the strict phase order (SPEC.md Section 9 lists
these under Phase 6), the static-stand path also exists: `run_sim.py --stand`
(`StandVehicle`, no aero, mechanical friction), `--thrust-from-log`/`--theta0-from-log`
to drive it from a decoded log, and `tools/iae_compare.py` (Section 7) with a
`--self-test` mode that proves the pipeline against a synthetic log before real stand
data exists. `core/`'s servo model now includes a first-order lag (`servo.tau_s`)
alongside the slew limit, shared by both vehicle models and the stand.

Current priority: the sim exists primarily to support the static-stand delta-IAE study
(measured vs. simulated attitude response, `tools/iae_compare.py`), not free-flight
physics; the free-flight model (`Vehicle`, `controllability_map`, `monte_carlo`) is kept
for the paper but is not the active focus. Phases 3-5 and 7 (vehicle firmware, radio,
ground GUI, flight mode) are not started; do not jump ahead of what's been explicitly
asked.

Resolved: the baseline scenario's failure to settle within the standard 5% band was
root-caused (see `run_sim.py diagnose_estimator`) to the Kalman filter's P0
initialization giving the bias state as much initial uncertainty as the angle state,
letting the first accelerometer correction after ignition misattribute part of a large
initial-angle error to `bias_hat`. Fixed via `core/controller.h`'s `controller_init`
(asymmetric P0: `kalman.p0_angle`/`p0_bias` in params.yaml, small for bias, large for
angle), the one place both sim and firmware now seed `AxisState`. Verified: the
baseline's steady-state drift dropped from ~2.9 deg to ~0.5 deg (matching the
bias-frozen diagnostic variant almost exactly); the remaining ~0.5 deg is gyro white
noise integrated over the long predict-only stretch while the accelerometer gate is
closed, which is intrinsic to gyro-only dead reckoning and not fixable by
initialization. `--legacy-physics` keeps the paper's original symmetric P0 = I
(`sim_overrides.legacy_physics.p0_angle`/`p0_bias`, both 1.0) since its own gate never
closes for an extended stretch and was never exposed to this failure mode; `legacy_check`
still passes at the same <1% (in fact unchanged: 0.22%/0.20%/0.22%).

## Style notes
- No em dashes in any generated text (paper, comments, docs): flagged as AI-sounding in prior essay reviews.
