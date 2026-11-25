# TVC Project: CLAUDE.md

Custom Thrust Vector Control (TVC) model rocket. **SPEC.md is the authoritative spec.**
Read it in full before doing anything in this repo. It covers the repo layout, `params.yaml`,
the exact control equations, firmware modes, ground station, simulation, and the phased
build plan. This file is just pointers.

## Bring-up (`./tvc`, see BRINGUP.md for the order)

```
./tvc test            # native tests, parity, codec, GUI offscreen
./tvc ports           # find the Teensy and the Feather
./tvc flash all       # build + upload both, then verify what each reports
./tvc selftest        # SELFTEST over USB, one PASS/FAIL line per check
./tvc link            # 60 s radio test through the Feather
./tvc ready           # pre-fire checklist, GREEN / RED / MANUAL
```

`./tvc` re-runs itself under `ground/venv`. Bare `python3` and PlatformIO's own Python have no
PyYAML; the pio pre-build step finds `ground/venv` by itself (override with `TVC_PYTHON`).

## Build and test by hand (core/ only, no hardware)

```
ground/venv/bin/python tools/gen_params.py  # params.yaml -> core/params.h, sim/tvc_params.py
~/.platformio/penv/bin/pio test -e native   # runs firmware/test/ against core/
ground/venv/bin/python tools/parity_test.py # core/ via ctypes vs. core/ via pio native test
```

`pio` lives at `~/.platformio/penv/bin/pio`; use the full path (SPEC.md Section 10: an
unrelated `.venv` once broke bare `pio` on PATH). `pio test -e native` auto-regenerates
`core/params.h` via a pre-build script, but run `gen_params.py` by hand after editing
`params.yaml` if you need `sim/tvc_params.py` refreshed too.

`core/*.cpp` also builds standalone as a shared library for the Python side:
`make -C core native` (needs clang or another C++17 compiler).

## Ground station and sim (ground/README.md)

```
ground/venv/bin/python ground/gs.py --replay data/synthetic/SF_SYNTH.BIN   # or --sim, or --port
ground/venv/bin/python -m pytest ground/tests                              # codec + GUI acceptance
```

Use `ground/venv`, never a dot-named venv: this folder is iCloud-synced and files inside
dot-directories get the macOS hidden flag, which stops Qt loading its plugins.

## Execution environment

All phases run locally on this Mac. The repo root is the working directory. `legacy/` is
read-only reference.

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
