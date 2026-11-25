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
ground/venv/bin/python -m pytest ground/tests                              # codec, GUI, serve model, log pipeline
ground/venv/bin/python sim/run_sim.py baseline                             # also legacy_check, diagnose_estimator, ...
ground/venv/bin/python tools/iae_compare.py --self-test                    # or: decode_log.py LOG.BIN --npz x.npz, then iae_compare.py x.npz
```

Use `ground/venv`, never a dot-named venv: this folder is iCloud-synced and files inside
dot-directories get the macOS hidden flag, which stops Qt loading its plugins.

## Execution environment

All phases run locally on this Mac. The repo root is the working directory. `legacy/` is
read-only reference.

## Current phase

The validated 2-state core and the full simulation are merged with the vehicle firmware, radio
and ground GUI (branch `merge/validated-core`). There is one controller and one vehicle model:
`core/` (2-state Kalman, PID, servo lag + slew, seeded by `controller_init`) runs on the Teensy
and, via `sim/tvc_core.py`, in every sim path; `Vehicle`/`StandVehicle` in `sim/vehicle.py` is
the plant for both `run_sim.py` and the GUI's `--sim` (`sim/serve.py`). SPEC.md Section 9
phases 1 to 5 exist; HIL, STAND_CLOSED on hardware and FLIGHT (phases 6 and 7) do not. Do not
jump ahead of what has been explicitly asked.

Current priority: the static-stand delta-IAE study (measured vs. simulated attitude response):
`run_sim.py --stand`, `tools/decode_log.py --npz` into `tools/iae_compare.py`. The free-flight
model (`Vehicle`, `controllability_map`, `monte_carlo`) is kept for the paper.

Open items, stated as measured, not as resolved:
- The baseline does not settle. It never enters the 5% band within the burn (`run_sim.py
  baseline` reports 2.440 s, which is the end of the burn). The P0 fix (asymmetric
  `kalman.p0_angle`/`p0_bias`) cut the steady-state offset (mean true angle over t in
  [1.5, 2.4] s) from -2.97 deg to -0.54 deg. The residual is gyro noise integrated while the
  accelerometer gate is closed; `diagnose_estimator` variant (c) (`bias_hat` pinned to 0, -0.52
  deg) supports that, it does not prove it. Perfect state feedback (variant (b)) settles in
  0.358 s, so SPEC.md Section 8's old "settles < 0.3 s" is not met even with a perfect
  estimator. `legacy_check` (the controller port against the paper's own physics) passes at
  0.22% / 0.20% / 0.22% of peak.
- Without a load cell a real log has no thrust, so `iae_compare.py` cannot find ignition in it
  yet. `decode_log.py` writes `pyro_on_time_s` and leaves `burn_start_time_s` NaN (it will come
  from the phototransistor); nothing reads either key yet.
- `./tvc ready` is RED while `pins.h` has the RFM95 reset on 24, which is encoder #2's Wire2
  SCL. Move the wire to 9 and set `kRfm95Rst = 9`.

## Style notes
- No em dashes in any generated text (paper, comments, docs): flagged as AI-sounding in prior essay reviews.
