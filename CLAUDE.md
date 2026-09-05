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

## Current phase

Phase 1 (SPEC.md Section 9): repo scaffold, `params.yaml`, `core/`, native unit tests, and
`parity_test.py`. Phases 2-7 (simulation, vehicle firmware, radio, ground GUI, HIL, flight
mode) are not started. Do not jump ahead of the current phase without being asked.

## Style notes
- No em dashes in any generated text (paper, comments, docs): flagged as AI-sounding in prior essay reviews.
