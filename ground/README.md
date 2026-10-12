# Ground station (`ground/gs.py`)

SPEC.md Section 5. PyQt6 + pyqtgraph, dark theme, one window.

## Setup

```
/Library/Frameworks/Python.framework/Versions/3.14/bin/python3 -m venv ground/venv
ground/venv/bin/pip install -r ground/requirements.txt
```

Name the venv `ground/venv`, **not** `ground/.venv`. This repo sits in an iCloud-synced
"Documents - <Mac name>" folder, and every file inside a dot-directory there carries the macOS
hidden flag (`ls -lO` shows `hidden`). Qt's plugin scan skips hidden files, so a `.venv` fails at
startup with "Could not find the Qt platform plugin". `gs.py` checks for this and says so.

## Run

```
ground/venv/bin/python ground/gs.py --replay data/synthetic/SF_SYNTH.BIN   # SD log or ground log
ground/venv/bin/python ground/gs.py --sim                                  # spawns run_sim.py --serve
ground/venv/bin/python ground/gs.py --port /dev/tty.usbmodemXXXX           # Feather bridge, live
```

- `--replay FILE` takes a vehicle SD log (`SF_*.BIN`, TVCL header) or a ground log that this GUI
  recorded (`data/ground/*.bin`, TVCG header). An SD log is turned back into the TELEM/STATUS/EVENT
  stream the vehicle would have radioed (`sd_replay.py`), at the vehicle's own 10 Hz TELEM rate by
  default (`--replay-telem-hz 50` for denser plots), and played at real time (`--speed 4` to go
  faster). Nothing is sent in replay; the buttons still show what the vehicle allowed.
- `--sim` connects to `sim/run_sim.py --serve` on port 5760, starting it if nothing is listening.
  `--sim-args --mode STAND_CLOSED --disturbance` passes options through. Started by hand in a
  terminal, the sim also takes `arm open|closed`, `cont ok|open`, `loss 0.1` on stdin.
- Live `--port`: same codec and link code as `--sim`, **not yet run against hardware**. The vehicle
  firmware currently acts on radio `HEARTBEAT` only, so every other command will show as unACKed
  (amber, then "no ACK" in the log) until radio uplink dispatch is added to `sequencer.cpp`.

Space is ABORT. Every session is recorded to `data/ground/` (`--log-dir`, `--no-record`); on SAFE
the run is finalised as `YYYYMMDD_HHMMSS_<mode>_<profile>.{bin,csv,_events.txt}`.

A synthetic SD log for replay, built through the real `record.h`, `core/` and `profiles.cpp`:
`ground/venv/bin/python tools/synth_sd_log.py` (writes `data/synthetic/SF_SYNTH.BIN`).

## Tests

```
ground/venv/bin/python -m pytest ground/tests            # ~50 s, offscreen
TVC_GUI_SHOTS=/tmp/shots ground/venv/bin/python -m pytest ground/tests   # + screenshots
```

`test_telem_codec.py` compiles `protocol/protocol.h` and checks `protocol/telem_codec.py` against
it byte for byte. `test_gui.py` drives the real window through a full `--replay` and a `--sim`
session (every state's enabled buttons, arm switch, heartbeat loss, hold-to-confirm, spacebar
ABORT, closed-loop recovery), then replays the ground log that session recorded.
