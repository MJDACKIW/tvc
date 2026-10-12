# BRINGUP.md: wire-check, flash, test, dry run

Run top to bottom from the repo root, using only `./tvc`. Every command prints PASS/FAIL lines
and, after a FAIL, one `next:` line. No igniter until a clean dry run: use the 1 kOhm + LED.

1. **Setup (once).** Python 3.11+ and PlatformIO at `~/.platformio/penv/bin/pio`.
   ```bash
   python3 -m venv ground/venv && ground/venv/bin/python -m pip install -r ground/requirements.txt
   ```
   Keep it `ground/venv`, never a dot-folder: the repo is iCloud-synced and dot-folders get the
   macOS hidden flag, which stops Qt loading its plugins.

2. **Wire-check, power off.** Igniter out, LED + 1 kOhm on the pyro output, arm switch open.
   Teensy: servos 3 (X) and 5 (Y), HX711 6/7, RFM95W CS 10, IRQ 8, **RST 9** (24 is encoder #2's
   SCL), SPI 11/12/13, pyro gate 14 (pulldown), arm switch 15, continuity A14, MPU-6050 18/19,
   AS5600 #1 17/16, #2 25/24, LEDs 20-22, battery A16. Antennas on both radios.

3. **Boards visible:** `./tvc ports` (both PASS).

4. **Tests:** `./tvc test` ends with `ALL TESTS PASSED` (about 1 minute).

5. **Flash:** `./tvc flash all`. The Teensy sweeps its servos on every boot, so clear the linkage.
   It reads the Teensy's firmware hash back and compares it to `HEAD` (commit first: `-dirty`
   fails). The Feather cannot report a hash; bossac verifies the upload and the tool stamps it.

6. **SELFTEST:** `./tvc selftest` with the stand still. One line each for IMU, SD, flash, radio
   init, encoders, servos. A `MEASURE values still unset` WARN is expected until step "MEASURE".

7. **Radio, 60 s:** `./tvc link` with the Feather where it will sit during the fire. Receive-only.
   PASS is loss under 2%; it also prints RSSI/SNR, CRC errors, and vehicle loop overruns.

8. **Ground station:** `./tvc gs --sim` to rehearse, then `./tvc gs --live`.

9. **Dry run (USB, bench only; never use USB during a firing):** `./tvc monitor teensy`, then type
   `PROFILE 0`, `DRY_RUN ON`, close the arm switch, `ARM`, `COUNTDOWN`, and
   `FIRE_CONFIRM <token>` (the token COUNTDOWN prints; send it during the 10 s count). Expect
   `>> IGNITE`, the LED on for about 0.8 s, `>> BURN`, `>> TAILOFF`, `>> SAFE` about 4 s after T-0.
   `RESET` returns to IDLE; Ctrl-C leaves the monitor. ARM is refused while MEASURE values are unset.

10. **Readiness:** `./tvc ready`. Step 9 is MANUAL by design, and the Kalman line is permanently
    RED until the sim branch is merged. Exit 0 only when nothing is RED or MANUAL.

## If X fails

| You see | Likely cause | Do this |
|---|---|---|
| `Feather not detected: try double-tap reset` | charge-only cable, hung board, or the bridge halted on `RF95 init failed` | Double-tap RESET (two taps within half a second, LED pulses slowly); `./tvc ports` should say "in its bootloader"; then `./tvc flash feather`. Try another cable. |
| `... is busy: held by <program>` | another program has the port | Close it. `./tvc flash` closes `screen`, `minicom`, `./tvc monitor` itself; close the GUI window by hand. `lsof /dev/cu.usbmodem*` names the holder. |
| `radio init` FAIL in SELFTEST | RFM95W silent on SPI: wiring, 3.3 V, or RST still on pin 24 | Fix it, then **power-cycle the Teensy**; radio init runs only at boot. |
| `no TELEM packets` | vehicle off, radio init failed, Feather in bootloader, or the two sides built from different `params.yaml` | Check `./tvc selftest` and `./tvc ports`; `./tvc flash all` after any `params.yaml` edit. |
| `loss N% (limit 2%)` | weak link, or saturation: 10 Hz TELEM + 1 Hz STATUS at SF7/BW125 is about 90% airtime (computed, not measured) | Read RSSI/SNR; antennas vertical, clear of metal. To test saturation set `telemetry.telem_rate_hz: 5`, then `./tvc flash all` and `./tvc link`. |
| `no Python with PyYAML found` | `ground/venv` missing | Step 1. `TVC_PYTHON` overrides the interpreter. |

## MEASURE

ARM waits for five values in `params.yaml`: `servo.us_per_deg_x`, `servo.us_per_deg_y`,
`servo.neutral_us_x`, `servo.neutral_us_y` (from `servo_ident`: `cd firmware &&
~/.platformio/penv/bin/pio run -e teensy_test_servo -t upload`) and
`loadcell.hx_scale_counts_per_n` (tared counts under a known weight, divided by kg x 9.81). Then
`./tvc flash teensy`. `./tvc ready` lists the rest separately; STAND_OPEN does not need them.

## Known gaps (tools cannot fix these)

- **Radio uplink is ignored.** The vehicle acts on HEARTBEAT only: ARM and ABORT over radio do
  nothing and are never ACKed (SPEC.md 4.7). ABORT works from the USB console only.
- **Heartbeat loss never goes to SAFE:** `msSinceLastHeartbeat()` has no caller (SPEC.md 4.4, 8).
- **No hardware watchdog** (see the comment at the top of `sequencer.cpp`).
- **Radio TX blocks the flight loop**, roughly 80 ms per TELEM by the LoRa formula, with no
  timeout in RadioHead: an IRQ wire on the wrong pin hangs the vehicle at its first send.
- **Filter:** firmware runs the 1-state Kalman; paper and sim validated the 2-state. Not merged.
