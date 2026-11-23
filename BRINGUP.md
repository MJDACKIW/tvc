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
   PASS is loss under 2%; it also prints the airtime budget, RSSI/SNR, CRC errors and the
   vehicle's overrun count. With the Teensy also plugged into this Mac it clears and then reads
   the vehicle's loop timing: the rate the 300 Hz loop really ran at, the longest tick body
   (`exec_us_max`) and longest tick-to-tick period (`period_us_max`), a histogram of tick body
   times, `tx_skipped` and `tx_timeouts`. Console `STATUS` shows the same fields; `CLEARSTATS`
   zeroes them. WARN lines mean the loop is not holding its rate: fix that before judging the
   radio.

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
| `loss N% (limit 2%)` | weak link, a slow flight loop, or saturation. 10 Hz TELEM + 1 Hz STATUS at SF7 is 23% airtime at BW500 (now), 46% at BW250 (the fallback) and 92% at BW125 (computed by `tools/lora_airtime.py`, not measured) | Read RSSI/SNR; antennas vertical, clear of metal. Check the link report's loop rate first: frames are started and noticed finished only on loop ticks. `tx_skipped` above 0 with a healthy loop means saturation: set `telemetry.lora_bw_khz: 250` only if BW500 loses range, since it is 3 dB more sensitive and 2x the airtime. Flash both boards after editing `params.yaml`. |
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
- **Radio TX is non-blocking, but only checked on the host.** `radio.cpp` starts a frame and
  returns, polls for completion, and clears a stuck one after 2x airtime + 10 ms, so a bad IRQ
  wire costs `tx_timeouts` instead of hanging the vehicle. STATUS and EVENT frames start only in
  the gap after a TELEM frame (`firmware/src/flight/tx_slots.h`, tested against a simulated
  radio at every tick phase). It compiles for both boards; nothing has run on hardware yet.
  `./tvc flash all` then `./tvc link` with the Teensy on USB is the check.
- **The flight loop may not be running at 300 Hz in a degraded bench state.** See "What
  loop_overrun_count counts" below: the last bench run is consistent with about 19 ticks a
  second, each costing about 52 ms. Unverified until `./tvc link` prints the new loop rate.
- **Filter:** firmware runs the 1-state Kalman; paper and sim validated the 2-state. Not merged.

## What `loop_overrun_count` counts

**Exactly one increment for every tick of the loop that starts, whatever the tick costs.** The
rule is `loop_us > 3300` (`countsAsOverrun()` in `firmware/src/flight/loop_stats.h`, applied at
the top of `sequencer::tick()`), where `loop_us` is `main.cpp`'s `g_tick_timer` at the moment
the tick is started: microseconds since that tick's *scheduled* start, measured from the
previous scheduled tick. `main.cpp` starts a tick only when that value has reached one period,
3333 us at 300 Hz, so it is never below 3333 and the test is true every time. The fault bit
(`kLoopOverrun`, bit 7) is therefore set on the first tick after boot and never clears, which is
why `./tvc ready` masks bit 7 out when it looks for hard faults. The count in STATUS is the
same number truncated to 16 bits.

So the counter says how many ticks ran, not how many were late or slow. Over a test window it
equals the tick rate times the window. Two bench runs show it:

| run | overrun count rose by | per second | tick rate it implies |
|---|---|---|---|
| after the first radio change (blocking TX, BW125) | 401 in 60 s | 6.7 | one tick per 150 ms, the time a TELEM or STATUS send held the loop; equal to the 6.7 TELEM/s received, because every tick sent one frame |
| after e30a2b6 (non-blocking TX, BW250, no SD card, state FAULT) | 1155 in 60 s | 19.25 | one tick per 52 ms |

On a healthy 300 Hz loop it would rise by about 18000 in 60 s (the STATUS field is 16 bits, so
it wraps every 218 s). The earlier note here that "every tick counts, which does not match
+401" was half right: the rule does count every tick, and the mismatch was only because the
loop was nowhere near 300 Hz.

**Why `loop_us_max` read 123,622,966 us.** It was not a stale timestamp. When tick bodies cost
more than a period, the schedule only retires one period per tick (`g_tick_timer -= 3333`), so
the distance behind schedule grows without limit, at about (1 - 3.33 / 52) = 0.94 s per second.
A value of 123.6 s fits roughly 132 s of uptime at that rate, which is about the time since
flashing. It is a backlog, not one stall, so telemetry kept flowing, and CLEARSTATS cannot
reset it because the backlog is state in the scheduler. `test_loop_stats` runs this arithmetic.
It also means `Record.loop_us` (16-bit, saturating at 65535) sits pinned at its maximum in the
SD log while the loop is overloaded.

**What costs about 52 ms a tick is not established.** The code path that could do it is I2C:
`imu::read()` and both `encoders::read()` calls go through the Teensy Wire library, which waits
up to 16 ms for the bus to go idle and gives a transaction up to 50 ms before it gives up
(`WireIMXRT.cpp`). One missing or unpowered device costing a 50 ms timeout per tick, plus about
2 ms of normal work, would give 52 ms. SD logging costs nothing with no card (`push()` returns
at once) and the HX711 read does not wait. This is a hypothesis from reading the code: the
histogram `./tvc link` now prints will show whether ticks are slow (it should read 100% in the
">=10 ms" bin if so), and `./tvc selftest` says which device is missing.

**The 53 skipped TELEMs in that run fit the same slow loop.** A host simulation of the
scheduler as it was in e30a2b6 skips exactly one TELEM per second whenever ticks are 40 to 51.5
ms apart, and none at 3.3 ms ticks: a 52 ms STATUS started on one tick is still on the air at
the next tick, which is where the TELEM slot is noticed. Ticks right at the edge (51.8 to 52.3
ms) skip on some seconds and not others, like the 53 of 60 seen. That is a model, not a
measurement, and the whole-second collision it describes only exists because the loop is slow.
The scheduler now starts STATUS and EVENT only where they finish before the next TELEM slot, and
`test_tx_slots` holds it to zero skips for tick lengths from 3 to 60 ms.

**Why it matters beyond the counter.** `estimator` and the servo slew model assume one control
step every 1/150 s. At 19 ticks a second they get one every 105 ms. Radio frames are also only
started and noticed finished on ticks, so a slow loop spaces them out regardless of airtime.

**Proposed fix, not applied** (the fault bit keeps its meaning for now): SPEC.md 4.2 says "any
tick exceeds 3.3 ms", which reads as the tick *taking* more than 3.3 ms, but the code measures
time since the scheduled start, which cannot be under 3.33 ms. Count a tick as an overrun when
its execution time (now measured as `exec_us`, end minus start of the tick body) is above 3300
us, evaluated when the tick finishes. A healthy loop then reads 0, a loop with one slow tick
in a thousand reads 1 per thousand, and an overloaded loop reads the number of slow ticks.
`behind_us_max` stays as the backlog indicator. About ten lines in `sequencer::tick()` plus
changing `test_current_overrun_rule_counts_every_tick_even_when_healthy`.
