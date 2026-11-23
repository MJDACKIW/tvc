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
   Teensy: servos 3 (X) and 5 (Y), HX711 6/7 (the load cell is dropped: optional), RFM95W CS 10,
   IRQ 8, **RST 9 in SPEC.md, but wired to 24 on this build**, SPI 11/12/13, pyro gate 14
   (pulldown), arm switch 15, continuity A14, MPU-6050 18/19, AS5600 #1 17/16, #2 25/24 (optional),
   LEDs 20-22, battery A16. Antennas on both radios. Pin 24 is also encoder #2's SCL: while the
   radio's RST wire is there, encoder #2 cannot work and the firmware leaves Wire2 alone.

3. **Boards visible:** `./tvc ports` (both PASS).

4. **Tests:** `./tvc test` ends with `ALL TESTS PASSED` (about 1 minute).

5. **Flash:** `./tvc flash all`. The Teensy sweeps its servos on every boot, so clear the linkage.
   It reads the Teensy's firmware hash back and compares it to `HEAD` (commit first: `-dirty`
   fails). The Feather cannot report a hash; bossac verifies the upload and the tool stamps it.

6. **SELFTEST:** `./tvc selftest` with the stand still. One line each for IMU, SD, flash, radio
   init, encoders, load cell, servos. The IMU, SD and radio are required (FAIL). The encoders and
   the load cell are optional: absent ones are WARN, are recorded in the presence mask, and are
   never polled again, so a missing one costs the loop nothing. An absent SD card is a FAIL (ARM
   refused) that also costs the loop nothing. A `MEASURE values still unset` WARN is expected
   until step "MEASURE".

7. **Radio, 60 s:** `./tvc link` with the Feather where it will sit during the fire. Receive-only.
   PASS is loss under 2%; it also prints the airtime budget, RSSI/SNR, CRC errors and the
   vehicle's overrun count. With the Teensy also plugged into this Mac it clears and then reads
   the vehicle's loop timing: the rate the 300 Hz loop really ran at, the longest tick body
   (`exec_us_max`) and longest tick-to-tick period (`period_us_max`), a histogram of tick body
   times, the slowest each part of a tick took (`sec_us_max`: IMU, load cell, encoders, log
   push, telemetry), I2C timeout and error counts per bus, which parts were present
   (`present=`), `tx_skipped` and `tx_timeouts`. Console `STATUS` shows the same fields;
   `CLEARSTATS` zeroes them. WARN lines mean the loop is not holding its rate: fix that before judging the
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

ARM waits for four values in `params.yaml`: `servo.us_per_deg_x`, `servo.us_per_deg_y`,
`servo.neutral_us_x`, `servo.neutral_us_y` (from `servo_ident`: `cd firmware &&
~/.platformio/penv/bin/pio run -e teensy_test_servo -t upload`). Then `./tvc flash teensy`.
`./tvc ready` lists the rest separately; STAND_OPEN does not need them. The load cell scale
(`loadcell.hx_scale_counts_per_n`) is no longer one of them: the load cell was dropped.
`servo_ident` needs both encoders; it says so if one does not answer.

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
- **The flight loop ran at about 19 Hz on the bench, and the fixes for it are unverified.** See
  "Per-tick blocking audit" below: absent parts are no longer polled, all I2C in a tick is cut
  off at 1 ms, and `./tvc link` prints the rate the loop really runs at. None of that has run on
  hardware. If the bounded I2C driver misbehaves, build with `-DTVC_I2C_LIBRARY_WIRE` (add it
  to `build_flags` in `firmware/platformio.ini`) to go back to the library's blocking read.
- **No loss of the loop to a stuck bus, but no recovery either.** A bus whose slave holds SDA
  low keeps failing (counted in `i2c_timeouts`, one try per 20 ms). The nine-clock recovery that
  Wire does in its own `wait_idle()` is not in the tick path. A failed IMU read keeps the
  previous sample, silently except for the counter; there is no fault bit for it yet.
- **Without a load cell a real fire cannot confirm ignition.** In IGNITE a non-dry run waits for
  thrust above 1 N (`kThrustDetectN`) and aborts as a misfire after `kNoIgnitionMs`; an absent
  load cell reads 0 N. Dry runs are unaffected (they skip that check). Something else has to
  detect ignition before a live run.
- **Filter:** firmware runs the 1-state Kalman; paper and sim validated the 2-state. Not merged.

## Per-tick blocking audit

What the 300 Hz tick (`sequencer::tick()`) can wait on, with the worst case for each and where the
number comes from. Wire here is `~/.platformio/packages/framework-arduinoteensy/libraries/Wire/
WireIMXRT.cpp`. Nothing below was measured: these are bounds read from the code.

| call in the tick | can block on | worst case | source |
|---|---|---|---|
| `imu::read()` before this change (Wire, LPI2C1, pins 18/19) | bus idle (16 ms), then the transaction (50 ms); `endTransmission`'s error was ignored, so `requestFrom` ran regardless: idle again, then 50 ms | **132.2 ms** (16.1 + 50 + 16.1 + 50) | `WireIMXRT.cpp` 83 (idle 16), 139 (write 50), 158 (idle in `requestFrom`), 196 (read 50); `force_clock()` adds 10 us + up to 9 x 10 us (246-257) |
| `encoders::read()`, each of Wire1 (LPI2C3, 17/16) and Wire2 (LPI2C4, 25/24), before | same, but `readRaw()` stops if `endTransmission` fails | **116.1 ms** each (16.1 + 50 + 50) | same lines, `encoders.cpp` `readRaw` |
| an encoder that is simply not there | the address is not acknowledged, `NDF` is set, `endTransmission` returns 2 at once | about 0.1 ms, **not** a stall | `WireIMXRT.cpp` 134 |
| the three I2C reads together, before | | **364.4 ms** worst case, 52 ms seen on the bench | sum of the above |
| `loadcell::service()` | `ready()` is one GPIO read; only if DOUT is low does `readRaw()` clock 25 bits | about 52 us when it reads, **0 us** with the HX711 absent (DOUT has a pull-up, reads high); it never waits for DOUT | `loadcell.cpp`. The only wait is `tare()`, up to 2000 ms, in SELFTEST and the TARE command |
| `logging_sd::push()` | no card: nothing. With a card: one 18,944 B write per 256 records (0.85 s) | **0 us** with no card; with one, each SDIO wait inside the write is capped at 1 s and a write makes several, so no bound for the call | `logging_sd.cpp` (`if (!g_open) return;`), `TeensySdio.cpp` 36 (`BUSY_TIMEOUT_MICROS = 1000000`) |
| `logging_flash::push()`, every 6th tick | when a LittleFS_Program block fills, a flash sector erase with interrupts off and a status poll with no timeout | no bound in the code; set by the flash chip, **not looked up here** | `eeprom.c` 226-239 (`flash_wait`), 288 (`__disable_irq`) |
| `telemetry::onLoopTick()` radio | one SPI burst to load the FIFO, no waiting | about 0.5 ms (estimate: 40 B at RadioHead's default 1 MHz SPI) | `RHHardwareSPI.h` 32; `radio.cpp` |
| `Serial.printf` in a tick (countdown, abort, peak) | USB TX buffer full and the host not reading | 120 ms, then it gives up until the host reads again; only at those prints, not every tick | `usb_serial.c` 281, 354 |

**What this says about the 52 ms.** The hypothesis that unwired encoders block the Wire library
is not supported by the code for a plain unwired bus: a missing device NACKs and the call
returns in about 0.1 ms. The 16 ms and 50 ms waits happen only when the bus is not idle or the
master cannot drive its clock. There is one place that happens by construction on this build:
pin 24 is the radio's RST wire (`pins.h`, commit eb37d8d) and also Wire2's SCL.
`sequencer::begin()` starts Wire2 and then `radio::begin()` turns pin 24 into a GPIO output, so
Wire2's clock never leaves the chip, and one 50 ms transaction timeout per tick plus about
2 ms of normal work is 52 ms. That fits, but it is a reading of the code, not a measurement:
`./tvc link` now prints `sec_us_max`, which names the part of the tick that is slow, and the
I2C timeout counts per bus.

**What changed.**
- SELFTEST asks each optional part once. IMU, encoders, load cell, SD, flash and radio go into
  a presence mask (STATUS `present_mask`, byte 0 of the log header's reserved pair, console
  `present=`). A part that did not answer is never read or written again.
- Every I2C read in the tick uses `i2c_bus.cpp` instead of Wire: one shared 1 ms budget per
  tick, polled against a deadline; a bus that fails is left alone for 20 ms, its master reset,
  and the failure counted (`i2c_timeouts`, `i2c_errors`, `i2c_skipped`). A host test models the
  contract (`test_i2c_guard`): three dead buses never cost a tick more than 1 ms, one dead bus
  costs a little over 4% of the loop. The register sequence itself follows `WireIMXRT.cpp`'s but
  could only be compiled here, not run.
- Wire2 is not touched at all while the radio's RST wire is on pin 24, so the I2C peripheral
  can never pulse the radio's reset line. Encoder #2 reads as absent until the wire moves to 9.
- Still unbounded, and not touched here: an SD write with a card in, and a flash-mirror erase.
  `sec_us_max` (log push) will show whether either is a problem.

## What `loop_overrun_count` counts

**Now: one increment for every tick whose body took more than 3300 us to execute**
(`countsAsOverrun()` in `firmware/src/flight/loop_stats.h`, applied when `sequencer::tick()`
finishes; the fault bit `kLoopOverrun`, bit 7, is set at the same time and keeps its meaning,
"a tick overran"). A healthy loop reads 0. The count in STATUS is the same number truncated to
16 bits.

**Before: one increment for every tick that started, whatever it cost.** The old rule was
`loop_us > 3300` at the top of the tick, where `loop_us` is `main.cpp`'s `g_tick_timer` when the
tick is started: microseconds since that tick's *scheduled* start, measured from the previous
scheduled tick. `main.cpp` starts a tick only when that value has reached one period, 3333 us at
300 Hz, so it was never below 3333 and the test was true every time. The fault bit was set on
the first tick after boot and never cleared, which is why `./tvc ready` masks bit 7 out when it
looks for hard faults (it still does; bit 7 now means something, and a single slow tick, an SD
flush for instance, latches it until the next SELFTEST). SPEC.md 4.2 says "any tick exceeds 3.3
ms", which reads as the tick taking that long; the old code measured something that cannot be
under 3.33 ms, so the rule could not tell a healthy tick from a slow one.

With the old rule the counter said how many ticks ran, not how many were late or slow. Over a
test window it equalled the tick rate times the window. Two bench runs show it:

| run (old rule) | overrun count rose by | per second | tick rate it implies |
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

**What costs about 52 ms a tick** is in "Per-tick blocking audit" above: not established, with
one strong candidate (pin 24).

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

**The fix, applied.** Count a tick as an overrun when its execution time (`exec_us`, end minus
start of the tick body) is above 3300 us, judged when the tick finishes. A healthy loop reads 0,
a loop with one slow tick in a thousand reads 1 per thousand, and an overloaded loop reads the
number of slow ticks. `behind_us_max` stays as the backlog indicator and `exec_hist` shows how
slow. `test_loop_stats` pins the rule: 3300 us is not an overrun, 3301 us is; a healthy 1 ms
loop reads 0; a loop with every 100th tick at 5 ms reads one in a hundred; a 52 ms loop reads
every tick.
