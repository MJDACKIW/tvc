# SPEC.md — TVC Rocket: Flight Computer, Ground Station, and Matching Simulation

Authoritative specification for Claude Code. Every design decision below is deliberate; do not
substitute alternatives without flagging them. The controlling reference is the manuscript
*Design and Validation of a Thrust Vector Controlled Model Rocket Using PID-Kalman Architecture*
(Dackiw, under review, JHSS). Where this spec says "paper", it means that document. The firmware
and the simulation must implement exactly the equations in the paper, with the same parameter
values, so that the companion study (static-fire IAE vs. simulation) compares like with like.

---

## 0. How to work on this repo

- Work in **phases** (Section 9). Complete one phase, run its acceptance checks, commit, stop.
  Do not start the next phase in the same session unless told to.
- Before writing code in any phase, produce a short plan (files to create/modify, interfaces).
  Wait for approval only if the plan deviates from this spec.
- Never invent calibration constants. Anything marked `MEASURE` in `params.yaml` stays as a
  placeholder with a boot-time assertion that refuses to leave `IDLE` until it has been set.
- Firmware is **PlatformIO + C++17**, never `.ino`. Ground station and simulation are **Python 3.11+**.
- Development machine is a MacBook Pro (zsh). Teensy 4.1 is the flight computer; Adafruit
  Feather M0 RFM95 (#3178) is the ground radio bridge; Adafruit RFM95W breakout (#3072) is the
  vehicle radio.
- Commit messages: imperative, one line, no emoji.

---

## 1. Repository layout

```
tvc/
├── params.yaml                  # SINGLE SOURCE OF TRUTH for every number shared by sim and firmware
├── tools/
│   ├── gen_params.py            # params.yaml -> core/params.h AND sim/tvc_params.py
│   ├── decode_log.py            # binary SD log -> CSV/npz + plots (extend the existing one)
│   ├── iae_compare.py           # measured static-fire attitude vs sim prediction, IAE metric
│   └── parity_test.py           # proves sim and firmware core produce identical outputs
├── core/                        # PURE C++, no Arduino/Teensy headers. Compiled into BOTH firmware and sim.
│   ├── params.h                 # GENERATED — do not edit by hand
│   ├── kalman1d.h/.cpp          # scalar Kalman filter, one instance per axis (paper §3.2, eq6–eq11)
│   ├── pid.h/.cpp                # discrete PID with saturation + anti-windup (paper §3.1, A.3)
│   ├── rate_limiter.h/.cpp      # servo slew model, 0.12 s / 60° (paper §3.3)
│   ├── attitude_from_accel.h    # accelerometer tilt z_k per axis
│   ├── controller.h/.cpp        # ties KF + PID + limiter per axis; the one function both sides call
│   └── ffi.h/.cpp               # extern "C" wrapper so Python can load core as a shared library
├── firmware/
│   ├── platformio.ini           # envs: teensy_flight, teensy_test_servo, teensy_test_radio, feather_bridge, native
│   ├── src/
│   │   ├── flight/              # the ONE vehicle firmware (all modes)
│   │   ├── feather_bridge/      # ground radio <-> USB serial bridge (dumb, stateless)
│   │   └── tools/                # servo_ident.cpp, radio_test.cpp (existing, keep)
│   ├── lib/                     # vehicle-only libs if needed
│   └── test/                    # pio unit tests for core/, run with `pio test -e native`
├── ground/
│   ├── gs.py                    # PyQt6 + pyqtgraph ground station GUI
│   ├── model.py                 # GUI state + command-enabling rules, Qt-free (tested headless)
│   ├── link.py                  # serial / sim-socket / replay transports (codec: protocol/telem_codec.py)
│   ├── sd_replay.py             # SD log -> the downlink frames the vehicle would have sent (--replay)
│   ├── recorder.py               # redundant ground-side logging
│   ├── tests/                   # codec-vs-protocol.h check; offscreen --replay/--sim acceptance
│   └── requirements.txt
├── sim/
│   ├── tvc_params.py            # GENERATED
│   ├── vehicle.py                # rigid-body dynamics, thrust curve, aero disturbance (paper §6.1, §3.4.2)
│   ├── sensors.py                # MPU-6050 noise model, gravity-tilt measurement (paper §6.7, §6.11)
│   ├── run_sim.py                # closed-loop RK4 @ 1 ms, control @ 150 Hz; uses core via ctypes
│   ├── serve.py                  # run_sim.py --serve: virtual vehicle on a TCP socket for gs.py --sim
│   ├── figures/                  # regenerates paper Figs 2–11 (existing plots may be ported here)
│   └── data/estes_e12.eng       # NAR thrust curve points
├── protocol/
│   ├── protocol.h                # packet structs (C)
│   └── telem_codec.py            # the one Python codec for them (checked against protocol.h by a test)
├── CLAUDE.md                    # short: pointers to this file + build/flash commands
└── README.md
```

Rationale for `core/`: the paper's claim that the flight controller and the simulated controller
are the same architecture becomes literally true. The Kalman filter, PID, and slew limiter are
compiled once for Teensy and once as `libtvccore.dylib` for the Mac; the Python simulation calls
the same object code. `tools/parity_test.py` proves it.

---

## 2. `params.yaml` and code generation

All shared numbers live here. `tools/gen_params.py` emits `core/params.h` (constexpr) and
`sim/tvc_params.py`. Both generated files carry a SHA-256 of `params.yaml`; firmware logs that
hash in every log file header and telemetry `STATUS` packet; the sim writes it into every output
file. Mismatched hashes make `iae_compare.py` refuse to run.

```yaml
schema: 1
control:
  rate_hz: 150            # paper §4, §6.1 — KF + PID update rate; dt = 1/150 s exactly
  loop_hz: 300            # sensor sample, state machine, logging; integer multiple of rate_hz
  kp: MEASURE             # paper: Ziegler–Nichols start, refined on stand
  ki: 0.0                 # paper §3.1: sim uses Ki = 0; firmware keeps the term
  kd: MEASURE
  max_deflection_deg: 10.0   # paper §2.2: ±10° command limit (mechanical hard stop is 12°, verified in CAD)
  integral_clamp_deg_s: 5.0  # anti-windup: |integral| clamp, and freeze integration while saturated
kalman:
  q: MEASURE              # process noise variance, deg^2 per step
  r: MEASURE              # accel tilt measurement variance, deg^2
  p0: 1.0
  accel_gate_g: [0.6, 1.4]   # |a| outside this band -> skip update step (gyro-only); paper §6.7 behaviour
servo:
  slew_deg_per_s: 500.0   # 60°/0.12 s, paper §3.3
  us_per_deg_x: MEASURE   # linkage geometry — from servo_ident tool
  us_per_deg_y: MEASURE
  neutral_us_x: MEASURE
  neutral_us_y: MEASURE
  sign_x: 1
  sign_y: 1
vehicle:
  mass_kg: MEASURE
  inertia_pitch_kg_m2: MEASURE
  inertia_yaw_kg_m2: MEASURE
  r_gimbal_to_com_m: MEASURE        # paper r in eq12
  l_cop_minus_com_m: 0.080          # paper §2.6: CoP ≈ 80 mm ahead of CoM (positive = destabilising)
  diameter_m: 0.0762
  cn_alpha_per_rad: MEASURE         # Barrowman, from OpenRocket
motor:
  name: "Estes E12-4"
  thrust_curve_file: "sim/data/estes_e12.eng"
  burn_time_s: 2.44                 # paper §6.2
staging:
  launch_accel_g: 2.0               # sustained axial accel threshold
  launch_accel_window_s: 0.05       # must persist this long
  launch_min_arm_time_s: 5.0        # paper §4.4
  launch_gyro_quiet_deg_s: 10.0     # paper §4.4: gyro noise must be below this before launch is accepted
  burnout_accel_g: 0.3              # axial accel drops below this -> burnout candidate
  burnout_window_s: 0.10
  prelaunch_tilt_abort_deg: 10.0    # paper §4.7: excessive pre-launch tilt -> abort to SAFE
  servo_rate_fault_deg_s: 2000.0    # paper §4.7: commanded rate above this for >50 ms -> disable TVC, log fault
  apogee_timeout_s: 8.0             # free-flight only
pyro:
  pulse_ms: 800
  continuity_ok_min_adc: 200
  continuity_ok_max_adc: 3800
  heartbeat_timeout_ms: 1000
telemetry:
  rate_hz: 20
  lora_freq_mhz: 915.0
  lora_sf: 7
  lora_bw_khz: 125
  lora_cr: 5
  tx_power_dbm: 13
logging:
  sd_rate_hz: 300
  ring_seconds: 12
  flash_mirror_rate_hz: 50
```

Generator rules: `MEASURE` becomes `constexpr float X = NAN;` plus an entry in a
`kMeasureList[]` that the firmware checks at boot. Any NaN in the list that is required for the
selected mode blocks the `IDLE → ARMED` transition and reports which parameter is missing in
`STATUS.fault_bits`.

---

## 3. Shared core — equations (implement exactly)

All angles in degrees, time in seconds, one independent scalar filter and PID per axis
(pitch about $X_b$, yaw about $Y_b$). No matrices anywhere in `core/`.

### 3.1 Kalman filter (paper §3.2, eq6–eq11, $H = 1$)

Predict:
$$\hat x_{k|k-1} = \hat x_{k-1} + \omega_k\,\Delta t, \qquad P_{k|k-1} = P_{k-1} + Q$$

Update (only if accel gate passes; otherwise $\hat x_k = \hat x_{k|k-1}$, $P_k = P_{k|k-1}$):
$$K_k = \frac{P_{k|k-1}}{P_{k|k-1} + R}, \qquad
\hat x_k = \hat x_{k|k-1} + K_k\,(z_k - \hat x_{k|k-1}), \qquad
P_k = (1 - K_k)\,P_{k|k-1}$$

`z_k` for pitch is $\operatorname{atan2}(a_y, a_z)$ and for yaw $\operatorname{atan2}(-a_x, a_z)$
in the **body frame** (after $R_{B\leftarrow S}$ remap) — confirm sign convention against the
servo axis map in Section 4.5 and write it once in `attitude_from_accel.h`, never inline.
The gate is the paper's "accelerometer unreliable ⇒ rely on gyro" behaviour made explicit; the
sim must use the identical gate.

### 3.2 PID (paper §3.1 eq5, Appendix A.3)

```
e      = 0 - x_hat                     // setpoint is vertical
if (!saturated_last) integral += e*dt   // conditional integration
integral = clamp(integral, ±integral_clamp)
deriv  = (e - e_prev)/dt
u      = Kp*e + Ki*integral + Kd*deriv
u_cmd  = clamp(u, ±max_deflection)
saturated_last = (u != u_cmd)
e_prev = e
```
`Ki` defaults to 0 from `params.yaml` (paper §3.1). The derivative acts on the Kalman estimate,
as the paper specifies; do not add a separate derivative filter without flagging it.

### 3.3 Servo slew model (paper §3.3, §6.6)
$$\delta_{k} = \delta_{k-1} + \operatorname{clamp}\!\left(u_{cmd,k} - \delta_{k-1},\; \pm\,\dot\delta_{max}\Delta t\right)$$
In firmware this is applied to the *command* sent to the servo (so the logged command is what
the sim models). In the sim it is the actuator model. Same function, same numbers.

### 3.4 `controller.h` API
```cpp
struct AxisState { float x_hat, P, integral, e_prev, delta; bool saturated; };
struct AxisOut   { float x_hat, u_raw, u_cmd, delta, K; bool accel_used; };
AxisOut controller_step(AxisState&, float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok, const ControlParams&);
```
`ffi.cpp` exposes `tvc_controller_step(...)` with plain floats for ctypes.

### 3.5 Corrective torque and disturbance (used by sim and by `iae_compare.py`)
$$\tau_{ctrl} = F_T(t)\, r\, \sin\delta, \qquad
\tau_{dist}(\theta) = \tfrac12 \rho v^2 A_{ref} C_{N\alpha}\, \ell\, \theta, \qquad
I\ddot\theta = \tau_{ctrl} - \tau_{dist}$$
(paper eq12, eq14, eq18). On the static stand $v = 0$, so $\tau_{dist}=0$ and only gimbal
friction/inertia remain — the sim must take a `--stand` flag that zeroes aero and uses the
stand's measured inertia (vehicle + gimbal plate) instead of the free-flight inertia.

---

## 4. Vehicle firmware (`firmware/src/flight/`)

### 4.1 Modes (selected at boot by a 3-position DIP/jumper on pins 26/27, overridable by uplink while in IDLE)

| Mode | Estimator | Controller | Pyro | Purpose |
|---|---|---|---|---|
| `SIM_ONBOARD` | on, fed by onboard 6-DOF model (port of `sim/vehicle.py`, Euler @ 1 kHz is acceptable here) | closed loop | locked out | Bench test of the full stack; servos move; ground GUI shows everything live |
| `HIL` | on, fed by IMU packets streamed over USB from `sim/run_sim.py --hil` | closed loop | locked out | Laptop model, real servos and encoders in the loop |
| `STAND_OPEN` | on (logging only) | open-loop profiles HOLD / STEP_X / STEP_Y / RAMP_X / CHIRP_X (keep existing) | armed | Plant identification: thrust curve, servo response, $\partial M/\partial\delta$ |
| `STAND_CLOSED` | on | closed loop; stand pivot free | armed | Companion study: measured attitude vs sim, IAE metric |
| `FLIGHT` | on | closed loop; launch/burnout detection active | armed | Requires G-class reload; not exercised yet |

`DRY_RUN` is not a mode: it is any armed mode with a 1 kΩ + LED in place of the igniter, and
the GUI has a checkbox that disables the `burnout` detector so the sequence completes on time
(the existing `g_dry` fix).

### 4.2 Scheduler
`elapsedMicros`-driven fixed-rate loop at `loop_hz` = 300 Hz. Every tick: read IMU, read HX711
if ready (it is ~80 SPS; use the last value), read both AS5600 encoders, evaluate state machine,
write SD record. Every 2nd tick (150 Hz exactly): `controller_step` on both axes and servo
write. Telemetry at 20 Hz from a separate `elapsedMicros`. Measure and log loop overrun count;
a `STATUS.fault_bits` bit is set if any tick exceeds 3.3 ms.

### 4.3 Test-stand / flight sequencer
```
BOOT → SELFTEST → IDLE → ARMED → COUNTDOWN → IGNITE → BURN → TAILOFF → SAFE
                          ↓ any fault or ABORT or heartbeat loss ↓
                                         SAFE / FAULT
```
- `SELFTEST`: IMU WHO_AM_I, gyro bias sample (2 s, vehicle still), $R_{B\leftarrow S}$
  determinant check (must be ±1), SD card write test, LittleFS mount, radio init, HX711 tare
  (STAND modes only), encoder read, servo sweep to ±5° and back with encoder confirmation
  (encoder must move in the commanded direction or fault). Missing `MEASURE` params → fault.
- `IDLE`: accepts uplink `SET_MODE`, `SET_PROFILE`, `TARE`, `ARM`. Requires physical arm switch
  (`ARM_SENSE_PIN`) AND uplink `ARM` to move to `ARMED`.
- `ARMED`: continuity reported every telemetry packet. Pre-launch tilt > threshold → `SAFE`
  (paper §4.7). Heartbeat loss > 1000 ms → `SAFE`. Uplink `COUNTDOWN` starts a 10 s count and
  returns a random 16-bit `fire_token`.
- `COUNTDOWN`: every second is a `EVENT` packet. At T-0 the vehicle waits up to 2 s for uplink
  `FIRE_CONFIRM(fire_token)`; without it → `SAFE`. With it → `IGNITE`.
- `IGNITE`: gate high for `pyro.pulse_ms`, then forced low regardless of anything else
  (hardware-timer-driven, not loop-driven). Control enabled at gate-high in closed-loop modes.
- `BURN`: exit on burnout detection (axial accel *and* load cell below threshold for
  `burnout_window_s`) or on a 6 s timeout.
- `TAILOFF` (1.5 s): keep logging, controller off, servos to neutral.
- `SAFE`: flush ring buffer, close files, write summary to EEPROM, beep.

`FLIGHT` mode adds the paper's free-flight states after `BURN`: `COAST` → `APOGEE` → `DESCENT`
→ `LANDED`, with the multi-condition apogee logic of Appendix A.2, the `apogee_timeout_s`
forced transition, and launch detection per §4.4 (sustained accel + increasing integrated
velocity + min arm time + gyro quiet). Barometer is absent; implement the altitude conditions
behind a compile-time `HAS_BARO` that is `false`.

### 4.4 Safety (non-negotiable)
- Pyro gate is a MOSFET driven from a pin that is held LOW by an external pulldown; firmware
  configures it as output LOW in the first line of `setup()`.
- Firing requires all of: mode allows pyro, physical arm switch closed, state == `COUNTDOWN`,
  token matches, heartbeat fresh, continuity in range.
- Servo command rate fault (paper §4.7): if $|u_{cmd,k}-u_{cmd,k-1}|/\Delta t$ exceeds
  `servo_rate_fault_deg_s` for more than 50 ms, disable TVC, centre servos, set fault bit,
  keep logging. Sequence continues (the motor is already burning).
- Watchdog (Teensy 4.1 WDT) at 100 ms; the ISR sets the pyro pin LOW before reset.
- On any reset, `SELFTEST` reads the EEPROM summary and reports if the previous run ended
  abnormally (paper §4.7, non-volatile backup).

### 4.5 Pin map (Teensy 4.1) — resolve the existing collision between HX711 and servo/buzzer pins
```
SERVO_X_PIN     3    // channel B -> moment about X_b (from servo_ident session)
SERVO_Y_PIN     5    // channel A -> moment about Y_b
BUZZER_PIN      4
HX_DOUT_PIN     6
HX_SCK_PIN      7
RFM95_IRQ       8
RFM95_RST       9    // MOVED from 24 to free Wire2 for the second encoder — one wire to move
RFM95_CS        10
SPI0 MOSI/MISO/SCK   11/12/13 (fixed)
PYRO_GATE_PIN   14
ARM_SENSE_PIN   15   // input pullup, switch to GND
PYRO_CONT_PIN   A2 (16)   // analog continuity sense through 1 kΩ
Wire  (MPU-6050 0x68)   SDA 18 / SCL 19
Wire1 (AS5600 #1 0x36)  SDA 17 / SCL 16  -> CONFLICT with A2; use PYRO_CONT_PIN = A3 (17)? No: use A14 (38) for continuity
Wire2 (AS5600 #2 0x36)  SDA 25 / SCL 24
STATUS LEDs     20, 21, 22 (power/IMU ok, armed, TVC active)
MODE JUMPERS    26, 27 (input pullup)
BATT_SENSE_PIN  A16    // resistor-divider battery voltage sense, added with the telemetry
                        // protocol (Section 4.7): TELEM's batt_mv field needs a real ADC input,
                        // and none was assigned before. Scale factor is params.yaml
                        // power.batt_adc_to_mv (MEASURE — no divider ratio invented here).
SD              built-in SDIO
```
Final: `PYRO_CONT_PIN = A14 (pin 38)`. AS5600 has a fixed address, hence one per I²C bus. Flag
in the README that the RFM95 RST wire moves from 24 to 9 and that HX711 moves to 6/7.

**Load cell wiring.** Four half-bridge load cells, one at each stand foot, wired in a loop for
excitation (white-white, black-black), with the two diagonal pairs' red (signal) leads going
to the HX711's two differential inputs (`E+/E-` and `A+/A-`). This sums all four cells into
one logical channel on `HX_DOUT_PIN`/`HX_SCK_PIN` (6/7): the firmware never sees four
channels, only one combined thrust reading. Mechanically: the fixed end of each cell is bolted
to the stand's base plate; the free end takes load through a point contact (a printed
bracket's pip) rather than a rigid bolt, so the cell sees pure axial load without a bending
moment from misalignment; loose guide pins at each foot prevent the stand from tipping without
constraining the cells themselves.

### 4.6 Redundant logging (three independent copies)
1. **SD primary**: pre-allocated 64 MB binary file, `Record` struct (below) at 300 Hz,
   written from a 12 s RAM ring buffer with `SdFat` and 512-byte aligned blocks. The file is
   opened and pre-allocated in `SELFTEST`, not at ignition. Header contains `params.yaml` hash,
   mode, profile, firmware git hash, boot count.
2. **LittleFS_Program mirror**: 50 Hz decimated records into the Teensy's program flash
   (1 MB region). Survives SD card ejection/corruption. Dumped over USB post-test with a
   serial command.
3. **Ground recorder**: every telemetry packet (20 Hz) with ground timestamp and RSSI,
   written as both raw `.bin` and `.csv`, plus the GUI event log. Independent of the vehicle.
Plus the EEPROM summary (state at exit, fault bits, peak thrust, peak |θ|, IAE) after every run.

`Record` (packed, 74 bytes; an earlier draft of this line said 64, but the field list below was
always the authoritative definition and was never trimmed to fit that count, so 74 is correct
and 64 was simply never updated): `uint32 t_us; uint8 state; uint8 mode; uint16 fault_bits;
float thrust_N; int16 gyro[3]; int16 accel[3]; float xhat_p, xhat_y; float u_p, u_y;
float delta_p, delta_y; float enc_p, enc_y; float K_p, K_y; uint16 servo_us_p, servo_us_y;
uint8 pyro; uint8 cont_adc8; uint16 loop_us; uint16 crc16;`

### 4.7 Telemetry protocol (`protocol/protocol.h`, shared) — FINALIZED

Framing on both radio and USB: `0xAA 0x55 | len | type | payload | crc16`, little-endian
throughout. `len` counts payload bytes only (max 48, one byte); `crc16` is CRC-16/CCITT-FALSE
(same polynomial and init value as `firmware/src/flight/record.h`'s `crc16()`, but computed
over `len ++ type ++ payload`, not the payload alone) and is 2 bytes. Max payload 48 bytes fits
one LoRa frame at SF7/BW125 with margin; ~5.5 kbps usable ⇒ 20 Hz × ~60 B ≈ 9.6 kbps is **too
much**, which is why downlink is 10 Hz `TELEM` + 1 Hz `STATUS` rather than one packet at 20 Hz.
Full rate stays on SD (Section 4.6).

**TELEM (10 Hz, 27-byte payload).** Every field int16/uint16 unless noted:
```
t_ms       int16   ms since ignition; negative pre-launch (t_ms = 0 at IGNITE's gate-high)
state      uint8   tvc::flight::State (record.h)
thrust     int16   x10   (N)
xhat_p     int16   x100  (deg)
xhat_y     int16   x100  (deg)
u_p        int16   x100  (deg)
u_y        int16   x100  (deg)
enc_p      int16   x100  (deg)
enc_y      int16   x100  (deg)
gyro_z     int16   x10   (deg/s)
K_p        uint16  x1000 (Kalman gain, 0..1)
K_y        uint16  x1000
flags      uint8   bit0 pitch-gate-ok, bit1 yaw-gate-ok, bit2 pitch-saturated,
                    bit3 yaw-saturated, bit4 tvc-active, bit5 dry-run,
                    bit6 arm-switch-closed, bit7 reserved
cont_adc8  uint8   continuity ADC, 12-bit -> 8-bit (same shift as Record.cont_adc8)
batt_mv    uint16  battery voltage, millivolts (BATT_SENSE_PIN x power.batt_adc_to_mv)
```
Total: 2+1+2+2+2+2+2+2+2+2+2+2+1+1+2 = 27 bytes. `tvc-active` mirrors the existing
`STATUS LED` condition (`state == BURN`); `pitch/yaw-gate-ok` is each axis's own
`ShadowOut.accel_used`, not one shared flag, so a future per-axis accelerometer gate needs no
protocol change; `*-saturated` is `u_raw != u_cmd` for that axis (identical condition to the
PID's own `saturated_last`, paper §3.1/Appendix A.3). `arm-switch-closed` (added in the
ground-GUI phase, using a previously reserved bit) is `ARM_SENSE_PIN` read as closed, so the GUI
can gate `ARM` on it (Section 5).

**STATUS (1 Hz, 42-byte payload).** Existing fields (mode, fault_bits, params hash, SD/flash
bytes, uplink RSSI, boot count, missing-MEASURE list) plus the cumulative fields below, all
computed and integrated **on the vehicle**, not the ground station, so they are correct even
across radio packet loss:
```
mode                 uint8
fault_bits            uint16
params_hash6          6 raw bytes  (first 6 bytes of SHA-256(params.yaml); the same truncation
                                    gen_params.py already prints as hash[:12] hex chars)
sd_bytes_written      uint32
flash_bytes_written   uint32
rssi_last_uplink_dbm  int8         (RSSI of the most recently received uplink frame)
boot_count            uint16
missing_measure_bits  uint32       (bit i set == params::measure_bits::* index i is still NaN;
                                    core/measure_bits.json names each bit — see Section 2)
loop_overrun_count    uint16
sat_pct_p             uint8        (% of control ticks this run with pitch saturated, 0-100)
sat_pct_y             uint8
running_iae_p         uint16 x100  (integral of |x_hat_p| dt since boot, deg*s)
running_iae_y         uint16 x100
peak_thrust           uint16 x10   (N)
peak_tilt_p           uint16 x100  (max |x_hat_p| this run, deg)
peak_tilt_y           uint16 x100
fw_git_hash           4 raw bytes  (first 4 bytes of kFirmwareGitHash's hex prefix; the
                                    "-dirty" suffix, if any, is not encoded here — it is still
                                    visible in the SD log header and the USB console)
```
Total: 1+2+6+4+4+1+2+4+2+1+1+2+2+2+2+2+4 = 42 bytes.

**EVENT (5-byte payload, no free-text):** `t_ms int16, event_code uint8, aux int16`. Sent on
state changes, countdown ticks, and faults; retransmitted 3× (best-effort redundancy against
loss, since EVENT is not integrated like STATUS's cumulative fields). `event_code` values come
from one generated table (`protocol/event_codes.yaml` -> `protocol/event_codes.h` for firmware,
`protocol/event_codes.json` for any Python decoder, via `tools/gen_protocol.py`), so the GUI
renders messages by looking up the code, never by parsing a transmitted string.

Downlink packet types: `TELEM`, `STATUS`, `EVENT`, `ACK`.
Uplink packet types: `HEARTBEAT` (2 Hz from GUI), `SET_MODE`, `SET_PROFILE`, `TARE`, `ARM`,
`DISARM`, `COUNTDOWN`, `FIRE_CONFIRM(token)`, `ABORT`, `DRY_RUN(bool)`, `PING`, `DUMP_FLASH`
(USB only). Every uplink is ACKed with the command's sequence number; GUI shows unacked
commands in amber. This phase's vehicle firmware recognizes `HEARTBEAT` (freshness + RSSI
tracking for `STATUS.rssi_last_uplink_dbm`) over the radio link; dispatching the rest of the
uplink commands from radio (they already work from the USB console) is left to the ground-GUI
phase (Section 9 Phase 5), flagged here rather than silently stubbed.

**Uplink payloads and ACK** (defined in the ground-GUI phase; `protocol.h` has the structs).
Every uplink payload starts with `seq uint16`, incremented by the ground station per command:
```
HEARTBEAT, TARE, ARM, DISARM, COUNTDOWN, ABORT, PING, DUMP_FLASH   seq                 (2 B)
SET_MODE (Mode), SET_PROFILE (Profile), DRY_RUN (0/1)              seq, value uint8    (3 B)
FIRE_CONFIRM                                                       seq, token uint16   (4 B)
ACK (downlink)          seq uint16, cmd_type uint8, result uint8, aux uint16           (6 B)
```
`result`: 0 OK, 1 wrong state, 2 arm switch open, 3 MEASURE missing, 4 bad token, 5 bad
argument, 6 no continuity, 7 unsupported, 8 heartbeat stale (ARM only: refused rather than
armed and then immediately dropped to SAFE by the heartbeat check). `aux` carries the
`fire_token` in COUNTDOWN's ACK and is 0 otherwise. `DISARM` returns ARMED → IDLE, and SAFE →
IDLE (the USB console's `RESET`). HEARTBEATs are ACKed like everything else, which is what
the GUI's ACK-latency readout measures. As of this phase, `sim/run_sim.py --serve` implements
all of this; the vehicle firmware still dispatches only HEARTBEAT from radio.

### 4.8 Feather M0 bridge
Stateless: LoRa ↔ USB CDC, same framing both directions, appends RSSI/SNR to each downlink
frame. Uses RadioHead `RH_RF95` with CS 8 / INT 3 / RST 4. No logic lives here.

---

## 5. Ground station GUI (`ground/gs.py`)

PyQt6 + pyqtgraph, dark theme, one window, resizable, 60 Hz redraw decoupled from serial
thread via a queue. Layout (left to right, top to bottom):

**Header bar**: connection state (port, RSSI/SNR sparkline, packet loss %, uplink ACK latency),
vehicle MODE, STATE as a large colored badge, mission clock T±, params-hash match indicator
(green if GUI's `tvc_params.py` hash == vehicle's), firmware git hash.

**Left column — Command panel**:
- Mode selector (only enabled in IDLE), profile selector (STAND_OPEN only), DRY RUN checkbox
- `TARE`, `PING`
- `ARM` (requires vehicle to report arm switch closed; button shows why it is disabled)
- `COUNTDOWN` — large; on click opens a 10 s countdown display driven by vehicle `EVENT`s
- `CONFIRM FIRE` — appears only during the last 3 s of countdown, red, requires press-and-hold
  500 ms, sends `FIRE_CONFIRM(token)`
- `ABORT` — always enabled, always visible, spacebar shortcut
- Continuity indicator (green/red with ADC value), battery voltage, arm-switch state

**Center — Live plots** (pyqtgraph, 30 s rolling, shared x):
1. Thrust [N] with E12-4 nominal curve overlaid from T-0
2. Attitude: $\hat\theta_{pitch}$, $\hat\theta_{yaw}$ and encoder-measured stand angles
3. Gimbal: commanded $u$, slew-limited $\delta$, ±10° limits
4. Gyro rates
5. Kalman gain $K$ per axis and accel-gate flag (shows the filter trusting/rejecting the accel)

**Right column — Telemetry table** (every field in `TELEM`+`STATUS`, updated live), fault bits
decoded to text, SD/flash bytes written, loop overrun count, live IAE accumulator
($\int|\hat\theta|\,dt$ per axis since T-0), peak thrust, peak |θ|, saturation fraction.

**Bottom — Event log** (timestamped, colour by severity) and a status line.

Behaviour: on `SAFE` the GUI finalises the ground log, names the run
`YYYYMMDD_HHMMSS_<mode>_<profile>`, and offers one-click "Run analysis" that calls
`tools/decode_log.py` on a chosen SD file and `tools/iae_compare.py`. Connection loss shows a
full-width red banner. The GUI also has a `--replay file.bin` flag that plays back a ground log
at real time for demoing without hardware, and a `--sim` flag that connects to
`sim/run_sim.py --serve` over a local socket so the whole GUI can be exercised with no radio.

---

## 6. Simulation (`sim/`) — must match the paper and the firmware

- RK4 at 1 ms on $I\ddot\theta = \tau_{ctrl}-\tau_{dist}$ per axis (paper §6.1). Thrust from
  NAR E12-4 data via `numpy.interp` (paper §6.2). Control tick every 6.667 ms: implement as
  "run controller when `t >= next_ctrl_t`, `next_ctrl_t += 1/150`" so the sim and firmware see
  the same $\Delta t$.
- The controller is `core/` via ctypes (`libtvccore.dylib` built by `make -C core native` with
  clang). A pure-Python fallback is **not** allowed in the main path; it may exist only inside
  `parity_test.py` as a reference.
- `sensors.py`: gyro white noise + bias, accel white noise + vibration term proportional to
  thrust, per MPU-6050 datasheet values (paper §6.7, §6.11). Seeded.
- Scenarios reproducing paper figures: `baseline` (5° tip-off), `open_loop`, `disturbance`
  (0.12 N·m for 50 ms at 0.6 s), `sensitivity`, `controllability_map`, `monte_carlo` (100
  trials, θ₀ ∈ [0.5°, 6°], thrust ±8%). Each writes `.npz` + PNG with the params hash.
- `--stand`: aero off, stand inertia, optional Coulomb friction torque at the pivot (parameter,
  default 0, `MEASURE`).
- `--hil`: streams synthetic IMU samples over USB serial at 300 Hz to the Teensy in `HIL` mode
  and reads back servo commands; the dynamics then use the *vehicle's* command, closing the
  loop through real firmware timing.
- `--serve`: exposes the same telemetry framing on a local TCP socket for the GUI's `--sim`.

### 6.1 Parity test (`tools/parity_test.py`)
Generates a fixed 3 s trace of (gyro, accel_tilt, gate) at 150 Hz, runs it through
(a) `libtvccore.dylib` via ctypes and (b) `pio test -e native` (which prints the same trace's
outputs as CSV). Asserts bitwise-equal `u_cmd` and `x_hat` sequences. This is the artefact that
lets the paper say the flight and simulation controllers are identical. Run it in CI
(`make check`).

---

## 7. Analysis tools

- `decode_log.py` (extend existing): parse `Record`, verify CRCs, report dropped ticks, export
  CSV and `.npz`, plots of thrust / attitude / gimbal / gyro / K / faults.
- `iae_compare.py`: takes a stand log + the matching sim scenario (same mode, same θ₀ from the
  encoder at T-0, same params hash), aligns at ignition (thrust rise), plots measured vs
  simulated θ(t), $\delta(t)$, and reports IAE $=\int_0^{t_b}|\theta|\,dt$ for both, plus RMSE
  and the time-shift that minimises RMSE (a direct latency estimate). Output a LaTeX table row.
- Thrust-curve fit: from `STAND_OPEN` HOLD runs, compare measured $F_T(t)$ to NAR curve;
  estimate $\partial M/\partial\delta$ from STEP runs using encoder angular acceleration and the
  stand inertia.

---

## 8. Testing without firing the motor — required test matrix

| Level | What | Pass criterion |
|---|---|---|
| Unit | `pio test -e native` on `core/` | KF converges on synthetic data; PID clamps and anti-windup behave; slew limiter never exceeds 500°/s |
| Parity | `tools/parity_test.py` | bitwise equal |
| Sim | `run_sim.py baseline` | settles < 0.3 s, saturation ≈ 1% of burn, matches paper Table 1 within 10% |
| Bench SIM_ONBOARD | Teensy runs onboard model, GUI live over radio | GUI plots show recovery from injected 5° offset; no loop overruns; three logs present and consistent |
| HIL | `run_sim.py --hil` | measured servo response matches slew model; loop timing verified with encoder |
| Radio | 30 min soak at 10 Hz at the intended stand distance | packet loss < 2%, heartbeat never times out |
| Dry run on stand | `STAND_OPEN`, LED igniter, `DRY_RUN` on | full sequence, pulse width on scope = `pulse_ms`, SAFE reached, GUI finalises run |
| Abort paths | pull heartbeat, open arm switch, send ABORT, tilt vehicle | each drops to SAFE within 1 s; pyro pin never rises |

---

## 9. Phased build plan (one phase per session; stop after acceptance)

1. **Scaffold + params + core + native tests + parity.** Deliver: layout, `gen_params.py`,
   `core/`, `pio test -e native` green, `parity_test.py` green. No hardware.
2. **Simulation.** `run_sim.py` scenarios reproduce paper Figures 3–8 from `core/`. Acceptance:
   numbers within 10% of paper Table 1.
3. **Vehicle firmware, no radio.** Modes `SIM_ONBOARD` and `STAND_OPEN`, sequencer, three-way
   logging, USB serial console for bench. Port the existing `static_fire.cpp` state machine and
   fixes (Section 10). Acceptance: bench `SIM_ONBOARD` run produces consistent SD + flash logs.
4. **Radio + protocol + Feather bridge + minimal CLI ground tool.** Acceptance: 30 min soak.
5. **Ground GUI.** Acceptance: `--replay` and `--sim` work with no hardware; then live bench.
6. **STAND_CLOSED + HIL + analysis tools.** Acceptance: `iae_compare.py` on an HIL run.
7. **FLIGHT mode** (launch/burnout/apogee logic), gated behind the G-reload procurement.

---

## 10. Existing decisions and fixes to preserve

- Dry-run burnout bug: burnout detector must be disabled by `DRY_RUN` flag with a time-based
  exit (already fixed once; do not regress).
- `-Wl,-u,_printf_float` in `build_flags` for Teensy float printing.
- One `.cpp` entry point per env with explicit `build_src_filter`; `lib_ignore = RadioHead` on
  Teensy envs if Teensyduino's bundled copy collides; PaulStoffregen RadioHead fork via
  `mikem/` namespace.
- `pio` lives at `~/.platformio/penv/bin`; an unrelated `.venv` once broke the CLI — use the
  full path in `CLAUDE.md`.
- Feather M0 needs the double-tap bootloader for upload; document the exact sequence.
- Servo axis map: channel A → moment about $Y_b$, channel B → moment about $X_b$.
- $R_{B\leftarrow S}$: signed permutation from the gravity-based procedure in `HANDOFF.md`;
  boot asserts $\det = \pm1$ and refuses `ARMED` otherwise.
- Never rely on USB serial during a firing (parasitic force path into the load cell); USB is
  for bench, HIL, and post-test dumps only.
- Gimbal mechanical limit is 12° (CAD-verified); paper's control limit is 10°. `max_deflection`
  = 10, and the servo driver additionally hard-clips at 11.5° as a last resort.
- The legacy `config.h` files under `legacy/` define an older pin map (different SERVO, HX711,
  BUZZER, and PYRO_CONT assignments) that conflicts with Section 4.5; Section 4.5 wins.

### Legacy platformio.ini environments

Verbatim env definitions from the pre-Phase-1 PlatformIO projects now archived read-only under
`legacy/`, kept here so Phase 3 can reuse `build_src_filter`, `lib_deps`, `lib_ignore`, and
`build_flags` without re-deriving them. Historical reference only: some referenced source
filenames (`radio_test_rx.cpp`, `tvc_servo_ident.cpp`) and the pin maps behind these projects
do not all match the current tree. Sections 1 and 4.5 win where they disagree.

`legacy/FlightComputer/platformio.ini` (bare PlatformIO template, no custom build settings):

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
```

`legacy/Radio test/platformio.ini`:

```ini
[env:teensy_static_fire]
platform = teensy
board = teensy41
framework = arduino
lib_deps = https://github.com/PaulStoffregen/RadioHead.git
lib_ignore = RadioHead@1.120.0
build_src_filter = +<static_fire.cpp> -<radio_test_tx.cpp> -<radio_test_rx.cpp>

[env:teensy_radio_tx]
platform = teensy
board = teensy41
framework = arduino
lib_deps = https://github.com/PaulStoffregen/RadioHead.git
lib_ignore = RadioHead@1.120.0
build_src_filter = +<radio_test_tx.cpp> -<static_fire.cpp> -<radio_test_rx.cpp>


[env:feather_radio_rx]
platform = atmelsam
board = adafruit_feather_m0
framework = arduino
lib_deps = https://github.com/PaulStoffregen/RadioHead.git
build_src_filter = +<feather_radio_rx.cpp> -<static_fire.cpp> -<radio_test_tx.cpp>
```

`legacy/TVC platform IO/platformio.ini`:

```ini
[teensy_base]
platform = teensy
board = teensy41
framework = arduino
build_flags = -O2
monitor_speed = 115200
monitor_echo = yes
monitor_filters = send_on_enter
monitor_eol = LF

[env:ident]
extends = teensy_base
build_src_filter = +<tvc_servo_ident.cpp>

[env:staticfire]
extends = teensy_base
build_src_filter = +<static_fire.cpp>
lib_deps = greiman/SdFat@^2.2.2
```

---

## 11. Physical measurements still required (blockers for STAND modes)

`HX_OFFSET`/`HX_SCALE`; $R_{B\leftarrow S}$; servo µs/deg and neutral per axis; Kp, Kd, Q, R
(start from Ziegler–Nichols on the stand per paper §3.1, with the sim used to pick Q/R);
vehicle mass, inertia (bifilar pendulum), $r$ (gimbal pivot to CoM); stand inertia; $C_{N\alpha}$
from OpenRocket; `power.batt_adc_to_mv` (Section 4.5/4.7 — multimeter on the battery vs. raw
`BATT_SENSE_PIN` ADC count, taken together). Each is a `MEASURE` in `params.yaml`; the firmware
will not arm without them (STAND_OPEN's own required subset is narrower — see
`measureGateOk()` in `sequencer.cpp`).
