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
│   ├── kalman2d.h/.cpp          # 2-state (angle, gyro bias) Kalman filter, one instance per axis (paper §3.2)
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
│   ├── link.py                  # serial framing, packet codec (shared definitions with firmware/protocol.h)
│   ├── recorder.py               # redundant ground-side logging
│   └── requirements.txt
├── sim/
│   ├── tvc_params.py            # GENERATED
│   ├── vehicle.py                # rigid-body dynamics, thrust curve, aero disturbance (paper §6.1, §3.4.2)
│   ├── sensors.py                # MPU-6050 noise model, gravity-tilt measurement (paper §6.7, §6.11)
│   ├── run_sim.py                # closed-loop RK4 @ 1 ms, control @ 150 Hz; uses core via ctypes
│   ├── figures/                  # regenerates paper Figs 2–11 (existing plots may be ported here)
│   └── data/estes_e12.eng       # NAR thrust curve points
├── protocol/
│   └── protocol.h                # packet structs (C), also parsed by ground/link.py via a generated JSON
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

All angles in degrees, time in seconds, one independent filter and PID per axis (pitch
about $X_b$, yaw about $Y_b$). No matrix *type* anywhere in `core/`: the 2x2 covariance
in 3.1 is four explicit scalars (p00/p01/p10/p11), not an `Eigen`/array-of-array object.

### 3.1 Kalman filter (paper §3.2, 2-state: angle + gyro bias, $H = [1\ \ 0]$)

State $\hat x = [\hat\theta,\ \hat b]$ (angle, gyro bias). This is a literal port of
`paper/tvc_paper_figures.py`'s `kalman_update()`, using the same $F$, $Q$, and update
algebra, not the simplified 1-state filter an earlier draft of this section described
(that version turned out to be unstable with the paper's gains once sensor noise was
enabled: its derivative had no noise-immune rate source to draw on, see 3.2).

Predict, $F = \begin{bmatrix}1 & -\Delta t\\ 0 & 1\end{bmatrix}$, $B = [\Delta t,\ 0]^T$:
$$\hat\theta_{k|k-1} = \hat\theta_{k-1} + \Delta t\,(\omega_k - \hat b_{k-1}), \qquad
\hat b_{k|k-1} = \hat b_{k-1}$$
$$P_{k|k-1} = F P_{k-1} F^T + Q\,\Delta t, \qquad Q = \operatorname{diag}(Q_\theta,\ Q_b)$$
worked out as four explicit scalars, not a matrix type:
```
n00 = p00 + dt*(dt*p11 - p01 - p10 + q_angle)
n01 = p01 - dt*p11
n10 = p10 - dt*p11
n11 = p11 + q_rate*dt
```

Update (only if accel gate passes; otherwise $\hat x_k = \hat x_{k|k-1}$,
$P_k = P_{k|k-1}$, and the returned gain $K_0 = 0$):
```
y      = z_k - angle           // angle is n00's row of the predicted state
k0     = n00 / (n00 + R)       // angle-measurement gain (returned as AxisOut.K)
k1     = n10 / (n00 + R)
angle += k0*y
bias  += k1*y
n11    = n11 - k1*n01          // all four use the PRE-update n00/n01/n10/n11
n10    = n10 - k1*n00          // (n01, n00 are overwritten last, from their own
n01    = (1 - k0)*n01          //  originals -- see core/kalman2d.cpp for why order matters)
n00    = (1 - k0)*n00
```

`z_k` for pitch is $\operatorname{atan2}(a_y, a_z)$ and for yaw $\operatorname{atan2}(-a_x, a_z)$
in the **body frame** (after $R_{B\leftarrow S}$ remap); confirm sign convention against the
servo axis map in Section 4.5 and write it once in `attitude_from_accel.h`, never inline.
The gate is the paper's "accelerometer unreliable ⇒ rely on gyro" behaviour made explicit; the
sim must use the identical gate, on $|a|$ against `kalman.accel_gate_g`
(Section 2), evaluated every tick, not on flight phase (e.g. "before burnout"). Note for the
sim: during most of a burn, axial specific force is well above 1 g (Estes E12-4 averages
roughly 1.5 g across the sustained-thrust plateau), so the gate is expected to reject the
accelerometer for most of powered flight and pass it mainly near ignition and burnout, unlike
`paper/tvc_paper_figures.py`'s simplified `t <= burn_time` gate. This has a real downstream
consequence, confirmed by `run_sim.py all`: with $\hat x$ seeded at 0, the brief gate-open
window near ignition has to correct the whole initial tip-off angle at once, and the coupled
angle/bias update can attribute part of that one-time correction to $\hat b$ rather than angle.
That bias then contaminates gyro-only dead reckoning for the rest of the burn (the gate stays
closed), producing true-angle drift on the order of a degree even with no sensor noise. This is
a state-estimator artifact of the paper's zero-initialized filter meeting a realistic gate, not
a `core/` bug; see `sim/run_sim.py`'s `cmd_report()` output for the full account.

### 3.2 PID (paper §3.1 eq5, Appendix A.3)

```
e      = 0 - x_hat                     // setpoint is vertical
if (!saturated_last) integral += e*dt   // conditional integration
integral = clamp(integral, ±integral_clamp)
u      = Kp*e + Ki*integral + Kd*d_error
u_cmd  = clamp(u, ±max_deflection)
saturated_last = (u != u_cmd)
```
`Ki` defaults to 0 from `params.yaml` (paper §3.1). `d_error` is supplied by the caller
(`controller.cpp`), computed as $-\,(\omega - \hat b)$ from 3.1's own bias-corrected rate
estimate, **not** a finite difference of `e` computed inside `pid_step`. The paper's own
code comment is explicit about why: a raw `(e - e_prev)/dt` on a noisy angle estimate at
150 Hz amplifies sensor noise into the derivative term badly enough to destabilise the
loop (confirmed independently while porting this: the 1-state-filter architecture an
earlier draft of this section specified diverges with the paper's gains once noise is
on). Do not reintroduce a finite-difference derivative without flagging it.

### 3.3 Servo slew model (paper §3.3, §6.6)
$$\delta_{k} = \delta_{k-1} + \operatorname{clamp}\!\left(u_{cmd,k} - \delta_{k-1},\; \pm\,\dot\delta_{max}\Delta t\right)$$
In firmware this is applied to the *command* sent to the servo (so the logged command is what
the sim models). In the sim it is the actuator model. Same function, same numbers.

### 3.4 `controller.h` API
```cpp
struct AxisState { float x_hat, bias_hat, p00, p01, p10, p11, integral, delta; bool saturated; };
struct AxisOut   { float x_hat, u_raw, u_cmd, delta, K; bool accel_used; };
struct ControlParams { float dt, kp, ki, kd, integral_clamp, max_deflection, q_angle, q_rate, r, slew_deg_per_s; };
AxisOut controller_step(AxisState&, float gyro_deg_s, float accel_tilt_deg, bool accel_gate_ok, const ControlParams&);
```
`K` in `AxisOut` is $k_0$, the angle-measurement gain from 3.1 (0 whenever `accel_used` is
false). `ffi.cpp` exposes `tvc_controller_step(...)` with plain floats for ctypes.

### 3.5 Corrective torque and disturbance (used by sim and by `iae_compare.py`)
$$\tau_{ctrl} = F_T(t)\, r\, \sin\delta, \qquad
\tau_{dist}(\theta) = \tfrac12 \rho v^2 A_{ref} C_{N\alpha}\, \ell\, \theta, \qquad
M_q = -\tfrac12 \rho v A_{ref} C_{N\alpha}\, \ell^2\, \omega, \qquad
I\ddot\theta = \tau_{ctrl} - \tau_{dist} + M_q$$
(paper eq12, eq14, eq18; $M_q$ is the rotational damping derivative, from the local
angle-of-attack increment $\ell\omega/v$ an off-CoM point sees, giving a moment
$\propto \ell$ times that force). $v$ is the integrated axial velocity, not a
thrust-derived approximation: $\dot v = F_T/m - g - \tfrac12\rho v^2 A_{ref} C_D / m$,
$q = \tfrac12\rho v^2$. This replaces `paper/tvc_paper_figures.py`'s dynamics, which had
no destabilising moment at all (only a rate-proportional damping term sized by an
arbitrary coefficient, not $C_{N\alpha}$); see `sim/vehicle.py` and the `sim_overrides`
comment in `params.yaml` for the discrepancies that motivated the change, and
`run_sim.py --legacy-physics` for a mode that still reproduces the paper's original
dynamics exactly, used to validate the controller port independently of this physics fix.
On the static stand $v = 0$, so $\tau_{dist} = M_q = 0$ and only gimbal friction/inertia
remain; the sim must take a `--stand` flag that zeroes aero and uses the stand's measured
inertia (vehicle + gimbal plate) instead of the free-flight inertia.

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
SD              built-in SDIO
```
Final: `PYRO_CONT_PIN = A14 (pin 38)`. AS5600 has a fixed address, hence one per I²C bus. Flag
in the README that the RFM95 RST wire moves from 24 to 9 and that HX711 moves to 6/7.

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

`Record` (packed, 64 bytes): `uint32 t_us; uint8 state; uint8 mode; uint16 fault_bits;
float thrust_N; int16 gyro[3]; int16 accel[3]; float xhat_p, xhat_y; float u_p, u_y;
float delta_p, delta_y; float enc_p, enc_y; float K_p, K_y; uint16 servo_us_p, servo_us_y;
uint8 pyro; uint8 cont_adc8; uint16 loop_us; uint16 crc16;`

### 4.7 Telemetry protocol (`protocol/protocol.h`, shared)
Framing on both radio and USB: `0xAA 0x55 | len | type | payload | crc16`. Max payload 48 bytes
(fits one LoRa frame at SF7/BW125 with margin; ~5.5 kbps usable ⇒ 20 Hz × ~60 B ≈ 9.6 kbps is
**too much** — so downlink is 10 Hz `TELEM` + 1 Hz `STATUS`, and `TELEM` carries only
`t_ms, state, thrust, xhat_p, xhat_y, u_p, u_y, enc_p, enc_y, gyro_z, cont_adc8, batt_mv`
as int16 fixed-point. Full rate stays on SD.)

Downlink: `TELEM` (10 Hz), `STATUS` (1 Hz: mode, fault_bits, params hash, SD bytes, flash
bytes, rssi_of_last_uplink, boot count, missing-MEASURE list), `EVENT` (state changes,
countdown ticks, faults; retransmitted 3×), `ACK`.
Uplink: `HEARTBEAT` (2 Hz from GUI), `SET_MODE`, `SET_PROFILE`, `TARE`, `ARM`, `DISARM`,
`COUNTDOWN`, `FIRE_CONFIRM(token)`, `ABORT`, `DRY_RUN(bool)`, `PING`, `DUMP_FLASH` (USB only).
Every uplink is ACKed with the command's sequence number; GUI shows unacked commands in amber.

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

---

## 11. Physical measurements still required (blockers for STAND modes)

`HX_OFFSET`/`HX_SCALE`; $R_{B\leftarrow S}$; servo µs/deg and neutral per axis; Kp, Kd, Q, R
(start from Ziegler–Nichols on the stand per paper §3.1, with the sim used to pick Q/R);
vehicle mass, inertia (bifilar pendulum), $r$ (gimbal pivot to CoM); stand inertia; $C_{N\alpha}$
from OpenRocket. Each is a `MEASURE` in `params.yaml`; the firmware will not arm without them.
