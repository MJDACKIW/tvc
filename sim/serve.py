"""sim/run_sim.py --serve (SPEC.md Section 6): a virtual vehicle on a local TCP socket that
speaks the real telemetry protocol, so ground/gs.py --sim can be exercised end to end with no
radio. The byte stream is exactly what the Feather bridge puts on USB (downlink frames + its
RSSI/SNR trailer; bare uplink frames back), framed by protocol/telem_codec.py.

Behaviour follows firmware/src/flight/sequencer.cpp + telemetry.cpp (states, transitions,
thresholds, t_ms, EVENT 3x retransmit, TELEM/STATUS rates) plus the SPEC 4.3/4.7 pieces the
firmware doesn't have yet: every SPEC 4.1 mode, radio uplink dispatch with ACKs, and heartbeat
loss -> SAFE. Plant: sim/vehicle.py; sensors: sim/sensors.py; controller: core/ via ctypes.

  STAND_OPEN                 stand locked; open-loop profile (firmware profiles.cpp shapes)
                             drives the gimbal in BURN; the shadow controller only estimates
  STAND_CLOSED               stand pivot free, closed loop from gate-high (SPEC 4.3)
  SIM_ONBOARD / HIL / FLIGHT closed loop, free flight: the accelerometer sees thrust, not
                             gravity, once airborne, so the gate drops it (paper 6.7)

params.yaml values still marked MEASURE are replaced by SERVE_PLACEHOLDERS (printed at start)
for the simulation only. STATUS.missing_measure_bits still reports params.yaml as it stands,
since that is what a vehicle built from it would report.
"""
from __future__ import annotations

import math
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

SIM_DIR = Path(__file__).resolve().parent
REPO_ROOT = SIM_DIR.parent
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import telem_codec as tc  # noqa: E402
import tvc_params as P  # noqa: E402
from core_ffi import Axis, ControlParams  # noqa: E402
from sensors import Imu, LoadCell, encoder  # noqa: E402
from vehicle import AxisDynamics, ThrustCurve  # noqa: E402

SERVE_PLACEHOLDERS = {
    # paper simulation values (simulation/config.py KP, KD, MOI, MOMENT_ARM, ROCKET_MASS)
    "control.kp": 8.5,
    "control.kd": 1.2,
    "vehicle.inertia_pitch_kg_m2": 0.0231,
    "vehicle.inertia_yaw_kg_m2": 0.0231,
    "vehicle.r_gimbal_to_com_m": 0.288,
    "vehicle.mass_kg": 0.661,
    # picked for this sim, not tuned (the paper's Kalman was a 2-state filter; core's is scalar).
    # Kept slow on purpose: core's PID differentiates x_hat (SPEC 3.2), so every accel update
    # step K*(z - x_hat) reaches the servo as Kd*K*(z - x_hat)/dt; with K ~ 0.1 the vibration
    # noise alone drives u to the +-10 deg rails every tick.
    "kalman.q": 0.0005,
    "kalman.r": 2.0,
    # so the GUI's battery readout has something to show; a 2S pack
    "power.batt_adc_to_mv": 1.0,
}
SIM_BATT_MV = 7600
# Stand pivot Coulomb friction for STAND_CLOSED. SPEC 6 makes this a MEASURE parameter (default
# 0) that params.yaml doesn't have yet; without some, the free pivot keeps turning forever after
# burnout. Sim placeholder.
SIM_PIVOT_FRICTION_NM = 0.01

S = {name: i for i, name in enumerate(tc.STATE_NAMES)}
M = {name: i for i, name in enumerate(tc.MODE_NAMES)}
STAND_MODES = {M["STAND_OPEN"], M["STAND_CLOSED"]}

# sequencer.cpp constants
COUNTDOWN_MS = 10000
FIRE_CONFIRM_WINDOW_MS = 2000
TAILOFF_MS = 1500
BURN_TIMEOUT_MS = 6000
THRUST_DETECT_N = 2.0
NO_IGNITION_MS = 1500
BURNOUT_THRUST_N = 1.0
SELFTEST_MS = 2000
# profiles.h
PROFILE_AMP_DEG, STEP_PERIOD_S, RAMP_PERIOD_S = 5.0, 1.0, 2.0
CHIRP_F0, CHIRP_F1, CHIRP_DUR = 0.5, 10.0, 8.0
# plant/sensor constants that are the sim's own (not params)
IGNITER_DELAY_S = 0.12
IGNITER_BURNTHROUGH_S = 0.25
CONTINUITY_RAW = 1850
HX711_HZ = 80.0
PHYS_DT = 0.001


def param(section: str, key: str) -> float:
    v = getattr(getattr(P, section), key)
    if isinstance(v, float) and math.isnan(v):
        return SERVE_PLACEHOLDERS[f"{section}.{key}"]
    return v


def missing_measure_bits(tables: tc.Tables) -> int:
    bits = 0
    for bit, name in tables.measure_bit_names.items():
        section, key = name.split(".", 1)
        v = getattr(getattr(P, section, None), key, 0.0)
        if isinstance(v, float) and math.isnan(v):
            bits |= 1 << bit
    return bits


def profile_targets(profile: int, t: float) -> tuple[float, float]:
    """firmware/src/flight/profiles.cpp targetAngles(), same constants."""
    name = tc.profile_name(profile)
    if name in ("STEP_X", "STEP_Y"):
        v = PROFILE_AMP_DEG if (t % STEP_PERIOD_S) / STEP_PERIOD_S < 0.5 else -PROFILE_AMP_DEG
        return (v, 0.0) if name == "STEP_X" else (0.0, v)
    if name == "RAMP_X":
        ph = (t % RAMP_PERIOD_S) / RAMP_PERIOD_S
        return PROFILE_AMP_DEG * (4.0 * abs(ph - 0.5) - 1.0), 0.0
    if name == "CHIRP_X":
        k = (CHIRP_F1 / CHIRP_F0) ** (1.0 / CHIRP_DUR)
        u = min(t, CHIRP_DUR)
        phase = 2.0 * math.pi * CHIRP_F0 * (k ** u - 1.0) / math.log(k)
        return PROFILE_AMP_DEG * math.sin(phase), 0.0
    return 0.0, 0.0


def slew(current: float, target: float, max_step: float) -> float:
    d = target - current
    return current + max(-max_step, min(max_step, d))


def repo_git_hash4() -> bytes:
    try:
        h = subprocess.run(["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"], capture_output=True,
                           text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        h = ""
    return tc.hex_prefix_bytes(h, 4)


class VirtualVehicle:
    def __init__(self, *, mode: str = "STAND_OPEN", seed: int = 1, theta0=(5.0, -2.0),
                 stand_tilt=(0.6, -0.4), arm_switch_closed: bool = True, loss: float = 0.0,
                 disturbance: tuple[float, float, float] | None = None,
                 tables: tc.Tables | None = None):
        self.tables = tables or tc.Tables.load(REPO_ROOT)
        self.rng = np.random.default_rng(seed)
        self.link_rng = np.random.default_rng(seed + 1)
        self.cp = ControlParams(
            dt=1.0 / P.control.rate_hz, kp=param("control", "kp"), ki=P.control.ki,
            kd=param("control", "kd"), integral_clamp=P.control.integral_clamp_deg_s,
            max_deflection=P.control.max_deflection_deg, q=param("kalman", "q"),
            r=param("kalman", "r"), slew_deg_per_s=P.servo.slew_deg_per_s)
        self.mass = param("vehicle", "mass_kg")
        self.thrust_curve = ThrustCurve()
        self.theta0, self.stand_tilt = theta0, stand_tilt
        self.disturbance = disturbance  # (t after ignition s, torque N*m, duration s), pitch
        self.arm_switch_closed = arm_switch_closed
        self.continuity_raw = CONTINUITY_RAW
        self.loss = loss
        self.mode = M[mode]
        self.profile = 0
        self.dry_run = False
        self.missing_bits = missing_measure_bits(self.tables)
        self.params_hash6 = tc.hex_prefix_bytes(P.PARAMS_HASH, 6)
        self.git4 = repo_git_hash4()

        self.uplink: queue.Queue = queue.Queue()
        self.downlink = None           # callable(bytes), set by the server
        self.t = 0.0
        self.state = S["BOOT"]
        self.t_state = 0.0
        self.fault_bits = 0
        self.last_uplink_rssi = -128
        self.last_heartbeat_t: float | None = None
        self.records = 0
        self._pending_events: list[tuple[float, bytes]] = []
        self._last_beep = -1e9
        self.imu = Imu(self.rng)
        self.loadcell = LoadCell(self.rng)
        # STATUS cumulative fields: since boot, like telemetry.cpp (peak thrust resets at ARM)
        self.iae = [0.0, 0.0]
        self.peak_tilt = [0.0, 0.0]
        self.peak_thrust = 0.0
        self.ctrl_ticks = 0
        self.sat_ticks = [0, 0]
        self._reset_run()
        self._enter(S["SELFTEST"])
        self._event("BOOT")

    # ---- run state ---------------------------------------------------------------------------

    def _reset_run(self):
        """Fresh plant, estimator, and run bookkeeping (boot, SET_MODE, SAFE -> IDLE)."""
        locked = self.mode == M["STAND_OPEN"]
        th0 = self.stand_tilt if locked else self.theta0
        friction = SIM_PIVOT_FRICTION_NM if self.mode == M["STAND_CLOSED"] else 0.0
        self.dyn_p = AxisDynamics(param("vehicle", "inertia_pitch_kg_m2"),
                                  param("vehicle", "r_gimbal_to_com_m"), th0[0], friction)
        self.dyn_y = AxisDynamics(param("vehicle", "inertia_yaw_kg_m2"),
                                  param("vehicle", "r_gimbal_to_com_m"), th0[1], friction)
        self.ax_p, self.ax_y = Axis(P.kalman.p0), Axis(P.kalman.p0)
        self.out_p = self.out_y = None
        self.delta_cmd = [0.0, 0.0]     # what's written to the servos
        self.delta_phys = [0.0, 0.0]    # where the gimbal actually is
        self.ol = [0.0, 0.0]            # STAND_OPEN open-loop profile state (g_ol_*_delta)
        self.t_countdown = self.t_ignite = None
        self.t_below = None
        self.fire_token = None
        self.fire_confirmed = False
        self.launched = False
        self.thrust_true = 0.0
        self.thrust_meas = 0.0
        self._next_hx = 0.0
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 1.0]

    def _enter(self, state: int):
        self.state = state
        self.t_state = self.t
        name = tc.STATE_NAMES[state]
        if name not in ("BOOT", "SELFTEST"):
            self._event(f"STATE_{name}")

    def _abort(self, event_name: str, aux: int = 0):
        self.delta_cmd = [0.0, 0.0]
        self.ol = [0.0, 0.0]
        self._event(event_name, aux)
        self._enter(S["SAFE"])

    # ---- telemetry out (telemetry.cpp) -------------------------------------------------------

    def t_ms(self) -> int:
        if self.state == S["COUNTDOWN"]:
            return -(COUNTDOWN_MS - int((self.t - self.t_countdown) * 1000))
        if self.t_ignite is not None and self.state in (S["IGNITE"], S["BURN"], S["TAILOFF"],
                                                        S["SAFE"], S["FAULT"]):
            return int((self.t - self.t_ignite) * 1000)
        return tc.T_MS_NONE

    def _send(self, ptype, payload: bytes):
        if self.downlink is None or self.link_rng.random() < self.loss:
            return
        rssi = int(round(-62 + self.link_rng.normal(0, 2)))
        snr = int(round(9 + self.link_rng.normal(0, 1)))
        self.downlink(tc.encode_frame(ptype, payload) + tc.bridge_trailer(rssi, snr))

    def _event(self, name: str, aux: int = 0):
        payload = tc.encode_event(self.t_ms(), self.tables.event_code(name), aux)
        self._send(tc.PacketType.EVENT, payload)
        for dt in (0.150, 0.300):  # 2 retransmits -> 3 total (SPEC 4.7)
            self._pending_events.append((self.t + dt, payload))

    def _ack(self, seq: int, ptype: int, result: int, aux: int = 0):
        self._send(tc.PacketType.ACK, tc.encode_ack(seq, ptype, result, aux))

    def send_telem(self):
        flags = 0
        if self.out_p and self.out_p.accel_used:
            flags |= tc.FLAG_PITCH_GATE_OK
        if self.out_y and self.out_y.accel_used:
            flags |= tc.FLAG_YAW_GATE_OK
        if self.out_p and self.out_p.u_raw != self.out_p.u_cmd:
            flags |= tc.FLAG_PITCH_SATURATED
        if self.out_y and self.out_y.u_raw != self.out_y.u_cmd:
            flags |= tc.FLAG_YAW_SATURATED
        if self.state == S["BURN"]:
            flags |= tc.FLAG_TVC_ACTIVE
        if self.dry_run:
            flags |= tc.FLAG_DRY_RUN
        if self.arm_switch_closed:
            flags |= tc.FLAG_ARM_SWITCH_CLOSED
        batt = SIM_BATT_MV - (150 if self.state == S["BURN"] else 0) + self.rng.normal(0, 8)
        o_p, o_y = self.out_p, self.out_y
        self._send(tc.PacketType.TELEM, tc.encode_telem(
            t_ms=self.t_ms(), state=self.state, thrust_N=self.thrust_meas,
            xhat_p_deg=o_p.x_hat if o_p else 0.0, xhat_y_deg=o_y.x_hat if o_y else 0.0,
            u_p_deg=o_p.u_cmd if o_p else 0.0, u_y_deg=o_y.u_cmd if o_y else 0.0,
            enc_p_deg=encoder(self.delta_phys[0], self.rng),
            enc_y_deg=encoder(self.delta_phys[1], self.rng), gyro_z_deg_s=self.gyro[2],
            K_p=o_p.K if o_p else 0.0, K_y=o_y.K if o_y else 0.0, flags=flags,
            cont_adc8=self._continuity_raw() >> 4, batt_mv=int(batt)))

    def send_status(self):
        pct = lambda n: int(100 * n / self.ctrl_ticks) if self.ctrl_ticks else 0  # noqa: E731
        self._send(tc.PacketType.STATUS, tc.encode_status(
            mode=self.mode, fault_bits=self.fault_bits, params_hash6=self.params_hash6,
            sd_bytes_written=105 + 74 * self.records, flash_bytes_written=74 * (self.records // 6),
            rssi_last_uplink_dbm=self.last_uplink_rssi, boot_count=1,
            missing_measure_bits=self.missing_bits, loop_overrun_count=0,
            sat_pct_p=pct(self.sat_ticks[0]), sat_pct_y=pct(self.sat_ticks[1]),
            running_iae_p_degs=self.iae[0], running_iae_y_degs=self.iae[1],
            peak_thrust_N=self.peak_thrust, peak_tilt_p_deg=self.peak_tilt[0],
            peak_tilt_y_deg=self.peak_tilt[1], fw_git_hash4=self.git4))

    # ---- uplink (the dispatch SPEC 4.7 leaves to this phase) ---------------------------------

    def _continuity_raw(self) -> int:
        burnt = (self.t_ignite is not None and not self.dry_run
                 and self.t - self.t_ignite > IGNITER_BURNTHROUGH_S)
        return 0 if burnt else self.continuity_raw

    def _continuity_ok(self) -> bool:
        return P.pyro.continuity_ok_min_adc <= self._continuity_raw() <= P.pyro.continuity_ok_max_adc

    def _handle(self, frame: tc.Frame):
        T, R = tc.PacketType, tc.AckResult
        try:
            ptype = T(frame.ptype)
            d = tc.decode_uplink(ptype, frame.payload)
        except (ValueError, struct.error):
            return  # not an uplink frame we understand; the real vehicle would drop it too
        self.last_uplink_rssi = int(round(-55 + self.link_rng.normal(0, 2)))
        seq, st = d["seq"], self.state
        idle = st == S["IDLE"]

        def ack(result=R.OK, aux=0):
            self._ack(seq, ptype, result, aux)

        if ptype == T.HEARTBEAT:
            self.last_heartbeat_t = self.t
            ack()
        elif ptype == T.PING:
            ack()
        elif ptype == T.SET_MODE:
            if not idle:
                ack(R.REJECTED_STATE)
            elif d["value"] >= len(tc.MODE_NAMES):
                ack(R.REJECTED_ARG)
            else:
                self.mode = d["value"]
                self._reset_run()  # locked vs free stand changes the plant
                ack()
        elif ptype == T.SET_PROFILE:
            if not idle:
                ack(R.REJECTED_STATE)
            elif d["value"] >= len(tc.PROFILE_NAMES):
                ack(R.REJECTED_ARG)
            else:
                self.profile = d["value"]
                ack()
                self._event("PROFILE_SET", self.profile)
        elif ptype == T.DRY_RUN:
            if not idle:
                ack(R.REJECTED_STATE)
            else:
                self.dry_run = bool(d["value"])
                ack()
                self._event("DRY_RUN_ON" if self.dry_run else "DRY_RUN_OFF")
        elif ptype == T.TARE:
            if not idle:
                ack(R.REJECTED_STATE)
            else:
                self.loadcell.tare(self.thrust_true)
                ack()
                self._event("TARE_OK")
        elif ptype == T.ARM:
            if not idle:
                ack(R.REJECTED_STATE)
            elif not self.arm_switch_closed:
                ack(R.REJECTED_ARM_SWITCH)
            elif not self._continuity_ok():
                ack(R.REJECTED_CONTINUITY)
            elif (self.last_heartbeat_t is None or
                  (self.t - self.last_heartbeat_t) * 1000 > P.pyro.heartbeat_timeout_ms):
                ack(R.REJECTED_HEARTBEAT)  # rather than arm and trip HEARTBEAT_LOST at once
            else:
                self.peak_thrust = 0.0  # loadcell::resetPeak()
                ack()
                self._enter(S["ARMED"])
        elif ptype == T.DISARM:
            if st == S["ARMED"]:
                ack()
                self._enter(S["IDLE"])
            elif st == S["SAFE"]:
                ack()
                self._reset_run()
                self._enter(S["IDLE"])
            else:
                ack(R.REJECTED_STATE)
        elif ptype == T.COUNTDOWN:
            if st != S["ARMED"]:
                ack(R.REJECTED_STATE)
            else:
                self.fire_confirmed = False
                self.fire_token = int(self.rng.integers(1, 65536))
                self.t_countdown = self.t
                self._last_beep = -1e9
                ack(aux=self.fire_token)
                self._enter(S["COUNTDOWN"])
        elif ptype == T.FIRE_CONFIRM:
            if st != S["COUNTDOWN"]:
                ack(R.REJECTED_STATE)
            elif d["token"] != self.fire_token:
                ack(R.REJECTED_TOKEN)
            else:
                self.fire_confirmed = True
                ack()
                self._event("FIRE_CONFIRMED")
        elif ptype == T.ABORT:
            ack()
            self._abort("ABORT_MANUAL")
        else:  # DUMP_FLASH is USB-only (SPEC 4.7)
            ack(R.UNSUPPORTED)

    # ---- the 300 Hz loop tick (sequencer.cpp tick()) -----------------------------------------

    def loop_tick(self):
        while True:
            try:
                self._handle(self.uplink.get_nowait())
            except queue.Empty:
                break

        # sensors
        airborne = self.mode not in STAND_MODES and self.launched
        sf = self.thrust_true / (self.mass * 9.80665) if airborne else None
        self.gyro, self.accel = self.imu.sample(self.dyn_p.omega, self.dyn_y.omega,
                                                self.dyn_p.theta, self.dyn_y.theta,
                                                self.thrust_true, sf)
        if self.t >= self._next_hx:
            self._next_hx = self.t + 1.0 / HX711_HZ
            self.thrust_meas = self.loadcell.read(self.thrust_true)
        self.records += 1
        now_ms = self.t * 1000.0
        el_state_ms = (self.t - self.t_state) * 1000.0
        st = self.state

        if st == S["SELFTEST"] and el_state_ms >= SELFTEST_MS:
            self._event("SELFTEST_PASS", self.fault_bits)
            self._enter(S["IDLE"])
        elif st in (S["ARMED"], S["COUNTDOWN"]):
            hb_age = None if self.last_heartbeat_t is None else (self.t - self.last_heartbeat_t) * 1000
            tilt = max(abs(math.degrees(math.atan2(self.accel[1], self.accel[2]))),
                       abs(math.degrees(math.atan2(-self.accel[0], self.accel[2]))))
            if not self.arm_switch_closed:
                self.fault_bits |= 1 << 9
                self._abort("ABORT_ARM_SWITCH_OPENED")
            elif hb_age is None or hb_age > P.pyro.heartbeat_timeout_ms:
                self._abort("HEARTBEAT_LOST", int(min(hb_age or 32767, 32767)))
            elif st == S["ARMED"] and tilt > P.staging.prelaunch_tilt_abort_deg:
                self._abort("ABORT_PRELAUNCH_TILT", int(tilt * 100))
            elif st == S["COUNTDOWN"]:
                el = (self.t - self.t_countdown) * 1000
                if el < COUNTDOWN_MS:
                    rem_s = int((COUNTDOWN_MS - el) // 1000)
                    if now_ms - self._last_beep > (250 if rem_s < 3 else 1000):
                        self._last_beep = now_ms
                        self._event("COUNTDOWN_TICK", rem_s + 1)
                elif self.fire_confirmed:
                    if not self._continuity_ok():
                        self.fault_bits |= 1 << 10
                        self._abort("ABORT_CONTINUITY_LOST")
                    else:
                        self.t_ignite = self.t
                        self._enter(S["IGNITE"])
                elif el - COUNTDOWN_MS >= FIRE_CONFIRM_WINDOW_MS:
                    self._abort("ABORT_NO_FIRE_CONFIRM")
        elif st == S["IGNITE"]:
            if self.dry_run:
                self._enter(S["BURN"])
            elif self.thrust_meas >= THRUST_DETECT_N:
                self._enter(S["BURN"])
            elif (self.t - self.t_ignite) * 1000 >= NO_IGNITION_MS:
                self.fault_bits |= 1 << 11
                self._abort("ABORT_NO_IGNITION")
        elif st == S["BURN"]:
            t_since = self.t - self.t_ignite
            done = False
            if self.dry_run:
                done = t_since >= P.motor.burn_time_s
            elif el_state_ms >= BURN_TIMEOUT_MS:
                done = True
            elif self.thrust_meas < BURNOUT_THRUST_N:
                if self.t_below is None:
                    self.t_below = self.t
                elif self.t - self.t_below >= P.staging.burnout_window_s:
                    done = True
            else:
                self.t_below = None
            if done:
                self.delta_cmd = [0.0, 0.0]
                self.ol = [0.0, 0.0]
                self._enter(S["TAILOFF"])
        elif st == S["TAILOFF"] and el_state_ms >= TAILOFF_MS:
            self._enter(S["SAFE"])

        if self.state not in (S["BOOT"], S["SELFTEST"], S["IDLE"]):
            self.peak_thrust = max(self.peak_thrust, self.thrust_meas)  # reset at ARM

        # telemetry timers
        for due, payload in [e for e in self._pending_events if e[0] <= self.t]:
            self._send(tc.PacketType.EVENT, payload)
        self._pending_events = [e for e in self._pending_events if e[0] > self.t]

    # ---- the 150 Hz control step -------------------------------------------------------------

    def control_tick(self):
        ax, ay, az = self.accel
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        gate = P.kalman.accel_gate_g[0] <= mag <= P.kalman.accel_gate_g[1]  # attitude_from_accel.h
        tilt_p = math.degrees(math.atan2(ay, az))
        tilt_y = math.degrees(math.atan2(-ax, az))
        self.out_p = self.ax_p.step(self.gyro[0], tilt_p, gate, self.cp)
        self.out_y = self.ax_y.step(self.gyro[1], tilt_y, gate, self.cp)

        dt = self.cp.dt
        self.ctrl_ticks += 1
        for i, o in enumerate((self.out_p, self.out_y)):
            self.sat_ticks[i] += int(o.u_raw != o.u_cmd)
            self.iae[i] += abs(o.x_hat) * dt
            self.peak_tilt[i] = max(self.peak_tilt[i], abs(o.x_hat))

        driving = self.state in (S["IGNITE"], S["BURN"])
        if self.mode == M["STAND_OPEN"]:
            tgt = (profile_targets(self.profile, self.t - self.t_ignite)
                   if self.state == S["BURN"] else (0.0, 0.0))
            step = P.servo.slew_deg_per_s * dt
            self.ol = [slew(self.ol[0], tgt[0], step), slew(self.ol[1], tgt[1], step)]
            self.delta_cmd = list(self.ol) if driving else [0.0, 0.0]
        else:
            # closed loop from gate-high (SPEC 4.3); core's slew-limited delta is the command
            self.delta_cmd = [self.out_p.delta, self.out_y.delta] if driving else [0.0, 0.0]

    # ---- the 1 ms physics step ---------------------------------------------------------------

    def physics_step(self):
        t_motor = (self.t - self.t_ignite - IGNITER_DELAY_S) if (
            self.t_ignite is not None and not self.dry_run) else -1.0
        self.thrust_true = self.thrust_curve(t_motor)
        if self.thrust_true > self.mass * 9.80665:
            self.launched = True
        # the servo physically can't outrun its slew rate, whatever it is told
        step = P.servo.slew_deg_per_s * PHYS_DT
        self.delta_phys = [slew(self.delta_phys[i], self.delta_cmd[i], step) for i in range(2)]
        if self.mode == M["STAND_OPEN"]:
            return  # stand locked: attitude does not move
        tau_p = 0.0
        if self.disturbance and self.t_ignite is not None:
            t0, torque, dur = self.disturbance
            if t0 <= self.t - self.t_ignite < t0 + dur:
                tau_p = torque
        self.dyn_p.step(PHYS_DT, self.thrust_true, self.delta_phys[0], tau_p)
        self.dyn_y.step(PHYS_DT, self.thrust_true, self.delta_phys[1])


class Server:
    """One GUI client at a time on host:port; a new connection replaces the old one."""

    def __init__(self, vehicle: VirtualVehicle, host: str, port: int):
        self.vehicle = vehicle
        self.lsock = socket.create_server((host, port))
        self.lsock.settimeout(0.5)
        self.client: socket.socket | None = None
        self.lock = threading.Lock()
        self.stop = threading.Event()
        vehicle.downlink = self.send
        threading.Thread(target=self._accept_loop, name="accept", daemon=True).start()

    def send(self, data: bytes):
        with self.lock:
            if self.client is None:
                return
            try:
                self.client.sendall(data)
            except OSError:
                self.client.close()
                self.client = None

    def _accept_loop(self):
        while not self.stop.is_set():
            try:
                conn, addr = self.lsock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(0.2)
            with self.lock:
                if self.client:
                    self.client.close()
                self.client = conn
            print(f"[serve] GUI connected from {addr[0]}:{addr[1]}", flush=True)
            threading.Thread(target=self._reader, args=(conn,), name="uplink", daemon=True).start()

    def _reader(self, conn: socket.socket):
        parser = tc.FrameParser(bridge_trailer=False)
        while not self.stop.is_set():
            try:
                data = conn.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            for frame in parser.feed(data):
                self.vehicle.uplink.put(frame)
        with self.lock:
            if self.client is conn:
                self.client = None
        print("[serve] GUI disconnected", flush=True)

    def close(self):
        self.stop.set()
        self.lsock.close()
        with self.lock:
            if self.client:
                self.client.close()


def _stdin_console(v: VirtualVehicle):
    """Bench-style knobs for exercising the GUI: `arm open|closed`, `cont ok|open`,
    `loss <0..1>`. Silently absent when stdin isn't a terminal (e.g. spawned by gs.py)."""
    for line in sys.stdin:
        words = line.split()
        if words[:1] == ["arm"] and len(words) == 2:
            v.arm_switch_closed = words[1] == "closed"
        elif words[:1] == ["cont"] and len(words) == 2:
            v.continuity_raw = CONTINUITY_RAW if words[1] == "ok" else 0
        elif words[:1] == ["loss"] and len(words) == 2:
            v.loss = float(words[1])
        else:
            print("commands: arm open|closed, cont ok|open, loss <0..1>", flush=True)
            continue
        print(f"[serve] arm_switch={'closed' if v.arm_switch_closed else 'open'} "
              f"continuity_raw={v.continuity_raw} loss={v.loss:g}", flush=True)


class Stepper:
    """The fixed-rate schedule around a VirtualVehicle, one 1 ms physics step per call:
    300 Hz loop tick, 150 Hz control (SPEC 6: "run controller when t >= next_ctrl_t,
    next_ctrl_t += 1/150"), TELEM and STATUS on their own timers (telemetry.cpp)."""

    def __init__(self, vehicle: VirtualVehicle):
        self.v = vehicle
        self.loop_dt, self.ctrl_dt = 1.0 / P.control.loop_hz, 1.0 / P.control.rate_hz
        self.telem_dt = 1.0 / P.telemetry.telem_rate_hz
        self.status_dt = 1.0 / P.telemetry.status_rate_hz
        self.next_loop = self.next_ctrl = self.next_telem = self.next_status = 0.0

    def step(self):
        v = self.v
        if v.t >= self.next_loop:
            self.next_loop += self.loop_dt
            v.loop_tick()
        if v.t >= self.next_ctrl:
            self.next_ctrl += self.ctrl_dt
            v.control_tick()
        if v.t >= self.next_telem:
            self.next_telem += self.telem_dt
            v.send_telem()
        if v.t >= self.next_status:
            self.next_status += self.status_dt
            v.send_status()
        v.physics_step()
        v.t += PHYS_DT


def run(vehicle: VirtualVehicle, host: str = "127.0.0.1", port: int = 5760,
        stop: threading.Event | None = None, speed: float = 1.0):
    """Advance the vehicle in real time (x speed) until `stop` is set or Ctrl-C."""
    stop = stop or threading.Event()
    server = Server(vehicle, host, port)
    if sys.stdin and sys.stdin.isatty():
        threading.Thread(target=_stdin_console, args=(vehicle,), daemon=True).start()
    stepper = Stepper(vehicle)
    wall0 = time.monotonic()
    try:
        while not stop.is_set():
            stepper.step()
            ahead = vehicle.t / speed - (time.monotonic() - wall0)
            if ahead > 0.004:
                time.sleep(ahead)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
