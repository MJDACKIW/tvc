"""Ground-station state, kept free of Qt so it can be tested headless: ingests decoded packets
(link.RxPacket / LinkNotice), tracks the vehicle's latest TELEM/STATUS, link health, command
ACKs, the event log, and the rolling plot buffers, and decides which commands are enabled
(controls()). gs.py only draws what this holds.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

import _repo  # noqa: F401
import telem_codec as tc
from link import LinkNotice, RxPacket, SentCommand
from recorder import UP

PLOT_WINDOW_S = 30.0
LINK_STALE_S = 1.0     # connection pill goes amber
LINK_LOST_S = 3.0      # full-width red banner (SPEC 5)
ACK_TIMEOUT_S = 2.0
EVENT_DEDUPE_S = 1.0   # EVENTs arrive 3x (SPEC 4.7); collapse the retransmits
LOSS_WINDOW_S = 5.0
CONFIRM_WINDOW_MS = -3000  # CONFIRM FIRE shows for the last 3 s of countdown (SPEC 5)

SEVERITY_ERROR = ("ABORT_", "SELFTEST_FAIL", "STATE_FAULT", "SERVO_RATE_FAULT", "HEARTBEAT_LOST",
                  "TARE_FAILED")
SEVERITY_WARN = ("STATE_ARMED", "STATE_COUNTDOWN", "COUNTDOWN_TICK", "FIRE_CONFIRMED",
                 "STATE_IGNITE", "STATE_BURN", "STATE_TAILOFF", "DRY_RUN_ON")
SEVERITY_OK = ("STATE_SAFE", "SELFTEST_PASS", "TARE_OK", "STATE_IDLE")


def event_severity(name: str) -> str:
    if name.startswith(SEVERITY_ERROR):
        return "error"
    if name.startswith(SEVERITY_WARN):
        return "warn"
    if name.startswith(SEVERITY_OK):
        return "ok"
    return "info"


@dataclass
class LogEntry:
    t: float            # link time
    wall: str           # HH:MM:SS
    severity: str       # info / ok / warn / error
    text: str


@dataclass
class Control:
    enabled: bool
    visible: bool = True
    reason: str = ""
    label: str | None = None


class Series:
    """Rolling (t, y) buffer for one plot curve."""
    def __init__(self):
        self.t: deque = deque()
        self.y: deque = deque()

    def add(self, t, y):
        self.t.append(t)
        self.y.append(y)

    def trim(self, t_min):
        while self.t and self.t[0] < t_min:
            self.t.popleft()
            self.y.popleft()


SERIES = ["thrust", "xhat_p", "xhat_y", "enc_p", "enc_y", "u_p", "u_y", "gyro_z", "K_p", "K_y",
          "gate_p", "gate_y"]


@dataclass
class Model:
    tables: tc.Tables
    telem_hz: float = 10.0
    can_send: bool = True           # False for replay: controls still evaluate, nothing is sent
    telem: dict | None = None
    status: dict | None = None
    t_last_rx: float | None = None
    t_last_telem: float | None = None
    t_first_rx: float | None = None
    state: int | None = None
    mode: int | None = None
    profile: int | None = None      # from PROFILE_SET events / SET_PROFILE ACKs / SD header
    fire_token: int | None = None
    fire_confirmed: bool = False
    t0_link: float | None = None    # link time of T-0 (ignition gate-high)
    iae_since_t0_p: float = 0.0
    iae_since_t0_y: float = 0.0
    rssi: int | None = None
    snr: int | None = None
    ack_latency_ms: float | None = None
    pending: dict = field(default_factory=dict)       # seq -> SentCommand
    last_result: dict = field(default_factory=dict)   # ptype -> ("ok"|"rejected"|"timeout", text)
    rx_counts: dict = field(default_factory=dict)
    log: list = field(default_factory=list)
    safe_entered: bool = False      # latched on the transition into SAFE; gs.py clears it
    link_connected: bool = False

    def __post_init__(self):
        self.series = {name: Series() for name in SERIES}
        self.rssi_hist: deque = deque(maxlen=120)
        self._telem_times: deque = deque()
        self._recent_events: deque = deque()
        self._last_tick_aux = None

    # ---- ingest ----------------------------------------------------------------------------

    def add_log(self, t: float, severity: str, text: str):
        self.log.append(LogEntry(t, time.strftime("%H:%M:%S"), severity, text))

    def ingest(self, item):
        if isinstance(item, LinkNotice):
            self.add_log(item.t, item.level, item.text)
            return
        pkt: RxPacket = item
        if pkt.direction == UP:
            d = pkt.decoded
            if pkt.ptype != tc.PacketType.HEARTBEAT:
                extra = f" value={d['value']}" if "value" in d else (
                    f" token={d['token']}" if "token" in d else "")
                self.add_log(pkt.t, "info",
                             f"uplink {tc.packet_name(pkt.ptype)} seq={d['seq']}{extra}")
            return
        self.t_last_rx = pkt.t
        if self.t_first_rx is None:
            self.t_first_rx = pkt.t
        name = tc.packet_name(pkt.ptype)
        self.rx_counts[name] = self.rx_counts.get(name, 0) + 1
        if pkt.rssi is not None:
            self.rssi, self.snr = pkt.rssi, pkt.snr
            self.rssi_hist.append(pkt.rssi)
        if pkt.ptype == tc.PacketType.TELEM:
            self._on_telem(pkt)
        elif pkt.ptype == tc.PacketType.STATUS:
            self.status = pkt.decoded
            self.mode = pkt.decoded["mode"]
        elif pkt.ptype == tc.PacketType.EVENT:
            self._on_event(pkt)
        elif pkt.ptype == tc.PacketType.ACK:
            self._on_ack(pkt)

    def _on_telem(self, pkt: RxPacket):
        d = pkt.decoded
        prev_t = self.t_last_telem
        self.telem = d
        self.t_last_telem = pkt.t
        self._telem_times.append(pkt.t)
        self._set_state(d["state"], pkt.t)
        t_ms = d["t_ms"]
        if t_ms != tc.T_MS_NONE and t_ms >= 0 and self.t0_link is None:
            self.t0_link = pkt.t - t_ms / 1000.0
        if self.t0_link is not None and t_ms != tc.T_MS_NONE and t_ms >= 0 and prev_t is not None:
            dt = pkt.t - prev_t
            self.iae_since_t0_p += abs(d["xhat_p_deg"]) * dt
            self.iae_since_t0_y += abs(d["xhat_y_deg"]) * dt
        s, t = self.series, pkt.t
        s["thrust"].add(t, d["thrust_N"])
        for k in ("xhat_p", "xhat_y", "enc_p", "enc_y", "u_p", "u_y"):
            s[k].add(t, d[f"{k}_deg"])
        s["gyro_z"].add(t, d["gyro_z_deg_s"])
        s["K_p"].add(t, d["K_p"])
        s["K_y"].add(t, d["K_y"])
        s["gate_p"].add(t, 1.0 if d["flags"] & tc.FLAG_PITCH_GATE_OK else 0.0)
        s["gate_y"].add(t, 1.0 if d["flags"] & tc.FLAG_YAW_GATE_OK else 0.0)

    def _set_state(self, state: int, t: float):
        if state == self.state:
            return
        prev, self.state = self.state, state
        name = tc.state_name(state)
        if name == "COUNTDOWN":
            self.fire_confirmed = False
            self._last_tick_aux = None
        if name in ("IDLE", "ARMED"):
            # a new run: forget the last one's T-0, token, and ground-side IAE
            self.t0_link = None
            self.fire_token = None
            self.iae_since_t0_p = self.iae_since_t0_y = 0.0
        if name == "SAFE" and prev is not None:
            self.safe_entered = True

    def _on_event(self, pkt: RxPacket):
        d = pkt.decoded
        key = (d["t_ms"], d["event_code"], d["aux"])
        while self._recent_events and pkt.t - self._recent_events[0][0] > EVENT_DEDUPE_S:
            self._recent_events.popleft()
        if any(k == key for _, k in self._recent_events):
            return
        self._recent_events.append((pkt.t, key))
        name = d["event_name"]
        text = name
        if name == "COUNTDOWN_TICK":
            # the firmware beeps (and sends a tick) 4x/s in the last 3 s; log each value once
            if d["aux"] == self._last_tick_aux:
                return
            self._last_tick_aux = d["aux"]
            text = f"countdown tick {d['aux']}"
        elif name == "PROFILE_SET":
            self.profile = d["aux"]
            text = f"PROFILE_SET {tc.profile_name(d['aux'])}"
        elif name == "FIRE_CONFIRMED":
            self.fire_confirmed = True
        elif name == "SELFTEST_FAIL":
            text = f"SELFTEST_FAIL faults: {', '.join(tc.fault_names(d['aux'] & 0xFFFF))}"
        elif d["aux"]:
            text = f"{name} aux={d['aux']}"
        if d["t_ms"] != tc.T_MS_NONE:
            text = f"[T{d['t_ms'] / 1000:+.2f}] {text}"
        if pkt.note:
            text += f"  ({pkt.note})"
        self.add_log(pkt.t, event_severity(name), text)

    def _on_ack(self, pkt: RxPacket):
        d = pkt.decoded
        sent: SentCommand | None = self.pending.pop(d["seq"], None)
        if sent is not None:
            self.ack_latency_ms = (pkt.t - sent.t) * 1000.0
        if d["cmd_type"] == tc.PacketType.HEARTBEAT:
            return
        ok = d["result"] == tc.AckResult.OK
        what = d["cmd_name"]
        if d["cmd_type"] == tc.PacketType.COUNTDOWN and ok:
            self.fire_token = d["aux"]
            what += f" (fire_token {d['aux']})"
        if d["cmd_type"] == tc.PacketType.SET_PROFILE and ok and sent is not None:
            self.profile = sent.value
        if d["cmd_type"] == tc.PacketType.SET_MODE and ok and sent is not None:
            self.mode = sent.value
        if d["cmd_type"] == tc.PacketType.FIRE_CONFIRM and ok:
            self.fire_confirmed = True
        self.last_result[d["cmd_type"]] = ("ok" if ok else "rejected", d["result_name"])
        if ok:
            self.add_log(pkt.t, "ok", f"ACK {what} seq={d['seq']}")
        else:
            self.add_log(pkt.t, "error", f"REJECTED {what} seq={d['seq']}: {d['result_name']}")

    def sent(self, cmd: SentCommand):
        self.pending[cmd.seq] = cmd
        if cmd.ptype != tc.PacketType.HEARTBEAT:
            self.last_result.pop(cmd.ptype, None)

    def check_timeouts(self, now: float):
        for seq, cmd in list(self.pending.items()):
            if now - cmd.t > ACK_TIMEOUT_S:
                del self.pending[seq]
                if cmd.ptype != tc.PacketType.HEARTBEAT:
                    self.last_result[cmd.ptype] = ("timeout", "no ACK")
                    self.add_log(now, "warn",
                                 f"no ACK for {tc.packet_name(cmd.ptype)} seq={seq} "
                                 f"after {ACK_TIMEOUT_S:.0f} s")

    def awaiting_ack(self, ptype: int) -> bool:
        return any(c.ptype == ptype for c in self.pending.values())

    def trim(self, now: float):
        for s in self.series.values():
            s.trim(now - PLOT_WINDOW_S - 1.0)
        while self._telem_times and self._telem_times[0] < now - LOSS_WINDOW_S:
            self._telem_times.popleft()

    # ---- derived ---------------------------------------------------------------------------

    def link_age(self, now: float) -> float | None:
        return None if self.t_last_rx is None else now - self.t_last_rx

    def link_state(self, now: float) -> str:
        age = self.link_age(now)
        if age is None:
            return "waiting"
        if age > LINK_LOST_S:
            return "lost"
        if age > LINK_STALE_S:
            return "stale"
        return "ok"

    def loss_pct(self, now: float) -> float | None:
        """TELEM arrivals over the last LOSS_WINDOW_S against the vehicle's nominal rate."""
        if self.t_first_rx is None:
            return None
        window = min(LOSS_WINDOW_S, now - self.t_first_rx)
        if window < 2.0:
            return None
        expected = window * self.telem_hz
        got = sum(1 for t in self._telem_times if t >= now - window)
        return max(0.0, 100.0 * (1.0 - got / expected))

    def mission_clock_ms(self, now: float) -> int | None:
        """T+/- in ms, advanced locally between 10 Hz TELEMs; None before any ignition is
        scheduled (TELEM.t_ms sentinel)."""
        if not self.telem or self.telem["t_ms"] == tc.T_MS_NONE:
            return None
        return int(self.telem["t_ms"] + (now - self.t_last_telem) * 1000.0)

    def flag(self, bit: int) -> bool | None:
        return None if not self.telem else bool(self.telem["flags"] & bit)

    def continuity_ok(self) -> bool | None:
        if not self.telem:
            return None
        try:
            import tvc_params
            lo, hi = tvc_params.pyro.continuity_ok_min_adc >> 4, tvc_params.pyro.continuity_ok_max_adc >> 4
        except (ImportError, AttributeError):
            lo, hi = 200 >> 4, 3800 >> 4
        return lo <= self.telem["cont_adc8"] <= hi

    def state_name(self) -> str:
        return "--" if self.state is None else tc.state_name(self.state)

    def mode_name(self) -> str:
        return "--" if self.mode is None else tc.mode_name(self.mode)

    def profile_name(self) -> str:
        return "--" if self.profile is None else tc.profile_name(self.profile)

    # ---- command enabling ------------------------------------------------------------------

    def controls(self, now: float) -> dict[str, Control]:
        """Which commands are enabled, visible, and why not. Driven only by what the vehicle
        last reported (state, mode, arm switch), never by what the GUI assumes it did."""
        st = self.state_name()
        fresh = self.link_state(now) in ("ok", "stale")
        c: dict[str, Control] = {}

        def ctl(ok: bool, reason: str = "", **kw) -> Control:
            if not fresh:
                return Control(False, reason="no telemetry from the vehicle", **kw)
            return Control(ok, reason="" if ok else reason, **kw)

        idle = st == "IDLE"
        for m in tc.MODE_NAMES:
            c[f"mode:{m}"] = ctl(idle, f"mode changes only in IDLE (vehicle is {st})")
        show_profiles = self.mode_name() == "STAND_OPEN"
        for p in tc.PROFILE_NAMES:
            c[f"profile:{p}"] = ctl(idle, f"profile changes only in IDLE (vehicle is {st})",
                                    visible=show_profiles)
        c["dry_run"] = ctl(idle, f"DRY RUN changes only in IDLE (vehicle is {st})")
        c["tare"] = ctl(idle, f"TARE only in IDLE (vehicle is {st})")
        c["ping"] = ctl(True)

        if not idle:
            c["arm"] = ctl(False, f"ARM only from IDLE (vehicle is {st})")
        elif not self.flag(tc.FLAG_ARM_SWITCH_CLOSED):
            c["arm"] = ctl(False, "physical arm switch is OPEN")
        else:
            c["arm"] = ctl(True)

        c["countdown"] = ctl(st == "ARMED", f"COUNTDOWN only from ARMED (vehicle is {st})")

        clock = self.mission_clock_ms(now)
        in_window = st == "COUNTDOWN" and clock is not None and clock >= CONFIRM_WINDOW_MS
        if self.fire_confirmed:
            c["confirm_fire"] = Control(False, visible=in_window, reason="fire confirmed")
        elif self.fire_token is None:
            c["confirm_fire"] = Control(False, visible=in_window,
                                        reason="no fire_token (COUNTDOWN was not ACKed)")
        else:
            c["confirm_fire"] = ctl(True, visible=in_window)

        c["abort"] = Control(True)
        if st == "SAFE":
            c["disarm"] = ctl(True, label="RESET → IDLE")
        else:
            c["disarm"] = ctl(st == "ARMED", f"DISARM only from ARMED or SAFE (vehicle is {st})",
                              label="DISARM")
        return c
