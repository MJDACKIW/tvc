#!/usr/bin/env python3
"""TVC ground station (SPEC.md Section 5). PyQt6 + pyqtgraph, dark theme.

    ground/venv/bin/python ground/gs.py --replay data/synthetic/SF_SYNTH.BIN
    ground/venv/bin/python ground/gs.py --sim            # starts sim/run_sim.py --serve if needed
    ground/venv/bin/python ground/gs.py --port /dev/tty.usbmodemXXXX   # Feather bridge (live)

Header: link pill (RSSI/SNR sparkline, loss, ACK latency), MODE, STATE badge, mission clock,
params-hash match, firmware hash. Left: commands, enabled only as the vehicle's reported state
allows. Center: five 30 s rolling plots on a shared time axis. Right: every TELEM/STATUS field.
Bottom: event log, newest first. Space = ABORT.

The live serial path uses the same codec and link code as --sim, but has not been run against
real hardware yet: the vehicle firmware does not act on radio uplink commands other than
HEARTBEAT (SPEC 4.7), so commands sent live will currently go unACKed.
"""
from __future__ import annotations

import argparse
import math
import queue
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

import _repo
import telem_codec as tc
from link import Link, ReplayLink, SerialLink, TcpLink, find_feather_ports
from model import Model
from recorder import GroundRecorder

import _qt_env
_qt_env.check_plugins_not_hidden()  # see _qt_env.py

from PyQt6.QtCore import QElapsedTimer, Qt, QTimer, pyqtSignal  # noqa: E402
from PyQt6.QtGui import QColor, QFont, QKeySequence, QShortcut  # noqa: E402
from PyQt6.QtWidgets import (QApplication, QButtonGroup, QFrame, QGridLayout, QHBoxLayout,  # noqa: E402
                             QLabel, QListWidget, QListWidgetItem, QMainWindow, QPushButton,
                             QSizePolicy, QSplitter, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                             QWidget)
import pyqtgraph as pg  # noqa: E402

REPO_ROOT = _repo.REPO_ROOT

# ---- palette ----------------------------------------------------------------------------------
BG, PANEL, PANEL2, BORDER = "#0d1117", "#141a22", "#1a212b", "#29313d"
TEXT, DIM, FAINT = "#dde3ea", "#8e98a6", "#5d6674"
PITCH, YAW = "#5aa9ff", "#ffab4a"
PITCH_ENC, YAW_ENC = "#a9d1ff", "#ffd29a"
GREEN, AMBER, RED, ORANGE, BLUE = "#2fbf71", "#f2a93b", "#ef4b4b", "#fb8c3c", "#5aa9ff"
STATE_COLORS = {
    "BOOT": ("#374151", TEXT), "SELFTEST": ("#374151", TEXT), "IDLE": ("#4b5563", TEXT),
    "ARMED": (AMBER, "#1a1204"), "COUNTDOWN": (AMBER, "#1a1204"), "IGNITE": (RED, "#fff"),
    "BURN": (RED, "#fff"), "TAILOFF": (ORANGE, "#1a0e04"), "SAFE": (GREEN, "#04140b"),
    "FAULT": ("#7f1d1d", "#fecaca"),
}
SEVERITY_COLORS = {"info": "#aeb6c2", "ok": GREEN, "warn": AMBER, "error": RED}

STYLE = f"""
QWidget {{ background: {BG}; color: {TEXT}; font-size: 12px; }}
QFrame#panel, QFrame#header {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px; }}
QLabel {{ background: transparent; }}
QLabel#section {{ color: {DIM}; font-size: 10px; font-weight: 600; letter-spacing: 1px; }}
QLabel#reason {{ color: {FAINT}; font-size: 10px; }}
QLabel#dim {{ color: {DIM}; }}
QPushButton {{ background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 6px; padding: 6px 8px;
              color: {TEXT}; }}
QPushButton:hover {{ border-color: #45505f; }}
QPushButton:disabled {{ color: {FAINT}; background: #12171e; border-color: #1f2630; }}
QPushButton[pill="true"] {{ border-radius: 12px; padding: 4px 6px; font-size: 11px; }}
QPushButton[pill="true"]:checked {{ background: #223349; border-color: {BLUE}; color: #e8f2ff; }}
QPushButton[pill="true"]:checked:disabled {{ background: #1a2433; border-color: #2d4461; color: #9fb6d1; }}
QPushButton[pending="true"] {{ border: 1px solid {AMBER}; }}
QPushButton[result="rejected"] {{ border: 1px solid {RED}; }}
QPushButton[result="timeout"] {{ border: 1px solid {AMBER}; }}
QPushButton#arm {{ font-weight: 700; padding: 9px; }}
QPushButton#arm:enabled {{ background: #3a2a0c; border-color: {AMBER}; color: #ffe2b0; }}
QPushButton#countdown {{ font-weight: 800; font-size: 15px; padding: 12px; }}
QPushButton#countdown:enabled {{ background: #3a2a0c; border-color: {AMBER}; color: #ffe2b0; }}
QPushButton#abort {{ background: #5c1111; border: 2px solid {RED}; color: #fff; font-weight: 800;
                    font-size: 16px; padding: 12px; border-radius: 8px; }}
QPushButton#abort:hover {{ background: #7a1515; }}
QPushButton#dryrun:checked {{ background: #2c2410; border-color: {AMBER}; color: #ffd48a; }}
QTreeWidget {{ background: {PANEL}; border: none; alternate-background-color: {PANEL}; }}
QTreeWidget::item {{ padding: 1px 0; }}
QHeaderView::section {{ background: {PANEL}; color: {DIM}; border: none; }}
QListWidget {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 8px;
              font-family: Menlo, monospace; font-size: 11px; }}
QSplitter::handle {{ background: {BG}; }}
QStatusBar {{ color: {DIM}; }}
QToolTip {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {BORDER}; }}
"""


def restyle(w: QWidget, **props):
    changed = False
    for k, v in props.items():
        if w.property(k) != v:
            w.setProperty(k, v)
            changed = True
    if changed:
        w.style().unpolish(w)
        w.style().polish(w)


def css(w: QWidget, sheet: str):
    """setStyleSheet only on change: it re-polishes the widget, far too slow to do at 60 Hz."""
    if w.styleSheet() != sheet:
        w.setStyleSheet(sheet)


CLOCK_CSS = "font-family: Menlo, monospace; font-size: 26px; font-weight: 700; color: {};"


def _steps(t, y):
    """Sample-and-hold rendering of a 0/1 flag: each value holds until the next sample."""
    if len(t) < 2:
        return t, y
    return np.repeat(t, 2)[1:], np.repeat(y, 2)[:-1]


def section(text: str) -> QLabel:
    lbl = QLabel(text.upper())
    lbl.setObjectName("section")
    return lbl


def fmt(v, spec=".2f", unit="", none="--"):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return none
    return f"{v:{spec}}{unit}"


class HoldButton(QPushButton):
    """Press-and-hold confirm (SPEC 5: CONFIRM FIRE requires a 500 ms hold). Fires `confirmed`
    once the hold completes; releasing early cancels. The fill shows hold progress."""
    confirmed = pyqtSignal()

    def __init__(self, text: str, hold_ms: int = 500):
        super().__init__(text)
        self.hold_ms = hold_ms
        self._clock = QElapsedTimer()
        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._done = False
        self._frac = -1.0
        self._paint(0.0)
        self.pressed.connect(self._start)
        self.released.connect(self._cancel)

    def _paint(self, frac: float):
        f = max(0.0, min(1.0, frac))
        self._frac = f
        css(self, 
            "QPushButton { color: #fff; font-weight: 800; font-size: 14px; padding: 12px;"
            f" border: 2px solid {RED}; border-radius: 8px; background: qlineargradient("
            f"x1:0, y1:0, x2:1, y2:0, stop:0 #d02a2a, stop:{f:.3f} #d02a2a, "
            f"stop:{min(1.0, f + 0.001):.3f} #4a1010, stop:1 #4a1010); }}"
            "QPushButton:disabled { color: #b88; border-color: #6b2a2a; background: #2a1414; }")

    def _start(self):
        self._done = False
        self._clock.start()
        self._timer.start()

    def _tick(self):
        if not (self.isEnabled() and self.isVisible()):  # window closed under the finger
            self.reset()
            return
        frac = self._clock.elapsed() / self.hold_ms
        self._paint(frac)
        if frac >= 1.0 and not self._done:
            self._done = True
            self._timer.stop()
            self.confirmed.emit()

    def _cancel(self):
        self._timer.stop()
        if not self._done:
            self._paint(0.0)

    def reset(self):
        if self._timer.isActive() or self._done or self._frac:
            self._timer.stop()
            self._done = False
            self._paint(0.0)


class Badge(QLabel):
    def __init__(self, text="", big=False):
        super().__init__(text)
        self.big = big
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_colors("#374151", TEXT)

    def set_colors(self, bg, fg, border=None):
        size = "22px" if self.big else "12px"
        pad = "6px 18px" if self.big else "4px 10px"
        weight = 800 if self.big else 600
        css(self, f"QLabel {{ background: {bg}; color: {fg}; border-radius: "
                           f"{'10' if self.big else '11'}px; padding: {pad}; font-size: {size}; "
                           f"font-weight: {weight}; border: 1px solid {border or bg}; }}")


class MainWindow(QMainWindow):
    def __init__(self, link: Link, model: Model, recorder: GroundRecorder | None = None):
        super().__init__()
        self.link, self.model, self.recorder = link, model, recorder
        self.replay = isinstance(link, ReplayLink)
        self._sent_value = {}   # ptype -> value, to mark the pending pill
        self._log_rendered = 0
        self._last_state = None
        self.setWindowTitle(f"TVC Ground Station: {link.description}")
        self.resize(1560, 960)
        self._build()
        QShortcut(QKeySequence(Qt.Key.Key_Space), self, activated=self.cmd_abort,
                  context=Qt.ShortcutContext.ApplicationShortcut)
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self.tick)
        self.redraw_timer.start(16)  # ~60 Hz, decoupled from the link thread (SPEC 5)
        self.hb_timer = QTimer(self)
        self.hb_timer.timeout.connect(self.heartbeat)
        if link.can_send:
            self.hb_timer.start(500)  # HEARTBEAT at 2 Hz (SPEC 4.7)

    # ---- layout ------------------------------------------------------------------------------

    def _build(self):
        root = QWidget()
        v = QVBoxLayout(root)
        v.setContentsMargins(10, 10, 10, 6)
        v.setSpacing(8)
        self.banner = QLabel()
        self.banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.banner.setStyleSheet(f"background: {RED}; color: white; font-weight: 800; "
                                  "font-size: 15px; padding: 8px; border-radius: 6px;")
        self.banner.hide()
        v.addWidget(self.banner)
        v.addWidget(self._build_header())
        body = QSplitter(Qt.Orientation.Horizontal)
        body.addWidget(self._build_commands())
        body.addWidget(self._build_plots())
        body.addWidget(self._build_telemetry())
        body.setStretchFactor(0, 0)
        body.setStretchFactor(1, 1)
        body.setStretchFactor(2, 0)
        body.setSizes([290, 900, 340])
        v.addWidget(body, 1)
        self.log_list = QListWidget()
        self.log_list.setFixedHeight(150)
        self.log_list.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        v.addWidget(self.log_list)
        self.setCentralWidget(root)
        self.statusBar().showMessage(self.link.description)

    def _build_header(self):
        h = QFrame()
        h.setObjectName("header")
        lay = QHBoxLayout(h)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(14)

        self.conn_pill = QFrame()
        cl = QHBoxLayout(self.conn_pill)
        cl.setContentsMargins(10, 4, 10, 4)
        cl.setSpacing(8)
        self.conn_dot = QLabel("●")
        self.conn_text = QLabel("connecting…")
        self.conn_text.setStyleSheet("font-size: 11px;")
        self.conn_text.setMinimumWidth(360)
        self.spark = pg.PlotWidget(background=None)
        self.spark.setFixedSize(84, 26)
        self.spark.hideAxis("left")
        self.spark.hideAxis("bottom")
        self.spark.setMouseEnabled(False, False)
        self.spark.hideButtons()
        self.spark.setYRange(-120, -30)
        self.spark_curve = self.spark.plot(pen=pg.mkPen(GREEN, width=1.4))
        cl.addWidget(self.conn_dot)
        cl.addWidget(self.conn_text)
        cl.addWidget(self.spark)
        lay.addWidget(self.conn_pill)

        self.mode_badge = Badge("MODE --")
        lay.addWidget(self.mode_badge)
        lay.addStretch(1)
        self.state_badge = Badge("--", big=True)
        self.state_badge.setMinimumWidth(190)
        lay.addWidget(self.state_badge)
        self.clock = QLabel("T  --.-")
        css(self.clock, CLOCK_CSS.format(FAINT))
        self.clock.setMinimumWidth(170)
        lay.addWidget(self.clock)
        lay.addStretch(1)
        self.hash_label = QLabel("params --")
        self.fw_label = QLabel("fw --")
        for w in (self.hash_label, self.fw_label):
            w.setStyleSheet("font-family: Menlo, monospace; font-size: 11px;")
            lay.addWidget(w)
        return h

    def _build_commands(self):
        p = QFrame()
        p.setObjectName("panel")
        p.setMinimumWidth(270)
        p.setMaximumWidth(330)
        v = QVBoxLayout(p)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(6)
        self.btn = {}
        self.reason = {}

        if self.replay:
            note = QLabel("REPLAY: buttons show what the vehicle allowed;\nnothing is sent")
            note.setStyleSheet(f"color: {BLUE}; font-size: 10px;")
            v.addWidget(note)

        v.addWidget(section("Mode"))
        grid = QGridLayout()
        grid.setSpacing(5)
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(False)
        for i, name in enumerate(tc.MODE_NAMES):
            b = self._pill(name.replace("_", " "))
            b.clicked.connect(lambda _=False, i=i: self.send(tc.PacketType.SET_MODE, i))
            grid.addWidget(b, i // 2, i % 2)
            self.btn[f"mode:{name}"] = b
        v.addLayout(grid)
        self.reason["mode"] = self._reason_label(v)

        self.profile_box = QWidget()
        pv = QVBoxLayout(self.profile_box)
        pv.setContentsMargins(0, 4, 0, 0)
        pv.setSpacing(5)
        pv.addWidget(section("Profile (STAND_OPEN)"))
        pgrid = QGridLayout()
        pgrid.setSpacing(5)
        for i, name in enumerate(tc.PROFILE_NAMES):
            b = self._pill(name.replace("_", " "))
            b.clicked.connect(lambda _=False, i=i: self.send(tc.PacketType.SET_PROFILE, i))
            pgrid.addWidget(b, i // 3, i % 3)
            self.btn[f"profile:{name}"] = b
        pv.addLayout(pgrid)
        v.addWidget(self.profile_box)

        v.addSpacing(4)
        row = QHBoxLayout()
        b = QPushButton("DRY RUN")
        b.setObjectName("dryrun")
        b.setCheckable(True)
        b.setToolTip("Burnout detector off, time-based BURN exit; igniter output must be the LED")
        b.clicked.connect(self.cmd_dry_run)
        self.btn["dry_run"] = b
        row.addWidget(b, 2)
        for key, text, ptype in (("tare", "TARE", tc.PacketType.TARE),
                                 ("ping", "PING", tc.PacketType.PING)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, t=ptype: self.send(t))
            self.btn[key] = b
            row.addWidget(b, 1)
        v.addLayout(row)

        v.addSpacing(6)
        v.addWidget(section("Sequence"))
        b = QPushButton("ARM")
        b.setObjectName("arm")
        b.clicked.connect(lambda: self.send(tc.PacketType.ARM))
        self.btn["arm"] = b
        v.addWidget(b)
        self.reason["arm"] = self._reason_label(v)
        b = QPushButton("COUNTDOWN")
        b.setObjectName("countdown")
        b.clicked.connect(lambda: self.send(tc.PacketType.COUNTDOWN))
        self.btn["countdown"] = b
        v.addWidget(b)
        self.reason["countdown"] = self._reason_label(v)
        self.countdown_label = QLabel("")
        self.countdown_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.countdown_label.setStyleSheet(f"color: {AMBER}; font-family: Menlo, monospace; "
                                           "font-size: 30px; font-weight: 800;")
        v.addWidget(self.countdown_label)
        self.confirm = HoldButton("HOLD TO CONFIRM FIRE")
        self.confirm.confirmed.connect(self.cmd_fire_confirm)
        self.btn["confirm_fire"] = self.confirm
        v.addWidget(self.confirm)
        self.reason["confirm_fire"] = self._reason_label(v)
        b = QPushButton("DISARM")
        b.clicked.connect(lambda: self.send(tc.PacketType.DISARM))
        self.btn["disarm"] = b
        v.addWidget(b)

        v.addStretch(1)
        b = QPushButton("ABORT   (space)")
        b.setObjectName("abort")
        b.clicked.connect(self.cmd_abort)
        self.btn["abort"] = b
        v.addWidget(b)

        v.addSpacing(6)
        v.addWidget(section("Vehicle"))
        ro = QGridLayout()
        ro.setVerticalSpacing(3)
        self.ro = {}
        for r, (key, label) in enumerate((("cont", "Continuity"), ("batt", "Battery"),
                                          ("armsw", "Arm switch"))):
            lbl = QLabel(label)
            lbl.setObjectName("dim")
            val = QLabel("--")
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            val.setStyleSheet("font-family: Menlo, monospace;")
            ro.addWidget(lbl, r, 0)
            ro.addWidget(val, r, 1)
            self.ro[key] = val
        v.addLayout(ro)

        for b in self.btn.values():  # Space is ABORT; never let it "click" a focused button
            b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        return p

    def _pill(self, text):
        b = QPushButton(text)
        b.setCheckable(True)
        b.setProperty("pill", True)
        b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        return b

    def _reason_label(self, layout):
        lbl = QLabel("")
        lbl.setObjectName("reason")
        lbl.setWordWrap(True)
        layout.addWidget(lbl)
        return lbl

    def _build_plots(self):
        pg.setConfigOptions(antialias=True, foreground=DIM)
        w = pg.GraphicsLayoutWidget()
        w.setBackground(PANEL)
        w.ci.setSpacing(4)
        self.plots = []
        self.curves = {}

        def mk(row, title, units):
            p = w.addPlot(row=row, col=0)
            p.setLabel("left", title, units=units, **{"color": DIM, "font-size": "10pt"})
            p.showGrid(x=True, y=True, alpha=0.12)
            p.setMouseEnabled(x=False, y=True)
            p.hideButtons()
            p.getAxis("left").setWidth(62)
            p.getAxis("left").enableAutoSIPrefix(False)
            p.addLegend(offset=(8, 2), labelTextSize="8pt", brush=pg.mkBrush(20, 26, 34, 200),
                        pen=pg.mkPen(BORDER), colCount=4)
            if self.plots:
                p.setXLink(self.plots[0])
            self.plots.append(p)
            return p

        def curve(p, key, name, color, width=1.6, style=Qt.PenStyle.SolidLine):
            c = p.plot(name=name, pen=pg.mkPen(color, width=width, style=style))
            self.curves[key] = c
            return c

        p = mk(0, "Thrust", "N")
        curve(p, "thrust", "load cell", "#e9edf2", 1.8)
        self.nominal_curve = p.plot(name="E12-4 nominal", pen=pg.mkPen("#7c8594", width=1.2,
                                                                      style=Qt.PenStyle.DashLine))
        try:
            from vehicle import load_eng
            import tvc_params
            self.nominal_t, self.nominal_f = load_eng(REPO_ROOT / tvc_params.motor.thrust_curve_file)
        except (ImportError, OSError, ValueError):
            self.nominal_t = self.nominal_f = None

        p = mk(1, "Attitude", "deg")
        curve(p, "xhat_p", "θ̂ pitch", PITCH, 1.8)
        curve(p, "xhat_y", "θ̂ yaw", YAW, 1.8)
        curve(p, "enc_p", "encoder p", PITCH_ENC, 1.1, Qt.PenStyle.DashLine)
        curve(p, "enc_y", "encoder y", YAW_ENC, 1.1, Qt.PenStyle.DashLine)

        p = mk(2, "Gimbal", "deg")
        curve(p, "u_p", "cmd u p", PITCH, 1.1)
        curve(p, "u_y", "cmd u y", YAW, 1.1)
        curve(p, "genc_p", "achieved p", PITCH_ENC, 2.0)
        curve(p, "genc_y", "achieved y", YAW_ENC, 2.0)
        lim = self._max_deflection()
        for y in (lim, -lim):
            p.addItem(pg.InfiniteLine(pos=y, angle=0, pen=pg.mkPen(RED, width=1,
                                                                  style=Qt.PenStyle.DashLine)))
        p.setYRange(-lim - 2, lim + 2)
        p.enableAutoRange(axis="y", enable=False)

        p = mk(3, "Gyro z (roll)", "deg/s")
        curve(p, "gyro_z", "gyro_z", "#b58cff", 1.4)

        p = mk(4, "Kalman K", "")
        curve(p, "K_p", "K pitch", PITCH, 1.6)
        curve(p, "K_y", "K yaw", YAW, 1.6)
        # Accel gate (TELEM flags bit0/1) as shading behind the K traces wherever it REJECTED
        # the accelerometer (gyro-only predict): pitch fills the lower half, yaw the upper.
        self.k_plot = p
        self.gate_vb = pg.ViewBox()
        self.gate_vb.setZValue(-10)
        p.scene().addItem(self.gate_vb)
        self.gate_vb.setXLink(p)
        self.gate_vb.setYRange(0.0, 2.0, padding=0)
        self.gate_vb.setMouseEnabled(False, False)
        self.gate_curves = {}
        for key, color, base in (("gate_p", PITCH, 0.0), ("gate_y", YAW, 1.0)):
            qc = QColor(color)
            c = pg.PlotCurveItem(pen=pg.mkPen(None), fillLevel=base,
                                 brush=pg.mkBrush(qc.red(), qc.green(), qc.blue(), 70))
            self.gate_vb.addItem(c)
            self.gate_curves[key] = (c, base)
        p.vb.sigResized.connect(lambda: self.gate_vb.setGeometry(p.vb.sceneBoundingRect()))
        for name, color in (("gate rejected p", PITCH), ("gate rejected y", YAW)):
            qc = QColor(color)
            p.addItem(pg.PlotDataItem(name=name, pen=pg.mkPen(qc.red(), qc.green(), qc.blue(),
                                                             110, width=8)))
        p.setLimits(yMin=-0.05)
        for q in self.plots[:-1]:
            q.getAxis("bottom").setStyle(showValues=False)
        self.plots[-1].setLabel("bottom", "link time", units="s", **{"color": DIM})
        self.t0_lines = []
        for q in self.plots:
            ln = pg.InfiniteLine(angle=90, pen=pg.mkPen("#e9edf2", width=1, style=Qt.PenStyle.DotLine),
                                 label="T-0", labelOpts={"position": 0.92, "color": DIM})
            ln.hide()
            q.addItem(ln)
            self.t0_lines.append(ln)
        return w

    def _max_deflection(self):
        try:
            import tvc_params
            return float(tvc_params.control.max_deflection_deg)
        except (ImportError, AttributeError):
            return 10.0

    def _build_telemetry(self):
        p = QFrame()
        p.setObjectName("panel")
        p.setMinimumWidth(300)
        p.setMaximumWidth(400)
        v = QVBoxLayout(p)
        v.setContentsMargins(8, 8, 8, 8)
        tree = QTreeWidget()
        tree.setColumnCount(2)
        tree.setHeaderHidden(True)
        tree.setRootIsDecorated(False)
        tree.setIndentation(8)
        tree.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        tree.setColumnWidth(0, 132)
        tree.setToolTipDuration(8000)
        self.rows = {}
        groups = {
            "Vehicle": ["state", "mode", "profile", "t_ms", "thrust", "peak_thrust", "enc",
                        "arm_switch", "continuity", "battery", "dry_run", "tvc_active"],
            "Estimator": ["xhat", "K", "gate", "u", "saturated", "sat_pct", "gyro_z", "iae_boot",
                          "iae_t0", "peak_tilt"],
            "Comms": ["link", "rssi", "uplink_rssi", "loss", "rx", "crc", "ack", "unacked",
                      "hash_vehicle", "hash_local", "fw"],
            "Storage & Health": ["sd", "flash", "boot", "overruns", "faults", "missing"],
        }
        labels = {
            "state": "State", "mode": "Mode", "profile": "Profile", "t_ms": "Mission t",
            "thrust": "Thrust", "peak_thrust": "Peak thrust", "enc": "Encoder p / y",
            "arm_switch": "Arm switch", "continuity": "Continuity", "battery": "Battery",
            "dry_run": "Dry run", "tvc_active": "TVC active", "xhat": "θ̂ p / y", "K": "K p / y",
            "gate": "Accel gate p / y", "u": "u cmd p / y", "saturated": "Saturated p / y",
            "sat_pct": "Saturation % p / y", "gyro_z": "Gyro z",
            "iae_boot": "IAE boot °·s", "iae_t0": "IAE T-0 °·s (gnd)", "peak_tilt": "Peak |θ̂| p / y",
            "link": "Link", "rssi": "RSSI / SNR", "uplink_rssi": "Uplink RSSI", "loss": "TELEM loss",
            "rx": "Frames rx", "crc": "CRC errors", "ack": "ACK latency", "unacked": "Unacked",
            "hash_vehicle": "Params (vehicle)", "hash_local": "Params (GUI)", "fw": "FW git",
            "sd": "SD written", "flash": "Flash written", "boot": "Boot count",
            "overruns": "Loop overruns", "faults": "Fault bits", "missing": "MEASURE missing",
        }
        mono = QFont("Menlo")
        mono.setPointSize(11)
        for gname, keys in groups.items():
            g = QTreeWidgetItem([gname.upper(), ""])
            g.setForeground(0, QColor(DIM))
            f = g.font(0)
            f.setBold(True)
            f.setPointSize(10)
            g.setFont(0, f)
            g.setFirstColumnSpanned(True)
            tree.addTopLevelItem(g)
            for k in keys:
                it = QTreeWidgetItem([labels[k], "--"])
                it.setForeground(0, QColor(DIM))
                it.setFont(1, mono)
                g.addChild(it)
                self.rows[k] = it
            g.setExpanded(True)
        v.addWidget(tree)
        self.tree = tree
        return p

    # ---- commands ----------------------------------------------------------------------------

    def send(self, ptype, value=None):
        if self.replay:
            self.statusBar().showMessage(f"replay: {tc.packet_name(ptype)} not sent", 3000)
            return
        cmd = self.link.send(ptype, value)
        if cmd is None:
            self.model.add_log(self.link.now(), "error",
                               f"{tc.packet_name(ptype)} NOT SENT: link not connected")
            return
        self.model.sent(cmd)
        self._sent_value[ptype] = value

    def heartbeat(self):
        cmd = self.link.send(tc.PacketType.HEARTBEAT)
        if cmd:
            self.model.sent(cmd)

    def cmd_abort(self):
        self.send(tc.PacketType.ABORT)

    def cmd_dry_run(self):
        # the button's checked state is repainted from TELEM; the click just asks for the flip
        current = bool(self.model.flag(tc.FLAG_DRY_RUN))
        self.send(tc.PacketType.DRY_RUN, 0 if current else 1)

    def cmd_fire_confirm(self):
        # re-check the rule itself, not just the widget: the hold took 500 ms of wall time
        ctl = self.model.controls(self.link.now())["confirm_fire"]
        if ctl.enabled and ctl.visible and self.model.fire_token is not None:
            self.send(tc.PacketType.FIRE_CONFIRM, self.model.fire_token)

    # ---- refresh -----------------------------------------------------------------------------

    def tick(self):
        m, link = self.model, self.link
        while True:
            try:
                m.ingest(link.queue.get_nowait())
            except queue.Empty:
                break
        now = link.now()
        m.check_timeouts(now)
        m.trim(now)
        if m.safe_entered:
            m.safe_entered = False
            if self.recorder:
                path = self.recorder.finalize(m.mode_name(), m.profile_name())
                m.add_log(now, "ok", f"ground log finalised: {path.name}")
                self.statusBar().showMessage(f"ground log: {path}")
        self._refresh_header(now)
        self._refresh_controls(now)
        self._refresh_plots(now)
        self._refresh_telemetry(now)
        self._refresh_log()

    def _refresh_header(self, now):
        m, link = self.model, self.link
        ls = m.link_state(now)
        if self.replay:
            color = BLUE
            txt = f"REPLAY  {link.now():5.1f} / {link.duration:.1f} s" + \
                  ("  (done)" if link.finished else "")
        else:
            color = {"ok": GREEN, "stale": AMBER, "lost": RED, "waiting": FAINT}[ls]
            txt = {"waiting": "waiting for vehicle", "lost": "LINK LOST"}.get(ls, link.kind.upper())
        parts = [txt]
        if m.rssi is not None:
            parts.append(f"RSSI {m.rssi} dBm · SNR {m.snr}")
        loss = m.loss_pct(now)
        if not self.replay and loss is not None:
            parts.append(f"loss {loss:.1f}%")
        if m.ack_latency_ms is not None and not self.replay:
            parts.append(f"ack {m.ack_latency_ms:.0f} ms")
        self.conn_text.setText("  ·  ".join(parts))
        css(self.conn_dot, f"color: {color}; font-size: 14px;")
        css(self.conn_pill, f"QFrame {{ background: {PANEL2}; border: 1px solid {color}; "
                                     "border-radius: 14px; } QLabel { border: none; }")
        if m.rssi_hist:
            self.spark_curve.setData(np.fromiter(m.rssi_hist, float))
        self.spark.setVisible(bool(m.rssi_hist))

        age = m.link_age(now)
        if ls == "lost" and not self.replay:
            self.banner.setText(f"CONNECTION LOST: no telemetry for {age:.1f} s  ({link.description})")
            self.banner.show()
        elif ls == "waiting" and not self.replay and now > 5.0:
            self.banner.setText(f"NO TELEMETRY YET from {link.description}")
            self.banner.show()
        else:
            self.banner.hide()

        st = m.state_name()
        bg, fg = STATE_COLORS.get(st, ("#374151", TEXT))
        self.state_badge.setText(st)
        self.state_badge.set_colors(bg, fg)
        mode = m.mode_name()
        prof = f" · {m.profile_name()}" if mode == "STAND_OPEN" and m.profile is not None else ""
        dry = "  · DRY RUN" if m.flag(tc.FLAG_DRY_RUN) else ""
        self.mode_badge.setText(f"{mode}{prof}{dry}")
        self.mode_badge.set_colors("#1c2a3d", "#cfe3ff", "#2d4461")

        clk = m.mission_clock_ms(now)
        if clk is None:
            self.clock.setText("T  --.-")
            css(self.clock, CLOCK_CSS.format(FAINT))
        else:
            sign = "-" if clk < 0 else "+"
            self.clock.setText(f"T{sign}{abs(clk) / 1000:.1f}")
            c = AMBER if clk < 0 else (RED if st in ("IGNITE", "BURN") else TEXT)
            css(self.clock, CLOCK_CSS.format(c))

        s = m.status
        if s:
            match = s["params_hash_match"]
            col, mark = {True: (GREEN, "✓ match"), False: (RED, "✗ MISMATCH"),
                         None: (FAINT, "? no local hash")}[match]
            self.hash_label.setText(f"params {s['params_hash6_hex']}  {mark}")
            css(self.hash_label, f"font-family: Menlo; font-size: 11px; color: {col};")
            self.fw_label.setText(f"fw {s['fw_git_hash_hex']}")

    def _refresh_controls(self, now):
        m = self.model
        ctl = m.controls(now)
        for key, b in self.btn.items():
            c = ctl.get(key)
            if c is None:
                continue
            b.setEnabled(c.enabled)
            b.setVisible(c.visible)
            b.setToolTip(c.reason)
            if c.label:
                b.setText(c.label)
        # pills reflect what the vehicle reports, not what was clicked
        for i, name in enumerate(tc.MODE_NAMES):
            b = self.btn[f"mode:{name}"]
            b.setChecked(m.mode == i)
            restyle(b, pending=m.awaiting_ack(tc.PacketType.SET_MODE)
                    and self._sent_value.get(tc.PacketType.SET_MODE) == i)
        for i, name in enumerate(tc.PROFILE_NAMES):
            b = self.btn[f"profile:{name}"]
            b.setChecked(m.profile == i)
            restyle(b, pending=m.awaiting_ack(tc.PacketType.SET_PROFILE)
                    and self._sent_value.get(tc.PacketType.SET_PROFILE) == i)
        self.profile_box.setVisible(ctl["profile:HOLD"].visible)
        self.btn["dry_run"].setChecked(bool(m.flag(tc.FLAG_DRY_RUN)))
        for key, ptype in (("dry_run", tc.PacketType.DRY_RUN), ("tare", tc.PacketType.TARE),
                           ("ping", tc.PacketType.PING), ("arm", tc.PacketType.ARM),
                           ("countdown", tc.PacketType.COUNTDOWN),
                           ("disarm", tc.PacketType.DISARM), ("abort", tc.PacketType.ABORT)):
            res = m.last_result.get(ptype, (None,))[0]
            restyle(self.btn[key], pending=m.awaiting_ack(ptype),
                    result=res if res in ("rejected", "timeout") else "")
        self.reason["mode"].setText(ctl["mode:STAND_OPEN"].reason)  # shared by all mode pills
        for key in ("arm", "countdown", "confirm_fire"):
            c = ctl[key]
            self.reason[key].setText("" if c.enabled or not c.visible else c.reason)
            self.reason[key].setVisible(c.visible and not c.enabled and bool(c.reason))

        st = m.state_name()
        clk = m.mission_clock_ms(now)
        if st == "COUNTDOWN" and clk is not None:
            self.countdown_label.setText(f"T{'-' if clk < 0 else '+'}{abs(clk) / 1000:.1f}")
            self.countdown_label.show()
        else:
            self.countdown_label.hide()
        if st != "COUNTDOWN":
            self.confirm.reset()

        t = m.telem
        if t:
            ok = m.continuity_ok()
            self.ro["cont"].setText(f"{'OK' if ok else 'OPEN'}  {t['cont_adc8']}")
            css(self.ro["cont"], f"font-family: Menlo; color: {GREEN if ok else RED};")
            mv = t["batt_mv"]
            self.ro["batt"].setText(f"{mv / 1000:.2f} V" if mv else "uncalibrated")
            css(self.ro["batt"], f"font-family: Menlo; color: {TEXT if mv else FAINT};")
            closed = m.flag(tc.FLAG_ARM_SWITCH_CLOSED)
            self.ro["armsw"].setText("CLOSED" if closed else "OPEN")
            css(self.ro["armsw"], f"font-family: Menlo; color: {AMBER if closed else DIM};")

    def _refresh_plots(self, now):
        m = self.model
        x0, x1 = now - 30.0, now + 0.5
        self.plots[0].setXRange(x0, x1, padding=0)
        s = m.series

        def arr(key):
            return np.fromiter(s[key].t, float), np.fromiter(s[key].y, float)

        for key in ("thrust", "xhat_p", "xhat_y", "enc_p", "enc_y", "u_p", "u_y", "gyro_z",
                    "K_p", "K_y"):
            self.curves[key].setData(*arr(key))
        self.curves["genc_p"].setData(*arr("enc_p"))
        self.curves["genc_y"].setData(*arr("enc_y"))
        self.gate_vb.setGeometry(self.k_plot.vb.sceneBoundingRect())
        for key, (c, base) in self.gate_curves.items():
            t, ok = arr(key)
            c.setData(*_steps(t, base + (1.0 - ok)))  # fill up to base+1 where rejected
        if m.t0_link is not None:
            for ln in self.t0_lines:
                ln.setValue(m.t0_link)
                ln.show()
            if self.nominal_t is not None:
                self.nominal_curve.setData(self.nominal_t + m.t0_link, self.nominal_f)
        else:
            for ln in self.t0_lines:
                ln.hide()
            self.nominal_curve.setData([], [])

    def _set(self, key, text, color=None):
        it = self.rows[key]
        if it.text(1) != text:
            it.setText(1, text)
        it.setForeground(1, QColor(color or TEXT))

    def _refresh_telemetry(self, now):
        m = self.model
        t, s = m.telem, m.status
        st = m.state_name()
        self._set("state", st, STATE_COLORS.get(st, (TEXT,))[0] if st != "IDLE" else TEXT)
        self._set("mode", m.mode_name())
        self._set("profile", m.profile_name())
        clk = m.mission_clock_ms(now)
        self._set("t_ms", "not scheduled" if clk is None else f"{clk / 1000:+.2f} s")
        if t:
            f = t["flags"]
            self._set("thrust", f"{t['thrust_N']:.1f} N")
            self._set("enc", f"{t['enc_p_deg']:+.2f} / {t['enc_y_deg']:+.2f}°")
            closed = bool(f & tc.FLAG_ARM_SWITCH_CLOSED)
            self._set("arm_switch", "CLOSED" if closed else "OPEN", AMBER if closed else DIM)
            ok = m.continuity_ok()
            self._set("continuity", f"{'OK' if ok else 'OPEN'} (adc8 {t['cont_adc8']})",
                      GREEN if ok else RED)
            self._set("battery", f"{t['batt_mv'] / 1000:.2f} V" if t["batt_mv"] else "uncalibrated",
                      None if t["batt_mv"] else FAINT)
            self._set("dry_run", "ON" if f & tc.FLAG_DRY_RUN else "off",
                      AMBER if f & tc.FLAG_DRY_RUN else None)
            self._set("tvc_active", "YES" if f & tc.FLAG_TVC_ACTIVE else "no",
                      RED if f & tc.FLAG_TVC_ACTIVE else None)
            self._set("xhat", f"{t['xhat_p_deg']:+.2f} / {t['xhat_y_deg']:+.2f}°")
            self._set("K", f"{t['K_p']:.3f} / {t['K_y']:.3f}")
            gp, gy = f & tc.FLAG_PITCH_GATE_OK, f & tc.FLAG_YAW_GATE_OK
            self._set("gate", f"{'ok' if gp else 'REJECT'} / {'ok' if gy else 'REJECT'}",
                      None if gp and gy else AMBER)
            self._set("u", f"{t['u_p_deg']:+.2f} / {t['u_y_deg']:+.2f}°")
            sp, sy = f & tc.FLAG_PITCH_SATURATED, f & tc.FLAG_YAW_SATURATED
            self._set("saturated", f"{'SAT' if sp else '-'} / {'SAT' if sy else '-'}",
                      AMBER if sp or sy else None)
            self._set("gyro_z", f"{t['gyro_z_deg_s']:+.1f} °/s")
        self._set("iae_t0", f"{m.iae_since_t0_p:.3f} / {m.iae_since_t0_y:.3f}")
        if s:
            self._set("peak_thrust", f"{s['peak_thrust_N']:.1f} N")
            self._set("sat_pct", f"{s['sat_pct_p']} / {s['sat_pct_y']} %")
            self._set("iae_boot", f"{s['running_iae_p_degs']:.2f} / {s['running_iae_y_degs']:.2f}")
            self._set("peak_tilt", f"{s['peak_tilt_p_deg']:.2f} / {s['peak_tilt_y_deg']:.2f}°")
            u = s["rssi_last_uplink_dbm"]
            self._set("uplink_rssi", "none yet" if u == -128 else f"{u} dBm", FAINT if u == -128 else None)
            match = s["params_hash_match"]
            self._set("hash_vehicle", s["params_hash6_hex"],
                      {True: GREEN, False: RED, None: None}[match])
            self._set("fw", s["fw_git_hash_hex"])
            self._set("sd", f"{s['sd_bytes_written'] / 1e6:.3f} MB")
            self._set("flash", f"{s['flash_bytes_written'] / 1e3:.1f} kB")
            self._set("boot", str(s["boot_count"]))
            self._set("overruns", str(s["loop_overrun_count"]),
                      AMBER if s["loop_overrun_count"] else None)
            faults = s["fault_names"]
            self._set("faults", ", ".join(faults) if faults else "none", RED if faults else GREEN)
            missing = s["missing_measure_names"]
            it = self.rows["missing"]
            self._set("missing", f"{len(missing)} params" if missing else "none",
                      AMBER if missing else GREEN)
            it.setToolTip(1, "\n".join(missing))
        self._set("hash_local", m.tables.local_hash_hex[:12] or "none")
        self._set("link", self.link.kind)
        self.rows["link"].setToolTip(1, self.link.description)
        self._set("rssi", "n/a (no radio)" if m.rssi is None else f"{m.rssi} dBm / {m.snr} dB")
        loss = m.loss_pct(now)
        self._set("loss", "n/a (replay)" if self.replay else fmt(loss, ".1f", " %"),
                  AMBER if (loss or 0) > 2 and not self.replay else None)
        rx = m.rx_counts
        self._set("rx", " ".join(f"{k[0]}{rx.get(k, 0)}" for k in ("TELEM", "STATUS", "EVENT", "ACK")))
        parser = getattr(self.link, "parser", None)
        crc = parser.crc_errors if parser else 0
        self._set("crc", str(crc), AMBER if crc else None)
        self._set("ack", "n/a" if self.replay else fmt(m.ack_latency_ms, ".0f", " ms"))
        unacked = [tc.packet_name(c.ptype) for c in m.pending.values()
                   if c.ptype != tc.PacketType.HEARTBEAT]
        self._set("unacked", ", ".join(unacked) if unacked else "none", AMBER if unacked else None)

    def _refresh_log(self):
        log = self.model.log
        if self._log_rendered == len(log):
            return
        for e in log[self._log_rendered:]:
            it = QListWidgetItem(f"{e.wall}  {e.t:7.2f}s  {e.text}")
            it.setForeground(QColor(SEVERITY_COLORS.get(e.severity, TEXT)))
            self.log_list.insertItem(0, it)  # newest on top
            if self.recorder:
                self.recorder.event_line(f"[{e.severity}] {e.text}")
        self._log_rendered = len(log)
        while self.log_list.count() > 1000:
            self.log_list.takeItem(self.log_list.count() - 1)

    def closeEvent(self, ev):
        self.redraw_timer.stop()
        self.hb_timer.stop()
        self.link.stop()
        if self.recorder:
            self.recorder.close()
        super().closeEvent(ev)


# ---- entry point -------------------------------------------------------------------------------

def _port_open(host, port) -> bool:
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex((host, port)) == 0


def start_sim_if_needed(port: int, extra_args=()) -> subprocess.Popen | None:
    if _port_open("127.0.0.1", port):
        return None
    proc = subprocess.Popen([sys.executable, str(REPO_ROOT / "sim" / "run_sim.py"), "--serve",
                             "--port", str(port), *extra_args],
                            stdin=subprocess.DEVNULL)
    for _ in range(50):
        if _port_open("127.0.0.1", port) or proc.poll() is not None:
            break
        time.sleep(0.1)
    return proc


def build(args) -> tuple[Link, Model, GroundRecorder | None, subprocess.Popen | None]:
    tables = tc.Tables.load(REPO_ROOT)
    try:
        import tvc_params
        telem_hz = float(tvc_params.telemetry.telem_rate_hz)
    except (ImportError, AttributeError):
        telem_hz = 10.0
    sim_proc = None
    if args.replay:
        link = ReplayLink(tables, Path(args.replay), speed=args.speed,
                          telem_hz=args.replay_telem_hz or telem_hz)
    elif args.sim:
        if not args.no_spawn:
            sim_proc = start_sim_if_needed(args.sim_port, args.sim_args)
        link = TcpLink(tables, port=args.sim_port)
    else:
        port = args.port
        if not port:
            found = find_feather_ports()
            if len(found) != 1:
                sys.exit("gs.py: give --port (Feather bridge), --sim, or --replay FILE"
                         + (f"; Adafruit ports seen: {', '.join(found)}" if found else ""))
            port = found[0]
        link = SerialLink(tables, port, args.baud)
    model = Model(tables=tables, telem_hz=telem_hz, can_send=link.can_send)
    if isinstance(link, ReplayLink) and link.meta.get("source") == "sd":
        model.mode = link.meta["mode"]
        model.profile = link.meta["profile"]
    recorder = None
    if not args.replay and not args.no_record:
        recorder = GroundRecorder(Path(args.log_dir), link.description, tables)
        link.recorder = recorder
    return link, model, recorder, sim_proc


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--port", help="Feather M0 bridge USB serial port (live)")
    src.add_argument("--sim", action="store_true", help="connect to sim/run_sim.py --serve")
    src.add_argument("--replay", metavar="FILE", help="SD log (SF_*.BIN) or ground log (.bin)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim-port", type=int, default=5760)
    ap.add_argument("--no-spawn", action="store_true", help="--sim: don't start the sim")
    ap.add_argument("--sim-args", nargs=argparse.REMAINDER, default=[],
                    help="--sim: remaining args go to run_sim.py --serve (e.g. --mode STAND_CLOSED)")
    ap.add_argument("--speed", type=float, default=1.0, help="--replay: x real time")
    ap.add_argument("--replay-telem-hz", type=float, default=None,
                    help="--replay of an SD log: TELEM rate to re-derive (default: the "
                         "vehicle's own telemetry rate, i.e. what the radio would have carried)")
    ap.add_argument("--log-dir", default=str(REPO_ROOT / "data" / "ground"))
    ap.add_argument("--no-record", action="store_true")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    link, model, recorder, sim_proc = build(args)
    win = MainWindow(link, model, recorder)
    link.start()
    win.show()
    try:
        rc = app.exec()
    finally:
        link.stop()
        if sim_proc:
            sim_proc.terminate()
    return rc


if __name__ == "__main__":
    sys.exit(main())
