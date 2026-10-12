"""Phase 5 acceptance (SPEC.md Section 9): the real gs.py window, offscreen, driven through
  - --replay of an SD log, start to finish, and
  - --sim against sim/run_sim.py's virtual vehicle (in-process here, so the test can also flip
    the arm switch and cut the heartbeat), checking at every state which commands the GUI
    enables, then replaying the ground log the recorder wrote during that session.

Runs in real time (~90 s). TVC_GUI_SHOTS=<dir> also saves a screenshot at each checkpoint.

    ground/venv/bin/python -m pytest ground/tests/test_gui.py -s
"""
import os
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.*=false")
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "ground"))

import gs  # noqa: E402
import telem_codec as tc  # noqa: E402
from PyQt6.QtCore import QPoint, Qt  # noqa: E402
from PyQt6.QtTest import QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

SHOTS = os.environ.get("TVC_GUI_SHOTS")
SYNTH_LOG = REPO_ROOT / "data" / "synthetic" / "SF_SYNTH.BIN"


@pytest.fixture(scope="module")
def app():
    a = QApplication.instance() or QApplication([])
    a.setStyle("Fusion")
    a.setStyleSheet(gs.STYLE)
    return a


def pump(app, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        app.processEvents()
        time.sleep(0.005)


LAST_MODEL = []


def wait_until(app, pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        app.processEvents()
        if pred():
            return
        time.sleep(0.005)
    tail = ""
    if LAST_MODEL:
        m = LAST_MODEL[-1]
        tail = f" (state {m.state_name()}); last events:\n" + "\n".join(
            f"  {e.t:7.2f} {e.severity:5s} {e.text}" for e in m.log[-10:])
    raise AssertionError(f"timed out after {timeout:.0f} s waiting for {what}{tail}")


def shot(win, name):
    if SHOTS:
        Path(SHOTS).mkdir(parents=True, exist_ok=True)
        win.grab().save(str(Path(SHOTS) / f"{name}.png"))


def enabled(win):
    """What the user can actually click right now: visible and enabled."""
    return {k for k, b in win.btn.items() if b.isVisible() and b.isEnabled()}


def hold(app, button, ms):
    QTest.mousePress(button, Qt.MouseButton.LeftButton, pos=QPoint(10, 10))
    pump(app, ms / 1000.0)
    QTest.mouseRelease(button, Qt.MouseButton.LeftButton, pos=QPoint(10, 10))


# ---- --replay -----------------------------------------------------------------------------------

def test_replay_sd_log_end_to_end(app):
    if not SYNTH_LOG.exists():
        pytest.skip("run tools/synth_sd_log.py first")
    link, model, rec, _ = gs.build(gs.parse_args(["--replay", str(SYNTH_LOG), "--speed", "4"]))
    win = gs.MainWindow(link, model, rec)
    win.resize(1600, 980)
    win.show()
    link.start()
    seen = []
    try:
        def track():
            if model.state is not None and model.state_name() not in seen[-1:]:
                seen.append(model.state_name())
            return link.finished
        wait_until(app, track, 30, "replay to finish")
        pump(app, 0.3)
        shot(win, "replay_end")
        assert seen == ["IDLE", "ARMED", "COUNTDOWN", "IGNITE", "BURN", "TAILOFF", "SAFE"], seen
        # the plots got the whole run, and the header/telemetry show the log's identity
        assert len(model.series["thrust"].t) > 150
        assert max(model.series["thrust"].y) > 25
        assert max(abs(v) for v in model.series["enc_p"].y) > 4  # STEP_X moved the gimbal
        assert model.status["params_hash_match"] is True
        assert win.state_badge.text() == "SAFE"
        assert model.mode_name() == "STAND_OPEN" and model.profile_name() == "STEP_X"
        assert model.t0_link is not None and win.t0_lines[0].isVisible()
        log = [e.text for e in model.log]
        assert any("STATE_BURN" in t for t in log) and any("inferred" in t for t in log)
        # replay sends nothing, whatever gets clicked
        win.btn["abort"].click()
        assert not model.pending
    finally:
        win.close()


# ---- --sim --------------------------------------------------------------------------------------

def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_sim_live_run_and_state_dependent_buttons(app, tmp_path):
    sys.path.insert(0, str(REPO_ROOT / "sim"))
    import serve

    port = _free_port()
    vehicle = serve.VirtualVehicle(mode="STAND_OPEN", seed=3)
    stop = threading.Event()
    sim = threading.Thread(target=serve.run, args=(vehicle, "127.0.0.1", port, stop), daemon=True)
    sim.start()

    link, model, rec, _ = gs.build(gs.parse_args(
        ["--sim", "--no-spawn", "--sim-port", str(port), "--log-dir", str(tmp_path)]))
    win = gs.MainWindow(link, model, rec)
    win.resize(1600, 980)
    win.show()
    link.start()
    LAST_MODEL.append(model)
    st = model.state_name
    modes = {f"mode:{m}" for m in tc.MODE_NAMES}
    profiles = {f"profile:{p}" for p in tc.PROFILE_NAMES}
    try:
        # ---- IDLE: setup commands and ARM, nothing that belongs to a later state -------------
        wait_until(app, lambda: st() == "IDLE", 10, "IDLE after SELFTEST")
        pump(app, 1.2)  # a STATUS (1 Hz) for mode/hash
        en = enabled(win)
        assert modes <= en and profiles <= en, en
        assert {"dry_run", "tare", "ping", "arm", "abort"} <= en
        assert not en & {"countdown", "confirm_fire", "disarm"}, en
        assert not win.btn["confirm_fire"].isVisible()
        assert win.hash_label.text().endswith("✓ match")
        shot(win, "sim_1_idle")

        # arm switch open -> ARM disabled, with the reason shown
        vehicle.arm_switch_closed = False
        wait_until(app, lambda: "arm" not in enabled(win), 3, "ARM to disable on open switch")
        assert "arm switch is OPEN" in win.reason["arm"].text()
        vehicle.arm_switch_closed = True
        wait_until(app, lambda: "arm" in enabled(win), 3, "ARM to re-enable")

        # profile pills only exist for STAND_OPEN; the pill follows the vehicle's ACK/event
        win.btn["profile:STEP_X"].click()
        wait_until(app, lambda: model.profile_name() == "STEP_X", 3, "SET_PROFILE ACK")
        wait_until(app, lambda: win.btn["profile:STEP_X"].isChecked(), 1, "STEP_X pill checked")

        # ---- ARMED -------------------------------------------------------------------------
        win.btn["arm"].click()
        wait_until(app, lambda: st() == "ARMED", 3, "ARMED")
        pump(app, 0.2)
        en = enabled(win)
        assert {"countdown", "disarm", "abort", "ping"} <= en, en
        assert not en & (modes | profiles | {"arm", "tare", "dry_run", "confirm_fire"}), en
        shot(win, "sim_2_armed")

        # heartbeat loss while ARMED -> vehicle drops to SAFE on its own (SPEC 4.3)
        win.hb_timer.stop()
        wait_until(app, lambda: st() == "SAFE", 4, "SAFE on heartbeat loss")
        assert any("HEARTBEAT_LOST" in e.text for e in model.log)
        win.hb_timer.start(500)
        t_resume = link.now()
        wait_until(app, lambda: model.t_last_rx and not any(
            c.ptype == tc.PacketType.HEARTBEAT and c.t > t_resume for c in model.pending.values())
            and link.now() - t_resume > 0.6, 3, "heartbeats to resume")
        en = enabled(win)
        assert "disarm" in en and win.btn["disarm"].text() == "RESET → IDLE"
        assert not en & {"arm", "countdown"}
        win.btn["disarm"].click()
        wait_until(app, lambda: st() == "IDLE", 3, "IDLE after RESET")

        # ---- full STAND_OPEN / STEP_X run ----------------------------------------------------
        win.btn["arm"].click()
        wait_until(app, lambda: st() == "ARMED", 3, "ARMED")
        win.btn["countdown"].click()
        wait_until(app, lambda: st() == "COUNTDOWN", 3, "COUNTDOWN")
        pump(app, 0.3)
        en = enabled(win)
        assert en & {"countdown", "arm", "disarm"} == set(), en
        assert "abort" in en and not win.btn["confirm_fire"].isVisible()  # not in last 3 s yet
        assert model.fire_token is not None  # from COUNTDOWN's ACK
        shot(win, "sim_3_countdown")
        wait_until(app, lambda: win.btn["confirm_fire"].isVisible(), 9, "CONFIRM FIRE to appear")
        clk = model.mission_clock_ms(link.now())
        assert -3100 <= clk <= -2500, clk
        assert win.btn["confirm_fire"].isEnabled()
        # a short press must not fire
        hold(app, win.btn["confirm_fire"], 200)
        pump(app, 0.2)
        assert tc.PacketType.FIRE_CONFIRM not in [c.ptype for c in model.pending.values()]
        assert not model.fire_confirmed
        shot(win, "sim_4_confirm_window")
        hold(app, win.btn["confirm_fire"], 650)
        wait_until(app, lambda: model.fire_confirmed, 2, "FIRE_CONFIRM ACK")
        wait_until(app, lambda: st() in ("IGNITE", "BURN"), 5, "ignition at T-0")
        wait_until(app, lambda: st() == "BURN", 2, "BURN")
        pump(app, 1.2)
        shot(win, "sim_5_burn")
        en = enabled(win)
        assert en == {"abort", "ping"}, en
        assert model.flag(tc.FLAG_TVC_ACTIVE)
        wait_until(app, lambda: st() == "SAFE", 8, "SAFE after burn")
        pump(app, 0.5)
        shot(win, "sim_6_safe")
        assert max(model.series["thrust"].y) > 25
        assert max(abs(v) for v in model.series["enc_p"].y) > 4
        assert rec.finalized and rec.finalized[-1].name.endswith("_STAND_OPEN_STEP_X.bin")
        ground_log = rec.finalized[-1]

        # ---- STAND_CLOSED: closed loop recovers the 5 deg tip-off ----------------------------
        win.btn["disarm"].click()
        wait_until(app, lambda: st() == "IDLE", 3, "IDLE")
        win.btn["mode:STAND_CLOSED"].click()
        wait_until(app, lambda: model.mode_name() == "STAND_CLOSED", 3, "SET_MODE ACK")
        pump(app, 0.3)
        assert not win.profile_box.isVisible()  # profile pills are STAND_OPEN only
        win.btn["arm"].click()
        wait_until(app, lambda: st() == "ARMED", 3, "ARMED")
        win.btn["countdown"].click()
        wait_until(app, lambda: win.btn["confirm_fire"].isVisible(), 12, "confirm window")
        hold(app, win.btn["confirm_fire"], 650)
        wait_until(app, lambda: st() == "BURN", 6, "BURN")
        pump(app, 1.5)
        shot(win, "sim_7_closed_burn")
        wait_until(app, lambda: st() == "SAFE", 8, "SAFE")
        xp = [v for t, v in zip(model.series["xhat_p"].t, model.series["xhat_p"].y)
              if model.t0_link is not None and t > model.t0_link + 1.0]
        assert xp and max(abs(v) for v in xp) < 1.5, "closed loop did not bring pitch back"

        # ABORT is always there, and works from IDLE too (firmware abortInternal)
        win.btn["disarm"].click()
        wait_until(app, lambda: st() == "IDLE", 3, "IDLE")
        assert "abort" in enabled(win)
        QTest.keyClick(win, Qt.Key.Key_Space)  # the spacebar shortcut
        wait_until(app, lambda: st() == "SAFE", 3, "SAFE on spacebar ABORT")
        assert any("ABORT_MANUAL" in e.text for e in model.log)
        assert not [e for e in model.log if e.severity == "error" and "REJECTED" in e.text]
    finally:
        win.close()
        stop.set()
        sim.join(timeout=3)

    # ---- the recorder's ground log replays too --------------------------------------------------
    link2, model2, _, _ = gs.build(gs.parse_args(["--replay", str(ground_log), "--speed", "8"]))
    win2 = gs.MainWindow(link2, model2, None)
    win2.show()
    link2.start()
    try:
        wait_until(app, lambda: link2.finished, 30, "ground log replay")
        pump(app, 0.2)
        assert model2.state_name() == "SAFE"
        assert max(model2.series["thrust"].y) > 25
        assert any(e.text.startswith("uplink FIRE_CONFIRM") for e in model2.log)
        shot(win2, "sim_8_ground_log_replay")
    finally:
        win2.close()
