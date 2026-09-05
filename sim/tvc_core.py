"""ctypes bridge to core/libtvccore (kalman2d + pid + rate_limiter via controller_step).
See SPEC.md Section 6: "The controller is core/ via ctypes... A pure-Python fallback is
not allowed in the main path." This is the one place that FFI contract is encoded; both
sim/run_sim.py and tools/parity_test.py import it, so there is exactly one ctypes call
site to keep in sync with core/ffi.h.
"""
import ctypes
import platform
import subprocess
from pathlib import Path

CORE_DIR = Path(__file__).resolve().parent.parent / "core"

_lib = None


def build():
    """Build core/ as a native shared library via its Makefile."""
    subprocess.run(["make", "-C", str(CORE_DIR), "native"], check=True)


def _lib_path():
    name = "libtvccore.dylib" if platform.system() == "Darwin" else "libtvccore.so"
    return CORE_DIR / name


def _load():
    global _lib
    if _lib is not None:
        return _lib
    lib = ctypes.CDLL(str(_lib_path()))
    c_float_p = ctypes.POINTER(ctypes.c_float)
    c_int_p = ctypes.POINTER(ctypes.c_int)
    lib.tvc_controller_step.argtypes = [
        # AxisState, in/out: x_hat, bias_hat, p00, p01, p10, p11, integral, delta, saturated
        c_float_p, c_float_p, c_float_p, c_float_p, c_float_p, c_float_p,
        c_float_p, c_float_p, c_int_p,
        # Inputs: gyro_deg_s, accel_tilt_deg, accel_gate_ok
        ctypes.c_float, ctypes.c_float, ctypes.c_int,
        # ControlParams: dt, kp, ki, kd, integral_clamp, max_deflection, q_angle, q_rate,
        # r, slew_deg_per_s
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
        ctypes.c_float, ctypes.c_float,
        # AxisOut: x_hat, u_raw, u_cmd, delta, K, accel_used
        c_float_p, c_float_p, c_float_p, c_float_p, c_float_p, c_int_p,
    ]
    lib.tvc_controller_step.restype = None
    _lib = lib
    return _lib


class ControllerAxis:
    """One instance per axis (pitch or yaw). Owns one core/controller.h AxisState and
    calls tvc_controller_step via ctypes on every step() call."""

    def __init__(self, dt, kp, ki, kd, integral_clamp, max_deflection, q_angle, q_rate,
                 r, slew_deg_per_s, p0=0.0):
        self._lib = _load()
        self._dt = ctypes.c_float(dt)
        self._kp = ctypes.c_float(kp)
        self._ki = ctypes.c_float(ki)
        self._kd = ctypes.c_float(kd)
        self._integral_clamp = ctypes.c_float(integral_clamp)
        self._max_deflection = ctypes.c_float(max_deflection)
        self._q_angle = ctypes.c_float(q_angle)
        self._q_rate = ctypes.c_float(q_rate)
        self._r = ctypes.c_float(r)
        self._slew_deg_per_s = ctypes.c_float(slew_deg_per_s)

        # AxisState, persistent across step() calls. core/controller.h's AxisState
        # defaults P's diagonal to 0.0 (an arbitrary struct default, not params.yaml's
        # kalman.p0); seed it from p0 here since that field only has meaning to a caller.
        # P0 = eye(2) * p0, matching the paper's kal_P = np.eye(2) initialization.
        self._x_hat = ctypes.c_float(0.0)
        self._bias_hat = ctypes.c_float(0.0)
        self._p00 = ctypes.c_float(p0)
        self._p01 = ctypes.c_float(0.0)
        self._p10 = ctypes.c_float(0.0)
        self._p11 = ctypes.c_float(p0)
        self._integral = ctypes.c_float(0.0)
        self._delta = ctypes.c_float(0.0)
        self._saturated = ctypes.c_int(0)

    def step(self, gyro_deg_s, accel_tilt_deg, accel_gate_ok):
        """Advance this axis by one control tick. Returns a dict matching AxisOut:
        x_hat, u_raw, u_cmd, delta, K, accel_used."""
        out_x_hat = ctypes.c_float()
        out_u_raw = ctypes.c_float()
        out_u_cmd = ctypes.c_float()
        out_delta = ctypes.c_float()
        out_K = ctypes.c_float()
        out_accel_used = ctypes.c_int()

        self._lib.tvc_controller_step(
            ctypes.byref(self._x_hat), ctypes.byref(self._bias_hat),
            ctypes.byref(self._p00), ctypes.byref(self._p01),
            ctypes.byref(self._p10), ctypes.byref(self._p11),
            ctypes.byref(self._integral), ctypes.byref(self._delta),
            ctypes.byref(self._saturated),
            ctypes.c_float(gyro_deg_s), ctypes.c_float(accel_tilt_deg),
            int(accel_gate_ok),
            self._dt, self._kp, self._ki, self._kd,
            self._integral_clamp, self._max_deflection,
            self._q_angle, self._q_rate, self._r, self._slew_deg_per_s,
            ctypes.byref(out_x_hat), ctypes.byref(out_u_raw),
            ctypes.byref(out_u_cmd), ctypes.byref(out_delta),
            ctypes.byref(out_K), ctypes.byref(out_accel_used),
        )
        return {
            "x_hat": out_x_hat.value,
            "u_raw": out_u_raw.value,
            "u_cmd": out_u_cmd.value,
            "delta": out_delta.value,
            "K": out_K.value,
            "accel_used": bool(out_accel_used.value),
        }
