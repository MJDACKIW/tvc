"""core/ via ctypes: the simulation's controller is the firmware's controller (SPEC.md Sections
1, 3.4, 6). libtvccore is built by `make -C core native`; no pure-Python fallback exists here
by design (SPEC 6 allows one only inside tools/parity_test.py)."""
from __future__ import annotations

import ctypes
import platform
import subprocess
from dataclasses import dataclass
from pathlib import Path

CORE_DIR = Path(__file__).resolve().parent.parent / "core"
_LIB = None


def _lib():
    global _LIB
    if _LIB is None:
        name = "libtvccore.dylib" if platform.system() == "Darwin" else "libtvccore.so"
        subprocess.run(["make", "-C", str(CORE_DIR), "native"], check=True,
                       stdout=subprocess.DEVNULL)
        lib = ctypes.CDLL(str(CORE_DIR / name))
        f, fp, ip = ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)
        lib.tvc_controller_step.argtypes = [fp, fp, fp, fp, fp, ip, f, f, ctypes.c_int,
                                            f, f, f, f, f, f, f, f, f, fp, fp, fp, fp, fp, ip]
        lib.tvc_controller_step.restype = None
        _LIB = lib
    return _LIB


@dataclass
class ControlParams:
    dt: float
    kp: float
    ki: float
    kd: float
    integral_clamp: float
    max_deflection: float
    q: float
    r: float
    slew_deg_per_s: float


@dataclass
class AxisOut:
    x_hat: float
    u_raw: float
    u_cmd: float
    delta: float
    K: float
    accel_used: bool


class Axis:
    """One axis' AxisState, held as ctypes floats so each step is a single FFI call."""

    def __init__(self, p0: float):
        self.lib = _lib()
        self.x_hat, self.P = ctypes.c_float(0.0), ctypes.c_float(p0)
        self.integral, self.e_prev = ctypes.c_float(0.0), ctypes.c_float(0.0)
        self.delta, self.saturated = ctypes.c_float(0.0), ctypes.c_int(0)
        self._out = [ctypes.c_float() for _ in range(5)]
        self._used = ctypes.c_int()

    def step(self, gyro_deg_s: float, accel_tilt_deg: float, gate_ok: bool,
             p: ControlParams) -> AxisOut:
        o = self._out
        self.lib.tvc_controller_step(
            ctypes.byref(self.x_hat), ctypes.byref(self.P), ctypes.byref(self.integral),
            ctypes.byref(self.e_prev), ctypes.byref(self.delta), ctypes.byref(self.saturated),
            gyro_deg_s, accel_tilt_deg, int(gate_ok), p.dt, p.kp, p.ki, p.kd, p.integral_clamp,
            p.max_deflection, p.q, p.r, p.slew_deg_per_s, *(ctypes.byref(x) for x in o),
            ctypes.byref(self._used))
        return AxisOut(o[0].value, o[1].value, o[2].value, o[3].value, o[4].value,
                       bool(self._used.value))
