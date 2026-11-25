"""sim/serve.py (the GUI's --sim backend) runs on the validated model: the same Vehicle /
StandVehicle plant and the same core/ controller that sim/run_sim.py's scenarios use, with no
second copy of either (SPEC.md Section 6). Headless: no sockets, no Qt, faster than real time.

    ground/venv/bin/python -m pytest ground/tests/test_serve_model.py
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "sim"))
sys.path.insert(0, str(REPO_ROOT / "protocol"))

import serve  # noqa: E402
import tvc_core  # noqa: E402
import vehicle  # noqa: E402
from serve import S  # noqa: E402


def lit_vehicle(mode):
    """A virtual vehicle already in IGNITE with the igniter just fired, and its stepper."""
    v = serve.VirtualVehicle(mode=mode, seed=1, theta0=(5.0, -2.0))
    v._enter(S["IDLE"])
    v._enter(S["IGNITE"])
    v.t_ignite = v.t
    return v, serve.Stepper(v)


def advance_to_motor_time(v, stepper, t_motor_s):
    while v._t_motor() < t_motor_s:
        stepper.step()


def test_free_flight_uses_one_vehicle_per_axis_and_the_shared_controller():
    v = serve.VirtualVehicle(mode="SIM_ONBOARD", seed=1)
    assert type(v.plant_p) is vehicle.Vehicle and type(v.plant_y) is vehicle.Vehicle
    # rk4_step advances the instance's own axial velocity: a shared instance would double it
    assert v.plant_p is not v.plant_y
    assert type(v.ax_p) is tvc_core.ControllerAxis and v.ax_p is not v.ax_y
    stand = serve.VirtualVehicle(mode="STAND_CLOSED", seed=1)
    assert type(stand.plant_p) is vehicle.StandVehicle


def test_the_two_vehicle_instances_integrate_the_same_velocity():
    v, stepper = lit_vehicle("SIM_ONBOARD")
    advance_to_motor_time(v, stepper, 1.0)
    assert v.plant_p.v > 5.0  # it has left the pad
    assert v.plant_p.v == v.plant_y.v


def test_free_flight_closes_the_accel_gate_on_specific_force_and_recovers_tilt():
    v, stepper = lit_vehicle("SIM_ONBOARD")
    advance_to_motor_time(v, stepper, 0.3)
    assert not v._accel_gate_ok()  # thrust ~ 5 g: the accelerometer reads thrust, not gravity
    advance_to_motor_time(v, stepper, 1.5)
    assert abs(v.theta[0]) < 1.0 and abs(v.theta[1]) < 1.0  # 5 and -2 deg tip-off recovered


def test_stand_closed_keeps_the_gate_open_and_recovers_tilt():
    v, stepper = lit_vehicle("STAND_CLOSED")
    advance_to_motor_time(v, stepper, 0.3)
    assert v._accel_gate_ok()
    advance_to_motor_time(v, stepper, 1.5)
    assert abs(v.theta[0]) < 0.5 and abs(v.theta[1]) < 1.0


def test_a_finished_free_flight_run_freezes_instead_of_diverging():
    v, stepper = lit_vehicle("SIM_ONBOARD")
    v._abort("ABORT_MANUAL")  # SAFE
    advance_to_motor_time(v, stepper, v.plant_p.burn_time_s + 0.1)
    frozen = list(v.theta)
    for _ in range(2000):  # the aero model is unstable once the motor is out: it must not run on
        stepper.step()
    assert v.theta == frozen
