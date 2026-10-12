#!/usr/bin/env python3
"""TVC simulation entry point (SPEC.md Section 6).

Implemented so far: --serve only, the virtual vehicle behind ground/gs.py --sim (sim/serve.py).
The paper scenarios (baseline, open_loop, disturbance, sensitivity, controllability_map,
monte_carlo), --stand, and --hil are Phase 2/6 work (SPEC Section 9) and are not built yet;
naming one says so instead of silently doing something else.

    python3 sim/run_sim.py --serve                      # tcp://127.0.0.1:5760, STAND_OPEN
    python3 sim/run_sim.py --serve --mode STAND_CLOSED --disturbance
"""
import argparse
import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(SIM_DIR.parent / "protocol"))

SCENARIOS = ["baseline", "open_loop", "disturbance", "sensitivity", "controllability_map",
             "monte_carlo"]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?", choices=SCENARIOS)
    ap.add_argument("--serve", action="store_true",
                    help="run a virtual vehicle speaking the telemetry protocol over TCP")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--mode", default="STAND_OPEN",
                    choices=["SIM_ONBOARD", "HIL", "STAND_OPEN", "STAND_CLOSED", "FLIGHT"],
                    help="boot mode (the jumpers); the GUI can change it in IDLE")
    ap.add_argument("--theta0", type=float, nargs=2, default=[5.0, -2.0], metavar=("PITCH", "YAW"),
                    help="initial tilt [deg] for closed-loop modes (paper baseline: 5 deg)")
    ap.add_argument("--disturbance", action="store_true",
                    help="paper disturbance case: 0.12 N*m pitch torque for 50 ms at 0.6 s")
    ap.add_argument("--loss", type=float, default=0.0, help="downlink frame loss, 0..1")
    ap.add_argument("--arm-switch", choices=["closed", "open"], default="closed")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0, help="x real time")
    args = ap.parse_args()

    if args.scenario or not args.serve:
        sys.exit("run_sim.py: only --serve exists so far; the paper scenarios, --stand and --hil "
                 "are Phase 2/6 (SPEC.md Section 9) and have not been built.")

    import serve
    vehicle = serve.VirtualVehicle(
        mode=args.mode, seed=args.seed, theta0=tuple(args.theta0),
        arm_switch_closed=args.arm_switch == "closed", loss=args.loss,
        disturbance=(0.6, 0.12, 0.05) if args.disturbance else None)
    used = {k: v for k, v in serve.SERVE_PLACEHOLDERS.items()
            if serve.param(*k.split(".", 1)) == v}
    print(f"[serve] tcp://{args.host}:{args.port}  mode={args.mode}  params {serve.P.PARAMS_HASH[:12]}")
    if used:
        print("[serve] params.yaml MEASURE values replaced for the sim only: " +
              ", ".join(f"{k}={v:g}" for k, v in used.items()))
    print("[serve] Ctrl-C to stop" + ("; type `arm open`, `cont open`, `loss 0.1` to poke it"
                                     if sys.stdin.isatty() else ""), flush=True)
    serve.run(vehicle, args.host, args.port, speed=args.speed)


if __name__ == "__main__":
    main()
