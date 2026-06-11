# TVC Project — CLAUDE.md

## Project overview
Custom Thrust Vector Control (TVC) model rocket: PID-Kalman architecture for
attitude control during powered ascent. Flagship 3-year project, basis for
research paper "Design and Validation of a Thrust Vector Controlled Model
Rocket Using PID-Kalman Architecture."

## Hardware
- Flight computer: Teensy 4.1
- IMU: MPU6050 (gyro + accel)
- Servos: SG90 (confirm vs. MG90S — open discrepancy, see paper Section 6)
- Motor: Estes E12-4
- Gimbal: custom 2-axis
- Control loop: PID at 200 Hz

## Repo structure
- `simulation/` — 12-file Python IAE simulation
  - `config.py` — physical parameters, constants
  - `motor.py` — thrust curve model
  - `dynamics.py` — equations of motion
  - `kalman.py` — Kalman filter implementation
  - `control.py` — PID controller
  - `flight_computer.py` — onboard logic emulation
  - `simulation.py` — main sim loop
  - `analysis.py` — IAE / metrics computation
  - `jacobian.py` — linearization for analysis
  - `sensitivity.py` — Monte Carlo / sensitivity sweeps
  - `figures.py` — paper figure generation
  - `main.py` — entry point
- `firmware/` — Teensy 4.1 C++ firmware
- `paper/` — LaTeX source (mirrors Overleaf), `references.bib`, figures
- `data/` — hardware test logs (CSV), simulation outputs

## Known issues / active fixes
- Aerodynamic damping was missing from dynamics model
- Wind torque moment arm was incorrect
- Accelerometer noise was in g-units, needs conversion to angle-equivalent degrees
- Kalman figure was re-synthesizing noise instead of using logged data
- Servo model: paper references MG90S, hardware uses SG90 — verify and reconcile
- IMU axis mapping: verify on bench before live fire

## Derived parameters
- Total length: ~0.648 m
- Moment of inertia: ~0.0231 kg·m²
- Moment arm: ~0.288 m
- Controller: pure PD (K_I = 0)
- Disturbance test: t = 0.6s, 0.12 N·m, 50ms duration
- Monte Carlo: 100 trials, 100/100 recovered

## Conventions
- Python 3, use `venv` per project
- Run figure generation via `figures.py`, outputs go to `paper/figures/`
- Keep simulation outputs out of git (see .gitignore) unless small and needed for reproducibility
- When editing the paper, mirror changes between local `.tex` and Overleaf manually unless Overleaf git sync is set up

## Style notes
- No em dashes in any generated text (paper, comments, docs) — flagged as AI-sounding in prior essay reviews
