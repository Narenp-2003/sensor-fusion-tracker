# Sensor Fusion Tracker

State estimation and sensor fusion in Python — Kalman filters built
from scratch, tracking noisy motion and fusing multiple sensors, with
a direct tie-in to real automotive/ADAS tracking pipelines.

## Motivation

Real-world sensors are noisy. A single GPS reading, radar detection,
or accelerometer sample can't be fully trusted on its own — but by
modeling how a system *should* move (physics) and continuously
correcting that model with new noisy measurements, a Kalman filter
produces an estimate more accurate than any individual sensor alone.

This is the core algorithm behind ADAS (advanced driver assistance
systems), autonomous vehicle tracking, aircraft navigation, and radar
target tracking — built here from first principles rather than using
a library like `filterpy`, so every step is understood and verifiable.

This project connects directly to [sdr-simulator](https://github.com/Narenp-2003/sdr-simulator),
where Week 7 built an FMCW radar range estimator — later weeks here
feed that radar's noisy range measurements into a Kalman filter to
demonstrate a full detect-and-track pipeline.

## Project Structure
sensor-fusion-tracker/
├── src/ # Core modules and weekly demo scripts
├── data/ # Any input data files
├── docs/ # Output plots and write-ups


## Setup
python -m venv sf-env
.\sf-env\Scripts\Activate.ps1
pip install numpy matplotlib


## Roadmap

- [x] **Week 1** — 1D Kalman filter (position-only state)
- [x] **Week 2** — 2D state vector (position + velocity), physics-based motion model
- [x] **Week 3** — Multi-sensor fusion (combining two noisy sensors)
- [ ] **Week 4** — 2D plane tracking (x-y motion)
- [ ] **Week 5** — Fusing radar data from `sdr-simulator` into the filter
- [ ] **Week 6** — Visualization polish, final write-up

## Week 1: 1D Kalman Filter

Run:
cd src
python week1_demo.py


Implements the classic predict-correct Kalman filter loop, tracking a
1D position from noisy measurements, assuming a **position-only** state (no explicit velocity model).

**Result — an important negative finding:** the filtered estimate
performed *worse* than the raw noisy measurements (MAE 10.6 vs 1.2,
a -750% "improvement"). This happens because the filter's prediction
step assumes the object stays still between measurements, so it
systematically lags behind a target moving at constant velocity —
smoothing confidently toward a wrong assumption rather than just
averaging out noise.

This motivates Week 2: adding velocity into the state vector so the
filter's predictions account for motion, not just measurement noise.
Output: `docs/week1_output.png`

## Week 2: Constant-Velocity Kalman Filter

Run:
cd src
python week2_demo.py


Extends the state to `[position, velocity]` with a constant-velocity motion
model (`F = [[1, dt], [0, 1]]`), fixing the lag problem found in Week 1.

**Result:** MAE improved from 2.74 (raw measurements) to 1.24 (filtered) —
a 54.9% improvement, confirming that modeling velocity explicitly lets the
filter track a moving target instead of assuming it's stationary.

Output: `docs/week2_output.png`

## Week 3: Multi-Sensor Fusion

Run:
cd src
python week3_demo.py


Two sensors with different noise levels independently measure the same
position each step. Both measurements are fused into a single state
estimate via sequential Kalman updates within each time step (predict
once, then update with sensor 1, then update again with sensor 2).

**Result:** MAE (sensor 1 alone): 3.66, MAE (sensor 2 alone): 1.06,
MAE (fused estimate): 0.48 — the fused estimate beats even the more
precise individual sensor, showing that combining measurements yields
a better estimate than trusting either sensor on its own.

Output: `docs/week3_output.png`

## License

MIT