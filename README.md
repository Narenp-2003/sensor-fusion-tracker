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

```
sensor-fusion-tracker/
├── src/    # Core modules and weekly demo scripts
├── data/   # Any input data files
└── docs/   # Output plots and write-ups
```

## Setup

```
python -m venv sf-env
.\sf-env\Scripts\Activate.ps1
pip install numpy matplotlib
```

## Roadmap

- [x] **Week 1** — 1D Kalman filter (position-only state)
- [x] **Week 2** — 2D state vector (position + velocity), physics-based motion model
- [x] **Week 3** — Multi-sensor fusion (combining two noisy sensors)
- [ ] **Week 4** — 2D plane tracking (x-y motion)
- [ ] **Week 5** — Fusing radar data from `sdr-simulator` into the filter
- [ ] **Week 6** — Visualization polish, final write-up

## Week 1: 1D Kalman Filter

Run:

```
cd src
python week1_demo.py
```

Implements the classic predict-correct Kalman filter loop, tracking a
1D position from noisy measurements, assuming a **position-only** state
(no explicit velocity model).

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

```
cd src
python week2_demo.py
```

State vector is `[position, velocity]` with a constant-velocity motion
model (`F = [[1, dt], [0, 1]]`, position-only measurement `H = [1, 0]`).
This fixes Week 1's lag: the prediction step now carries the target
forward at its estimated velocity instead of assuming it stays still.

Setup: true velocity 2.0 units/step, measurement noise σ = 3.0,
50 steps, seed 0.

**Result:** MAE dropped from 2.74 (raw measurements) to 1.24 (filtered),
a 54.9% improvement. After the initial transient (steps 10+), MAE is
0.97. The final velocity estimate is 1.90 (true: 2.0).

Note: Week 1 and Week 2 use different noise settings, so their MAE
numbers are not directly comparable.
Output: `docs/week2_output.png`

## Week 3: Multi-Sensor Fusion

Run:

```
cd src
python week3_demo.py
```

Same constant-velocity filter, now fed by two independent position
sensors (σ = 4.0 and σ = 1.5). One predict step per time step, then
sequential updates: sensor 1, then sensor 2.

| Estimator | MAE |
|---|---|
| Sensor 1 raw | 3.66 |
| Sensor 2 raw | 1.06 |
| Kalman, sensor 1 only | 1.57 |
| Kalman, sensor 2 only | 0.65 |
| **Kalman, fused** | **0.48** |

**Result:** fusion beats both raw sensors and the best single-sensor
filter. The noisier sensor still helps, but contributes only ~12% of
the measurement information (1/σ² = 0.0625 vs 0.444).

**Verification:** sequential updates match a single batch update with
stacked `H` and `R = diag(σ₁², σ₂²)` to within 1e-13, as expected for
independent sensor noise.
Output: `docs/week3_output.png`

## License

MIT
