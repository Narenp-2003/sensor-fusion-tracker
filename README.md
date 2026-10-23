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
- [x] **Week 4** — 2D plane tracking (x-y motion)
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

![Week 1 output](docs/week1_output.png)

## Week 2: Constant-Velocity Kalman Filter

Run:

```
cd src
python week2_demo.py
```

Extends the state to `[position, velocity]` with a constant-velocity
motion model (`F = [[1, dt], [0, 1]]`, position-only measurement
`H = [1, 0]`), fixing the lag problem found in Week 1.

Setup: true velocity 2.0 units/step, measurement noise σ = 3.0,
50 steps, seed 0.

**Result:** MAE improved from 2.74 (raw measurements) to 1.24 (filtered),
a 54.9% improvement. After the initial transient (steps 10+), MAE is
0.97. The final velocity estimate is 1.90 (true: 2.0).

Note: Week 2 uses σ = 3.0 noise, different from Week 1, so MAE values
are not directly comparable across weeks.

![Week 2 output](docs/week2_output.png)

## Week 3: Multi-Sensor Fusion

Run:

```
cd src
python week3_demo.py
```

Two independent sensors with different noise levels (σ = 4.0 and
σ = 1.5) measure the same position each step. Both measurements are
fused into a single state estimate via sequential Kalman updates
within each time step (predict once, then update with sensor 1, then
update again with sensor 2).

| Estimator | MAE |
|---|---|
| Sensor 1 raw | 3.66 |
| Sensor 2 raw | 1.06 |
| Kalman, sensor 1 only | 1.57 |
| Kalman, sensor 2 only | 0.65 |
| **Kalman, fused** | **0.48** |

**Result:** filtering accounts for most of the improvement over raw
data (sensor 2: 1.06 → 0.65), and fusion adds a further ~26%
reduction over the best single-sensor filter (0.65 → 0.48). The
noisier sensor contributes only ~12% of the measurement information
(1/σ² = 0.0625 vs 0.444), which is why the gain is modest.

**Verification:** sequential updates match a single batch update with
stacked `H` and `R = diag(σ₁², σ₂²)` to within 1e-13, as expected for
independent sensor noise.

![Week 3 output](docs/week3_output.png)

## Week 4: 2D Plane Tracking

Run:

```
cd src
python week4_demo.py
```

State is `[x, y, vx, vy]` with a constant-velocity model and noisy
(x, y) position measurements (σ = 3.0). Two scenarios: a straight-line
target, and a target that turns 6°/step from step 30. Each is run with
low process noise (q = 0.01) and high process noise (q = 0.5).
Metric: mean Euclidean position error, 60 steps, seed 0.

| Scenario | Raw | KF (q=0.01) | KF (q=0.5) |
|---|---|---|---|
| Straight line | 3.95 | **2.33** | 2.75 |
| Turning target | 3.95 | 4.12 | **2.90** |

**Result:** on a straight path the filter cuts error by ~41%. When the
target maneuvers, the low-noise filter does *worse than raw* (4.12 vs
3.95) because the constant-velocity model is wrong — the same failure
mode as Week 1. Raising process noise `q` restores tracking (2.90) at
the cost of smoothing on straight segments, illustrating the core
Kalman tuning trade-off between smoothing and responsiveness.

![Week 4 output](docs/week4_output.png)

## License

MIT