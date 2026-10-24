# Sensor Fusion Tracker: Write-up

## What this project is

Kalman filters built from scratch in Python, extended over five weeks:
1D tracking → constant-velocity model → two-sensor fusion → 2D tracking →
an FMCW radar (from my [sdr-simulator](https://github.com/Narenp-2003/sdr-simulator))
feeding a tracker. All data is simulated, so the ground truth is known and
every claim can be checked.

## How I made the results trustworthy

My first results came from a single random seed, and two of my own
claims turned out to be seed artifacts: fusion looked 26% better than a
single-sensor filter (it is really ~6%), and a filter looked worse than
raw data on a turning target (true in only 35 of 200 runs). So every
headline number is now a mean ± std over 200 seeds (`monte_carlo.py`;
Week 5 has its own evaluation).

The ~6% fusion gain also matches theory: the noisier sensor carries about
14% as much information as the better one (1/σ²: 0.0625 vs 0.444), which
predicts a 1 − 1/√1.14 ≈ 6.4% error reduction.

## Three failure modes

**1. A wrong motion model (Weeks 1 and 4).** The Week 1 filter assumed a
stationary target, so it lagged a moving one and ended up worse than the
raw measurements (MAE 10.6 vs 1.2 in the demo run). Adding velocity to the
state fixed it: error fell from 2.38 to 1.19 (50%). The same failure
returned in Week 4, when a target turned and the constant-velocity model
stopped being true.

**2. Mis-tuned process noise (Week 4).** Process noise q is the variance of
the acceleration the model does not capture. For the turn in my simulation,
a = vω ≈ 0.23 units/step², so q ≈ 0.055. A sweep found the best
turning-target error around q = 0.1 (2.40), while q = 0.01 gave 3.37 and was
worse than raw in 35 of 200 runs. Straight-line tracking prefers a tiny q
(1.85). Smoothing and responsiveness trade off, and no single q wins both.

**3. Non-Gaussian outliers (Week 5).** My radar has 0.75 m range resolution;
its measured range noise (σ = 0.232 m) is close to the 0.217 m theory
(bin/√12). Near its detection threshold (−26 dB per-sample SNR), noise peaks
beat the real target in about 4.8% of frames and return wild ranges. A plain
Kalman filter trusts them: 107 m mean error, with 51% of frames more than 5 m
off, worse than raw detections (86 m). Innovation gating (skip an update when
y²/S > 9, i.e. beyond 3σ) restored 0.174 m error with 0.05% of frames off and
0.093 m/s velocity error.

## What worked

- On well-modelled motion, a Kalman filter roughly halves position error.
- Fusing sensors weights each by its information, so a weak sensor helps
  only a little (~6%).
- Velocity from range alone: 0.057 m/s RMSE at healthy SNR, versus
  3.539 m/s from differencing raw ranges.

## Limitations

- Simulated data only; no real sensor recordings.
- The radar model has no Doppler (`f_start = 0`), a constant-velocity target,
  a single target (no data association), and a −0.146 m range bias from
  truncating the echo delay to whole samples. A filter cannot remove a bias.
- q was chosen by sweeping on simulated data; I did not test a maneuvering
  target in the radar scenario.
- Week 1 is a single demo run, not a Monte Carlo result.

## What I would do next

- Handle maneuvers properly (an IMM filter, or an EKF with a turn model).
- Multi-target tracking with data association.
- Add Doppler to the radar and test on real automotive radar data.