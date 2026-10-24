"""
Week 5: FMCW radar -> Kalman tracker (detect-and-track pipeline).

Every frame: transmit a chirp, simulate the echo from a moving target
(your radar_sim.py from sdr-simulator), add receiver noise, take the
beat-frequency FFT, and read off the range. Those noisy ranges feed a
constant-velocity Kalman filter with state [range, range-rate].

Two operating points: healthy SNR, and just below the radar's
detection threshold, where noise peaks occasionally beat the target
and produce wild range "detections" (outliers). A plain Kalman filter
trusts them; a gated one rejects them.
"""

import numpy as np
import matplotlib.pyplot as plt
from radar_sim import generate_chirp, simulate_target_echo, range_fft, estimate_range

# ---------------------------------------------------------------
# Radar and scenario parameters (same radar as sdr-simulator Week 7)
# ---------------------------------------------------------------
fs = 500e6              # sample rate
bandwidth = 200e6       # -> 0.75 m range resolution
chirp_T = 20e-6
t = np.arange(0, chirp_T, 1 / fs)
tx, slope = generate_chirp(t, 0, bandwidth, chirp_T)

dt = 0.1                # radar frame interval (10 Hz)
n_frames = 60
v_true = 20.0           # m/s, moving away
N_SEEDS = 200
GATE = 9.0              # reject if innovation^2 / S > 9 (3 sigma)
Q_ACC = 1.0             # process noise q = sigma_a^2, sigma_a = 1 m/s^2


def radar_measure(r, snr_db, rng):
    rx = simulate_target_echo(tx, t, fs, r, v_true, slope, 0)
    pn = 10 ** (-snr_db / 10)
    rx = rx + np.sqrt(pn / 2) * (rng.standard_normal(len(rx))
                                 + 1j * rng.standard_normal(len(rx)))
    ranges, mag = range_fft(tx, rx, fs, slope)
    return estimate_range(ranges, mag)


def simulate(seed, snr_db):
    rng = np.random.default_rng(seed)
    r0 = rng.uniform(20, 30)
    truth = r0 + v_true * dt * np.arange(n_frames)
    meas = np.array([radar_measure(r, snr_db, rng) for r in truth])
    return truth, meas


# ---------------------------------------------------------------
# Kalman filter: state [range, range-rate], optional gating
# ---------------------------------------------------------------
F = np.array([[1, dt], [0, 1]])
H = np.array([[1.0, 0.0]])


def track(meas, R, gate=None, q=Q_ACC):
    Q = q * np.array([[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]])
    x = np.zeros((2, 1))
    P = np.eye(2) * 500
    est = np.zeros((len(meas), 2))
    rejected = np.zeros(len(meas), dtype=bool)
    for k, z in enumerate(meas):
        x = F @ x
        P = F @ P @ F.T + Q
        y = z - (H @ x)[0, 0]                 # innovation
        S = (H @ P @ H.T)[0, 0] + R           # innovation variance
        if gate is not None and y**2 / S > gate:
            rejected[k] = True                # outlier: coast on prediction
        else:
            K = (P @ H.T) / S
            x = x + K * y
            P = (np.eye(2) - K @ H) @ P
        est[k] = x[:, 0]
    return est, rejected


# ---------------------------------------------------------------
# R comes from the radar itself: std of its range error at healthy SNR
# ---------------------------------------------------------------
rng_cal = np.random.default_rng(123)
cal_err = np.array([radar_measure(r, -20, rng_cal) - r
                    for r in rng_cal.uniform(20, 120, 2000)])
sigma_meas = np.std(cal_err)
R = sigma_meas**2
print(f"Calibrated radar range noise: sigma = {sigma_meas:.3f} m "
      f"(theory, bin/sqrt(12) = {0.75 / np.sqrt(12):.3f} m), "
      f"bias = {np.mean(cal_err):.3f} m\n")


# ---------------------------------------------------------------
# Monte Carlo over random noise and starting range
# ---------------------------------------------------------------
def evaluate(snr_db):
    rows = {"Raw radar": [], "Kalman": [], "Kalman + gate": []}
    for seed in range(N_SEEDS):
        truth, meas = simulate(seed, snr_db)
        v_raw = np.concatenate([[0], np.diff(meas) / dt])   # differencing
        est, _ = track(meas, R)
        est_g, _ = track(meas, R, gate=GATE)
        for name, r_est, v_est in [("Raw radar", meas, v_raw),
                                   ("Kalman", est[:, 0], est[:, 1]),
                                   ("Kalman + gate", est_g[:, 0], est_g[:, 1])]:
            err = np.abs(r_est - truth)
            rows[name].append((err.mean(),
                               100 * (err > 5).mean(),
                               np.sqrt(np.mean((v_est[10:] - v_true) ** 2))))
    print(f"=== SNR {snr_db} dB ({N_SEEDS} seeds) ===")
    print(f"{'':16s}{'range MAE (m)':>15s}{'% frames >5 m off':>20s}{'vel RMSE (m/s)':>17s}")
    for name, vals in rows.items():
        m = np.mean(vals, axis=0)
        print(f"{name:16s}{m[0]:15.3f}{m[1]:20.2f}{m[2]:17.3f}")
    print()


evaluate(-20)
evaluate(-26)

# ---------------------------------------------------------------
# Demo plot: first seed at -26 dB with at least 3 outlier detections
# ---------------------------------------------------------------
for seed in range(100):
    truth, meas = simulate(seed, -26)
    if (np.abs(meas - truth) > 5).sum() >= 3:
        break
est, _ = track(meas, R)
est_g, rej = track(meas, R, gate=GATE)
frames = np.arange(n_frames)
out = np.abs(meas - truth) > 5

fig, ax = plt.subplots(2, 1, figsize=(10, 8))
ax[0].plot(frames, truth, "k", linewidth=2, label="True range")
ax[0].plot(frames[~out], meas[~out], ".", alpha=0.6, label="Radar detections")
ax[0].plot(frames, est[:, 0], label="Kalman (no gate)", linewidth=1.5)
ax[0].plot(frames, est_g[:, 0], label="Kalman + gate", linewidth=2)
ax[0].plot(frames[out], truth[out], "rx", markersize=9,
           label=f"Frames with outlier detections ({out.sum()})")
ax[0].set_ylim(truth.min() - 20, truth.max() + 20)
ax[0].set_xlabel("Radar frame (0.1 s each)")
ax[0].set_ylabel("Range (m)")
ax[0].set_title("Radar -> Kalman tracker at -26 dB SNR (near detection threshold)")
ax[0].legend(); ax[0].grid(True, alpha=0.3)

ax[1].plot(frames, est_g[:, 1], label="Kalman + gate: estimated range-rate")
ax[1].axhline(v_true, color="k", linestyle="--", label=f"True: {v_true} m/s")
ax[1].set_xlabel("Radar frame (0.1 s each)")
ax[1].set_ylabel("Range-rate (m/s)")
ax[1].set_title("Velocity recovered from range measurements alone")
ax[1].legend(); ax[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("../docs/week5_output.png", dpi=150)
plt.show()