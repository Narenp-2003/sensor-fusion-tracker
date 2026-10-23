"""
Week 4: 2D plane tracking (x-y motion).

State = [x, y, vx, vy]. Constant-velocity model, noisy (x, y) position
measurements. Two scenarios: straight-line target, and a target that
turns at step 30 (violates the constant-velocity assumption).
"""

import numpy as np
import matplotlib.pyplot as plt

dt = 1.0
n_steps = 60
meas_std = 3.0


def make_truth(turn):
    pos = np.zeros((n_steps, 2))
    vel = np.array([2.0, 1.0])
    ang = np.deg2rad(6.0)                       # turn rate (deg/step)
    Rot = np.array([[np.cos(ang), -np.sin(ang)],
                    [np.sin(ang),  np.cos(ang)]])
    for k in range(1, n_steps):
        if turn and k >= 30:
            vel = Rot @ vel
        pos[k] = pos[k - 1] + vel * dt
    return pos


F = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1,  0],
              [0, 0, 0,  1]])
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])
R = np.eye(2) * meas_std**2


def make_Q(q):
    blk = np.array([[dt**4/4, dt**3/2],
                    [dt**3/2, dt**2]])
    Q = np.zeros((4, 4))
    Q[np.ix_([0, 2], [0, 2])] = q * blk         # x, vx
    Q[np.ix_([1, 3], [1, 3])] = q * blk         # y, vy
    return Q


def run_filter(meas, q):
    Q = make_Q(q)
    x = np.zeros((4, 1))
    P = np.eye(4) * 500
    est = np.zeros((n_steps, 2))
    for k in range(n_steps):
        x = F @ x
        P = F @ P @ F.T + Q
        z = meas[k].reshape(2, 1)
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        P = (np.eye(4) - K @ H) @ P
        est[k] = x[:2, 0]
    return est


def mean_err(a, truth):
    return np.mean(np.linalg.norm(a - truth, axis=1))


results = {}
for name, turn in [("straight", False), ("turn", True)]:
    np.random.seed(0)
    truth = make_truth(turn)
    meas = truth + np.random.normal(0, meas_std, truth.shape)
    row = {"raw": mean_err(meas, truth)}
    for q in [0.01, 0.5]:
        row[q] = mean_err(run_filter(meas, q), truth)
    results[name] = (truth, meas, row)

for name, (_, _, row) in results.items():
    print(f"[{name}] raw: {row['raw']:.2f} | q=0.01: {row[0.01]:.2f} | q=0.5: {row[0.5]:.2f}")

truth, meas, _ = results["turn"]
plt.figure(figsize=(8, 6))
plt.plot(truth[:, 0], truth[:, 1], label="True path", linewidth=2)
plt.scatter(meas[:, 0], meas[:, 1], s=12, alpha=0.4, label="Noisy measurements")
plt.plot(*run_filter(meas, 0.01).T, label="KF (q=0.01)", linewidth=2)
plt.plot(*run_filter(meas, 0.5).T, label="KF (q=0.5)", linewidth=2)
plt.xlabel("x"); plt.ylabel("y"); plt.axis("equal")
plt.title("Week 4: 2D Tracking (target turns at step 30)")
plt.legend(); plt.tight_layout()
plt.savefig("../docs/week4_output.png")
plt.show()