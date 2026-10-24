"""
Monte Carlo evaluation of Weeks 2-4: each experiment is repeated over
N_SEEDS random noise draws and reported as mean +/- std.
"""

import numpy as np

N_SEEDS = 200
dt = 1.0


# ---------------------------------------------------------------
# Shared Kalman helpers
# ---------------------------------------------------------------
def kf_update(x, P, z, H, R):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(len(x)) - K @ H) @ P
    return x, P


def cv_Q(q):
    return q * np.array([[dt**4 / 4, dt**3 / 2],
                         [dt**3 / 2, dt**2]])


def fmt(a):
    return f"{np.mean(a):.2f} ± {np.std(a):.2f}"


# ---------------------------------------------------------------
# Week 2: 1D constant-velocity filter, one sensor
# ---------------------------------------------------------------
def week2(seed, n=50, v=2.0, sigma=3.0):
    rng = np.random.default_rng(seed)
    truth = v * dt * np.arange(n)
    meas = truth + rng.normal(0, sigma, n)
    F = np.array([[1, dt], [0, 1]]); H = np.array([[1, 0]])
    Q = cv_Q(0.01); R = np.array([[sigma**2]])
    x = np.zeros((2, 1)); P = np.eye(2) * 500
    est = np.zeros(n)
    for k in range(n):
        x = F @ x; P = F @ P @ F.T + Q
        x, P = kf_update(x, P, np.array([[meas[k]]]), H, R)
        est[k] = x[0, 0]
    return (np.mean(np.abs(meas - truth)), np.mean(np.abs(est - truth)))


# ---------------------------------------------------------------
# Week 3: two sensors, sequential updates
# ---------------------------------------------------------------
def week3(seed, n=50, v=2.0, s1=4.0, s2=1.5):
    rng = np.random.default_rng(seed)
    truth = v * dt * np.arange(n)
    m1 = truth + rng.normal(0, s1, n)
    m2 = truth + rng.normal(0, s2, n)
    F = np.array([[1, dt], [0, 1]]); H = np.array([[1, 0]])
    Q = cv_Q(0.01); R1 = np.array([[s1**2]]); R2 = np.array([[s2**2]])

    def run(use1, use2):
        x = np.zeros((2, 1)); P = np.eye(2) * 500
        est = np.zeros(n)
        for k in range(n):
            x = F @ x; P = F @ P @ F.T + Q
            if use1:
                x, P = kf_update(x, P, np.array([[m1[k]]]), H, R1)
            if use2:
                x, P = kf_update(x, P, np.array([[m2[k]]]), H, R2)
            est[k] = x[0, 0]
        return np.mean(np.abs(est - truth))

    return (np.mean(np.abs(m1 - truth)), np.mean(np.abs(m2 - truth)),
            run(True, False), run(False, True), run(True, True))


# ---------------------------------------------------------------
# Week 4: 2D tracking, straight vs turning target
# ---------------------------------------------------------------
def make_truth(n, turn):
    pos = np.zeros((n, 2)); vel = np.array([2.0, 1.0])
    a = np.deg2rad(6.0)
    Rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    for k in range(1, n):
        if turn and k >= 30:
            vel = Rot @ vel
        pos[k] = pos[k - 1] + vel * dt
    return pos


def week4(seed, turn, qs, n=60, sigma=3.0):
    rng = np.random.default_rng(seed)
    truth = make_truth(n, turn)
    meas = truth + rng.normal(0, sigma, truth.shape)
    F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    R = np.eye(2) * sigma**2
    out = [np.mean(np.linalg.norm(meas - truth, axis=1))]
    for q in qs:
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = cv_Q(q)
        Q[np.ix_([1, 3], [1, 3])] = cv_Q(q)
        x = np.zeros((4, 1)); P = np.eye(4) * 500
        est = np.zeros((n, 2))
        for k in range(n):
            x = F @ x; P = F @ P @ F.T + Q
            x, P = kf_update(x, P, meas[k].reshape(2, 1), H, R)
            est[k] = x[:2, 0]
        out.append(np.mean(np.linalg.norm(est - truth, axis=1)))
    return out


if __name__ == "__main__":
    seeds = range(N_SEEDS)

    r = np.array([week2(s) for s in seeds])
    print(f"=== Week 2 ({N_SEEDS} seeds, mean ± std) ===")
    print(f"Raw MAE:      {fmt(r[:, 0])}")
    print(f"Filtered MAE: {fmt(r[:, 1])}")
    print(f"Improvement:  {100 * (1 - r[:, 1].mean() / r[:, 0].mean()):.1f}%")

    r = np.array([week3(s) for s in seeds])
    print(f"\n=== Week 3 ({N_SEEDS} seeds, mean ± std) ===")
    for name, col in zip(["Sensor 1 raw", "Sensor 2 raw", "KF sensor 1 only",
                          "KF sensor 2 only", "KF fused"], r.T):
        print(f"{name:18s} {fmt(col)}")
    print(f"Fused beats KF sensor 2 only in {np.sum(r[:, 4] < r[:, 3])}/{N_SEEDS} seeds")

    qs = [0.01, 0.1, 0.5]
    for name, turn in [("Straight", False), ("Turn", True)]:
        r = np.array([week4(s, turn, qs) for s in seeds])
        print(f"\n=== Week 4: {name} ({N_SEEDS} seeds, mean ± std) ===")
        print(f"Raw:        {fmt(r[:, 0])}")
        for i, q in enumerate(qs, start=1):
            worse = np.sum(r[:, i] > r[:, 0])
            print(f"KF q={q:<5} {fmt(r[:, i])}  (worse than raw in {worse}/{N_SEEDS})")