"""
Week 2: 2D state Kalman filter (position + velocity), constant-velocity model.

Tracks a target moving at (roughly) constant velocity from noisy position-only
measurements. Compares against Week 1's position-only filter to show the fix
for the lag problem found in Week 1.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------
np.random.seed(0)

dt = 1.0                # time step
n_steps = 50
true_velocity = 2.0      # constant velocity, units/step
measurement_noise_std = 3.0

true_position = np.zeros(n_steps)
for k in range(1, n_steps):
    true_position[k] = true_position[k - 1] + true_velocity * dt

measurements = true_position + np.random.normal(0, measurement_noise_std, n_steps)

# ---------------------------------------------------------------
# Kalman filter setup (state = [position, velocity])
# ---------------------------------------------------------------
F = np.array([[1, dt],
              [0, 1]])          # state transition (constant velocity model)

H = np.array([[1, 0]])          # we only measure position

process_var = 0.01
Q = process_var * np.array([[dt**4 / 4, dt**3 / 2],
                             [dt**3 / 2, dt**2]])   # process noise

R = np.array([[measurement_noise_std**2]])          # measurement noise

x = np.array([[0], [0]])        # initial state estimate: position=0, velocity=0
P = np.eye(2) * 500             # initial uncertainty (high, since we're unsure)

estimates = np.zeros(n_steps)

for k in range(n_steps):
    # ---- Predict ----
    x = F @ x
    P = F @ P @ F.T + Q

    # ---- Update ----
    z = np.array([[measurements[k]]])
    y = z - H @ x                       # innovation
    S = H @ P @ H.T + R                 # innovation covariance
    K = P @ H.T @ np.linalg.inv(S)      # Kalman gain

    x = x + K @ y
    P = (np.eye(2) - K @ H) @ P

    estimates[k] = x[0, 0]

# ---------------------------------------------------------------
# Results
# ---------------------------------------------------------------
mae_raw = np.mean(np.abs(measurements - true_position))
mae_filtered = np.mean(np.abs(estimates - true_position))

print(f"MAE (raw measurements): {mae_raw:.2f}")
print(f"MAE (filtered, 2D state): {mae_filtered:.2f}")
print(f"Improvement: {100 * (mae_raw - mae_filtered) / mae_raw:.1f}%")

plt.figure(figsize=(9, 5))
plt.plot(true_position, label="True position", linewidth=2)
plt.plot(measurements, label="Noisy measurements", alpha=0.5, linestyle="--")
plt.plot(estimates, label="Kalman filter estimate (pos+vel)", linewidth=2)
plt.xlabel("Time step")
plt.ylabel("Position")
plt.title("Week 2: Constant-Velocity Kalman Filter")
plt.legend()
plt.tight_layout()
plt.savefig("../docs/week2_output.png")
plt.show()