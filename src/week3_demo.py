"""
Week 3: Multi-sensor fusion (combining two noisy sensors).

Same constant-velocity Kalman filter as Week 2, but now two sensors
independently measure position each step (different noise levels).
Both measurements are fused into one estimate via sequential updates:
predict once, then update with sensor 1, then update again with sensor 2.
"""

import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------
# Simulation setup
# ---------------------------------------------------------------
np.random.seed(0)

dt = 1.0
n_steps = 50
true_velocity = 2.0

# Two sensors, different noise characteristics
sensor1_noise_std = 4.0   # noisier
sensor2_noise_std = 1.5   # more precise

true_position = np.zeros(n_steps)
for k in range(1, n_steps):
    true_position[k] = true_position[k - 1] + true_velocity * dt

meas1 = true_position + np.random.normal(0, sensor1_noise_std, n_steps)
meas2 = true_position + np.random.normal(0, sensor2_noise_std, n_steps)

# ---------------------------------------------------------------
# Kalman filter setup (state = [position, velocity])
# ---------------------------------------------------------------
F = np.array([[1, dt],
              [0, 1]])

H = np.array([[1, 0]])   # both sensors measure position only

process_var = 0.01
Q = process_var * np.array([[dt**4 / 4, dt**3 / 2],
                             [dt**3 / 2, dt**2]])

R1 = np.array([[sensor1_noise_std**2]])
R2 = np.array([[sensor2_noise_std**2]])

x = np.array([[0], [0]])
P = np.eye(2) * 500

estimates = np.zeros(n_steps)


def update(x, P, z, R):
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(2) - K @ H) @ P
    return x, P


for k in range(n_steps):
    # ---- Predict (once per time step) ----
    x = F @ x
    P = F @ P @ F.T + Q

    # ---- Update with sensor 1, then sensor 2 ----
    x, P = update(x, P, np.array([[meas1[k]]]), R1)
    x, P = update(x, P, np.array([[meas2[k]]]), R2)

    estimates[k] = x[0, 0]

# ---------------------------------------------------------------
# Results
# ---------------------------------------------------------------
mae_sensor1 = np.mean(np.abs(meas1 - true_position))
mae_sensor2 = np.mean(np.abs(meas2 - true_position))
mae_fused = np.mean(np.abs(estimates - true_position))

print(f"MAE (sensor 1 alone): {mae_sensor1:.2f}")
print(f"MAE (sensor 2 alone): {mae_sensor2:.2f}")
print(f"MAE (fused estimate): {mae_fused:.2f}")

plt.figure(figsize=(9, 5))
plt.plot(true_position, label="True position", linewidth=2)
plt.plot(meas1, label="Sensor 1 (noisy)", alpha=0.4, linestyle="--")
plt.plot(meas2, label="Sensor 2 (less noisy)", alpha=0.4, linestyle="--")
plt.plot(estimates, label="Fused Kalman estimate", linewidth=2)
plt.xlabel("Time step")
plt.ylabel("Position")
plt.title("Week 3: Multi-Sensor Fusion")
plt.legend()
plt.tight_layout()
plt.savefig("../docs/week3_output.png")
plt.show()