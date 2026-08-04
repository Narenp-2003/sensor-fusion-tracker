"""
week1_demo.py
Week 1 deliverable: 1D Kalman filter vs raw noisy measurements vs ground truth.
Run: python week1_demo.py
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_1d import KalmanFilter1D, simulate_true_position, simulate_noisy_measurements


def main():
    n_steps = 50
    true_positions = simulate_true_position(n_steps, velocity=1.0)
    measurements = simulate_noisy_measurements(true_positions, noise_std=2.0)

    kf = KalmanFilter1D(
        initial_estimate=measurements[0],
        initial_uncertainty=1.0,
        process_variance=0.01,
        measurement_variance=4.0,  # variance = noise_std^2 = 2.0^2
    )

    filtered_estimates = []
    for z in measurements:
        kf.predict()
        estimate = kf.update(z)
        filtered_estimates.append(estimate)

    filtered_estimates = np.array(filtered_estimates)

    # --- Measure how much the filter improved accuracy ---
    raw_error = np.mean(np.abs(measurements - true_positions))
    filtered_error = np.mean(np.abs(filtered_estimates - true_positions))

    print(f"Mean absolute error - raw measurements: {raw_error:.3f}")
    print(f"Mean absolute error - Kalman filtered:  {filtered_error:.3f}")
    print(f"Improvement: {(1 - filtered_error/raw_error) * 100:.1f}%")

    # --- Plot ---
    plt.figure(figsize=(10, 6))
    plt.plot(true_positions, label="True position", color='black', linewidth=2)
    plt.scatter(range(n_steps), measurements, label="Noisy measurements", color='red', alpha=0.5, s=20)
    plt.plot(filtered_estimates, label="Kalman filtered estimate", color='blue', linewidth=2)
    plt.xlabel("Time step")
    plt.ylabel("Position")
    plt.title("1D Kalman Filter: Tracking Position from Noisy Measurements")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig("../docs/week1_output.png", dpi=150)
    print("\nSaved figure to ../docs/week1_output.png")


if __name__ == "__main__":
    main()