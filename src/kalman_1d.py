"""
kalman_1d.py
Week 1: 1D Kalman Filter fundamentals

Tracks a single moving object's position using noisy measurements,
using the classic predict-correct Kalman filter loop.
"""

import numpy as np


class KalmanFilter1D:
    """
    A minimal 1D Kalman filter tracking a single scalar state
    (e.g. position), assuming constant velocity motion.

    State: x (position)
    """

    def __init__(self, initial_estimate, initial_uncertainty,
                 process_variance, measurement_variance):
        self.x = initial_estimate          # current state estimate
        self.p = initial_uncertainty        # current estimate uncertainty (variance)
        self.q = process_variance           # how much we trust our motion model
        self.r = measurement_variance       # how noisy we believe measurements are

    def predict(self):
        """
        Predict step: project the state forward. For this simple 1D
        case with no explicit motion model yet, we assume the state
        doesn't change on its own, but uncertainty grows over time
        (we become less sure the longer we go without a measurement).
        """
        self.p = self.p + self.q
        return self.x

    def update(self, measurement):
        """
        Correct step: blend the prediction with a new noisy
        measurement, weighted by their relative uncertainties.
        The Kalman gain determines how much to trust the new
        measurement vs the existing estimate.
        """
        kalman_gain = self.p / (self.p + self.r)
        self.x = self.x + kalman_gain * (measurement - self.x)
        self.p = (1 - kalman_gain) * self.p
        return self.x


def simulate_true_position(n_steps, velocity=1.0, dt=1.0):
    """Generate ground-truth positions for an object moving at constant velocity."""
    return np.array([i * velocity * dt for i in range(n_steps)])


def simulate_noisy_measurements(true_positions, noise_std=2.0, seed=42):
    """Add Gaussian noise to true positions, simulating a real sensor."""
    rng = np.random.default_rng(seed)
    noise = rng.normal(0, noise_std, len(true_positions))
    return true_positions + noise