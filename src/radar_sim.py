"""
radar_sim.py
Bonus Week 7: FMCW Radar Simulation

Simulates the core signal processing behind automotive FMCW radar
(the same principle used in adaptive cruise control, collision
detection, and blind-spot monitoring systems).
"""

import numpy as np


def generate_chirp(t, f_start, bandwidth, chirp_duration):
    """
    Generate a linear FMCW chirp: frequency sweeps linearly from
    f_start to f_start + bandwidth over chirp_duration.
    """
    slope = bandwidth / chirp_duration
    phase = 2 * np.pi * (f_start * t + 0.5 * slope * t**2)
    return np.exp(1j * phase), slope


def simulate_target_echo(tx_chirp, t, sample_rate, target_range, target_velocity,
                          slope, f_start, c=3e8):
    """
    Simulate the received echo from a single target at a given range
    (meters) and radial velocity (m/s, positive = moving away).

    Applies:
      - Time delay corresponding to round-trip distance (2*range/c)
      - Doppler frequency shift corresponding to target velocity
    """
    delay = 2 * target_range / c
    delay_samples = int(delay * sample_rate)

    # Doppler shift: moving away redshifts (lowers) the frequency
    doppler_shift = -2 * target_velocity * f_start / c

    # Shift the chirp in time (simulating round-trip delay)
    echo = np.zeros_like(tx_chirp)
    if delay_samples < len(tx_chirp):
        echo[delay_samples:] = tx_chirp[:len(tx_chirp) - delay_samples]

    # Apply Doppler shift as an extra phase rotation over time
    echo = echo * np.exp(1j * 2 * np.pi * doppler_shift * t)

    return echo


def range_fft(tx_chirp, rx_echo, sample_rate, slope, c=3e8):
    """
    Compute the beat signal (mix of tx and rx), then FFT it to
    estimate target range. The beat frequency is proportional to
    round-trip delay, which is proportional to range.
    """
    beat_signal = tx_chirp * np.conj(rx_echo)

    n = len(beat_signal)
    fft_vals = np.fft.fft(beat_signal)
    freqs = np.fft.fftfreq(n, d=1 / sample_rate)

    # Only positive frequencies are physically meaningful here
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    magnitude = np.abs(fft_vals[pos_mask])

    # Convert beat frequency -> range: R = (beat_freq * c) / (2 * slope)
    ranges = (freqs * c) / (2 * slope)

    return ranges, magnitude


def estimate_range(ranges, magnitude):
    """Find the range corresponding to the strongest peak in the beat spectrum."""
    peak_idx = np.argmax(magnitude)
    return ranges[peak_idx]