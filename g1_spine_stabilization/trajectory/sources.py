"""
Trajectory drivers — produce the time-varying signal that the controller tracks.

The sine driver is the current placeholder. Session 3 will add an ArUco-backed
driver with the same interface so the controller doesn't need to change.
"""

import math


def sine_trajectory(t, amplitude, freq):
    """
    Sine wave rotation trajectory.

    Args:
        t:         time in seconds
        amplitude: peak amplitude in radians
        freq:      frequency in Hz

    Returns:
        theta: rotation angle in radians at time t
    """
    return amplitude * math.sin(2 * math.pi * freq * t)
