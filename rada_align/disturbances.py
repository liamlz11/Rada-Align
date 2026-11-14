"""Disturbance models: wind, slow tripod drift, step quantization, backlash."""

from __future__ import annotations

import math

import numpy as np


class OUProcess:
    """Zero-mean Ornstein-Uhlenbeck process (low-frequency noise).

    Stationary standard deviation `sigma`, correlation time `tau_s`. The
    update is the exact discretization, so it is valid for any time step.
    """

    def __init__(self, sigma: float, tau_s: float, rng: np.random.Generator):
        self.sigma = sigma
        self.tau_s = tau_s
        self._rng = rng
        self.value = sigma * rng.standard_normal()

    def step(self, dt_s: float) -> float:
        draw = self._rng.standard_normal()
        if dt_s <= 0:
            return self.value
        if self.tau_s <= 0:
            self.value = self.sigma * draw
            return self.value
        decay = math.exp(-dt_s / self.tau_s)
        self.value = self.value * decay + self.sigma * math.sqrt(1.0 - decay * decay) * draw
        return self.value


class WindModel:
    """Angular motion of one tripod head on two axes (az, el).

    White jitter (new value every step) plus low-frequency sway.
    """

    def __init__(self, jitter_sigma: float, sway_sigma: float, sway_tau_s: float, rng):
        self._rng = rng
        self._jitter_sigma = jitter_sigma
        self._sway = [OUProcess(sway_sigma, sway_tau_s, rng) for _ in range(2)]
        self.value = (0.0, 0.0)
        self.step(0.0)

    def step(self, dt_s: float) -> tuple[float, float]:
        self.value = tuple(
            sway.step(dt_s) + self._jitter_sigma * self._rng.standard_normal()
            for sway in self._sway
        )
        return self.value


def quantize(value: float, step: float) -> float:
    """Round to the nearest whole step; step <= 0 means no quantization."""
    if step <= 0:
        return value
    return round(value / step) * step


class Backlash:
    """Dead zone between the motor and the mechanism.

    The mechanism only follows once the motor has taken up the play, so a
    reversal smaller than `width` does not move the load.
    """

    def __init__(self, width: float, initial_offset: float = 0.0):
        self.width = width
        self.mechanism = initial_offset

    def update(self, motor: float) -> float:
        half = self.width / 2.0
        if motor - self.mechanism > half:
            self.mechanism = motor - half
        elif self.mechanism - motor > half:
            self.mechanism = motor + half
        return self.mechanism
