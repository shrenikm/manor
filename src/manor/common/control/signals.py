from __future__ import annotations

from typing import Protocol

import attr
import numpy as np


@attr.frozen
class ControlSignal(Protocol):
    def compute_signal(self, time_step: float) -> float:
        ...


@attr.frozen
class StepControlSignal:

    value: float
    delay_time: float

    def compute_signal(self, time_step: float) -> float:

        if time_step <= self.delay_time:
            return 0.0
        else:
            return self.value


@attr.frozen
class SineControlSignal:

    amplitude: float
    frequency: float
    phase_shift: float
    offset: float

    def compute_signal(self, time_step: float) -> float:

        return (
            self.amplitude
            * np.sin(2 * np.pi * self.frequency * time_step + self.phase_shift)
            + self.offset
        )

    @classmethod
    def standard_positive_signal(
        cls,
        amplitude: float,
        frequency: float,
    ) -> SineControlSignal:
        """
        Standard positive signal that starts at 0 and goes up to the given max value.
        It is equivalent to a regular sine signal of amplitude equal to amplitude / 2. which is also
        offset by amplitude / 2.
        In order to start at 0, the phase must be shifted by 3 * np.pi / 2.
        """
        return cls(
            amplitude=0.5 * amplitude,
            frequency=frequency,
            phase_shift=3 * np.pi / 2.0,
            offset=0.5 * amplitude,
        )
