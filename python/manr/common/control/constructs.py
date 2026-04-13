from __future__ import annotations

import attr
import numpy as np

from manr.common.custom_types import GainsVector


@attr.frozen
class PIDGains:
    kp: GainsVector
    ki: GainsVector
    kd: GainsVector

    @classmethod
    def from_scalar_gains(
        cls,
        size: int,
        kp_scalar: float,
        ki_scalar: float,
        kd_scalar: float,
    ) -> PIDGains:
        return PIDGains(
            kp=kp_scalar * np.ones(size, dtype=np.float64),
            ki=ki_scalar * np.ones(size, dtype=np.float64),
            kd=kd_scalar * np.ones(size, dtype=np.float64),
        )
