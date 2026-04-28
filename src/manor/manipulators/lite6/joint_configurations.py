"""
Named joint-angle configurations for the Ufactory Lite6.

The enum names a small set of well-known poses (ZERO, PRIME, ...); each member exposes
get_joint_positions_vector() to resolve to its 6-element joint-angle vector in radians.

Add a new entry only when the configuration is referenced from code; one-off poses for a single experiment
should be passed inline rather than added here.
"""

from __future__ import annotations

from enum import StrEnum

import numpy as np


class Lite6JointConfiguration(StrEnum):
    """
    Well-known joint-space pose names for the Lite6.

    - ZERO -- all joints at 0 rad. The factory default. The arm sits close to a self-collision boundary in
    this pose on the current fixture, so it shouldn't dwell here during operation; we only pass through it
    on the way to disabling motors at unprime.
    - PRIME -- a known-clear operational start pose. prime() moves here at the end of bring-up so any
    subsequent command starts from a safe configuration; mirrored by unprime() returning to ZERO before
    disabling.
    """

    ZERO = "zero"
    PRIME = "prime"

    def get_joint_positions_vector(self) -> np.ndarray:
        """
        Return the 6-element joint-angle vector (radians, j1..j6) for this configuration as a fresh
        float64 numpy array.
        """
        match self:
            case Lite6JointConfiguration.ZERO:
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            case Lite6JointConfiguration.PRIME:
                return np.array([0.0, 0.1733, 0.5550, 0.0, 0.3817, 0.0], dtype=np.float64)
