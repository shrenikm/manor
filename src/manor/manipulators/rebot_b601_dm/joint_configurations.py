"""
Named joint-angle configurations for the Seeed reBot B601 DM.

The enum names a small set of well-known poses (REST, PRIME, ...); each member exposes
get_joint_positions_vector() to resolve to its 6-element joint-angle vector in radians.

Add a new entry only when the configuration is referenced from code; one-off poses for a single experiment
should be passed inline rather than added here.
"""

from __future__ import annotations

from enum import StrEnum

import numpy as np


class RebotB601DmJointConfiguration(StrEnum):
    """
    Well-known joint-space pose names for the reBot B601 DM.

    - REST -- the all-zero vendor home configuration: shoulder and forearm horizontal, gripper pointing
    forward, everything parallel to the ground. This is the pose the manufacturer's tooling
    (initial_positions.yaml in the ROS2 stack, MotorBridge zeroing) treats as the origin, so parking and
    power-down happen here. Note that joints 2 and 3 sit exactly at their upper joint limit (0.0) in this
    configuration, and the folded arm packs so compactly that the tight collision primitives of
    non-adjacent links (link1/link2 against link4/link5) interpenetrate by a few millimetres. The arm
    shouldn't dwell here during operation; we only pass through REST on the way to disabling motors at
    unprime.
    - PRIME -- a known-clear operational start pose with the elbow raised and the gripper in front of and
    above the base, clear of the mounting table. prime() moves here at the end of bring-up so any subsequent
    command starts from a safe configuration; mirrored by unprime() returning to REST before disabling.
    """

    REST = "rest"
    PRIME = "prime"

    def get_joint_positions_vector(self) -> np.ndarray:
        """
        Return the 6-element joint-angle vector (radians, j1..j6) for this configuration as a fresh
        float64 numpy array.
        """
        match self:
            case RebotB601DmJointConfiguration.REST:
                return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
            case RebotB601DmJointConfiguration.PRIME:
                return np.array([0.0, -0.75, -0.65, 0.0, 0.0, 0.0], dtype=np.float64)
