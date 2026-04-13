from __future__ import annotations

import attr
import numpy as np

from manr.common.custom_types import (
    GripperPositionsVector,
    GripperVelocitiesVector,
    JointPositionsVector,
    JointVelocitiesVector,
)
from manr.common.definitions.control_definitions import ManipulatorGripperStatus


@attr.frozen
class ManipulatorState:
    """
    Class to represent the state of a manipulator.
    """

    joint_positions: JointPositionsVector
    joint_velocities: JointVelocitiesVector

    gripper_positions: GripperPositionsVector
    gripper_velocities: GripperVelocitiesVector
    gripper_status: ManipulatorGripperStatus

    @classmethod
    def create_dummy(cls) -> ManipulatorState:
        """
        Create a dummy manipulator state.
        Useful for declaring abstract states, etc.
        """
        return cls(
            joint_positions=np.empty(0),
            joint_velocities=np.empty(0),
            gripper_positions=np.empty(0),
            gripper_velocities=np.empty(0),
            gripper_status=ManipulatorGripperStatus.NEUTRAL,
        )
