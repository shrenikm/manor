"""
Concrete ``IManipulatorModel`` for the Ufactory Lite6.

Per-variant data (description filename, MultibodyPlant position count)
lives in module-level dicts keyed by ``Lite6Variant``. The arm DOF count
and frame names are constant across all variants: the Lite6 arm always
has 6 joints, and every supported variant uses ``link_base`` /
``link_eef_tip`` for its base / EEF-tip frames.
"""

from __future__ import annotations

import os
from typing import override

import attr

from manor.common.custom_types import FilePath
from manor.common.model_utils import ROBOT_MODELS_DRAKE_URDF_DIRNAME, get_robot_models_directory_path
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType

LITE6_ARM_DOF = 6
LITE6_PARALLEL_GRIPPER_DOF = 2

_LITE6_DESCRIPTION_DIRNAME = "lite6_description"
_LITE6_ROBOT_WITH_GRIPPER_SUBDIR = "robot_with_gripper"

_LITE6_BASE_FRAME_NAME = "link_base"
_LITE6_EEF_TIP_FRAME_NAME = "link_eef_tip"

_VARIANT_TO_DESCRIPTION_FILENAME: dict[Lite6Variant, str] = {
    Lite6Variant.VACUUM_GRIPPER: "lite6_robot_with_vacuum_gripper.urdf",
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: "lite6_robot_with_actuated_normal_parallel_gripper.urdf",
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: "lite6_robot_with_actuated_reverse_parallel_gripper.urdf",
}

# MultibodyPlant position counts. The vacuum gripper has no actuated
# DOFs of its own, so its plant has just the 6 arm joints. The actuated
# parallel grippers add two prismatic joints.
_VARIANT_TO_NUM_POSITIONS: dict[Lite6Variant, int] = {
    Lite6Variant.VACUUM_GRIPPER: LITE6_ARM_DOF,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_DOF,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_DOF,
}

# EEF generalized-DOF counts surfaced through the IManipulatorModel
# interface (i.e. the size of EEFPositions / EEFVelocities vectors for
# this variant). The vacuum gripper exposes a single binary on/off
# state; both parallel-gripper variants expose two prismatic joints.
_VARIANT_TO_NUM_EEF_DOFS: dict[Lite6Variant, int] = {
    Lite6Variant.VACUUM_GRIPPER: 1,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_PARALLEL_GRIPPER_DOF,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_PARALLEL_GRIPPER_DOF,
}


@attr.frozen
class Lite6Model(IManipulatorModel):
    """
    Lite6 model description selected by ``variant``.
    """

    variant: Lite6Variant

    @override
    def get_manipulator_type(self) -> ManipulatorType:
        return self.variant.get_manipulator_type()

    @override
    def get_variant(self) -> Lite6Variant:
        return self.variant

    @override
    def get_num_dof(self) -> int:
        return LITE6_ARM_DOF

    @override
    def get_num_eef_dofs(self) -> int:
        return _VARIANT_TO_NUM_EEF_DOFS[self.variant]

    @override
    def get_description_filepath(self) -> FilePath:
        return os.path.join(
            get_robot_models_directory_path(),
            _LITE6_DESCRIPTION_DIRNAME,
            ROBOT_MODELS_DRAKE_URDF_DIRNAME,
            _LITE6_ROBOT_WITH_GRIPPER_SUBDIR,
            _VARIANT_TO_DESCRIPTION_FILENAME[self.variant],
        )

    @override
    def get_base_frame_name(self) -> str:
        return _LITE6_BASE_FRAME_NAME

    @override
    def get_eef_tip_frame_name(self) -> str:
        return _LITE6_EEF_TIP_FRAME_NAME

    @override
    def get_num_positions(self) -> int:
        return _VARIANT_TO_NUM_POSITIONS[self.variant]

    @override
    def get_num_velocities(self) -> int:
        # All Lite6 joints (arm + parallel gripper) are simple revolute /
        # prismatic, so num_velocities equals num_positions.
        return _VARIANT_TO_NUM_POSITIONS[self.variant]

    @override
    def get_num_states(self) -> int:
        return self.get_num_positions() + self.get_num_velocities()
