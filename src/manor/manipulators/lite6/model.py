"""
Concrete ``IManipulatorModel`` for the Ufactory Lite6.

Per-variant data (description filename, MultibodyPlant position count)
lives in module-level dicts keyed by ``Lite6Variant``. The arm DOF count
and frame names are constant across all variants: the Lite6 arm always
has 6 joints, and every supported variant uses ``link_base`` as the
base frame and ``link_eef_tip`` as the FK / IK reference frame.
"""

from __future__ import annotations

import os
from typing import override

import attr
import numpy as np

from manor.common.control.pid import PIDGains
from manor.common.custom_types import FilePath
from manor.common.model_utils import ROBOT_MODELS_DRAKE_URDF_DIRNAME, get_robot_models_directory_path
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType

LITE6_ARM_DOF = 6
# Number of EE joints in the Drake plant (URDF) for the parallel
# gripper variants: two prismatic finger joints with opposite signed
# travel (left in [0, +0.008], right in [-0.008, 0]; total opening =
# left - right).
LITE6_PARALLEL_GRIPPER_PLANT_DOF = 2
# Number of EE-level DOFs surfaced through IManipulatorModel for the
# parallel gripper variants. The EE vector is the opening width (one
# scalar) -- one EE DOF, not two.
LITE6_PARALLEL_GRIPPER_EE_DOF = 1

# Lite6 parallel-gripper finger joint travel from the URDF, confirmed
# against the deprecated codebase's hardware measurements
# (deprecated_lite6/utils/lite6_model_utils.py:LITE6_*_GRIPPER_*_POSITIONS).
# Each finger has 8 mm of travel along the gripper centre line; total
# opening width when fully open is 16 mm, with closed = 0. The signs
# are opposite by URDF axis convention.
_LITE6_PARALLEL_FINGER_HALF_TRAVEL_M: float = 0.008
LITE6_PARALLEL_GRIPPER_OPEN_WIDTH_M: float = 2.0 * _LITE6_PARALLEL_FINGER_HALF_TRAVEL_M
LITE6_PARALLEL_GRIPPER_CLOSED_WIDTH_M: float = 0.0

_LITE6_DESCRIPTION_DIRNAME = "lite6_description"
_LITE6_ROBOT_WITH_GRIPPER_SUBDIR = "robot_with_gripper"

_LITE6_BASE_FRAME_NAME = "link_base"
_LITE6_FK_IK_FRAME_NAME = "link_eef_tip"

_VARIANT_TO_DESCRIPTION_FILENAME: dict[Lite6Variant, str] = {
    Lite6Variant.VACUUM_GRIPPER: "lite6_robot_with_vacuum_gripper.urdf",
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: "lite6_robot_with_actuated_normal_parallel_gripper.urdf",
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: "lite6_robot_with_actuated_reverse_parallel_gripper.urdf",
}

# MultibodyPlant position counts. The vacuum gripper has no actuated
# DOFs of its own, so its plant has just the 6 arm joints. The actuated
# parallel grippers add two prismatic finger joints.
_VARIANT_TO_NUM_POSITIONS: dict[Lite6Variant, int] = {
    Lite6Variant.VACUUM_GRIPPER: LITE6_ARM_DOF,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_PLANT_DOF,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_ARM_DOF + LITE6_PARALLEL_GRIPPER_PLANT_DOF,
}

# EE generalized-DOF counts surfaced through the IManipulatorModel
# interface (i.e. the size of EEPositions / EEVelocities vectors for
# this variant). The vacuum gripper exposes a single binary on/off
# state; both parallel-gripper variants expose a single opening width
# that the model splits internally across the two prismatic finger
# joints in the URDF.
_VARIANT_TO_NUM_EE_DOFS: dict[Lite6Variant, int] = {
    Lite6Variant.VACUUM_GRIPPER: 1,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_PARALLEL_GRIPPER_EE_DOF,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_PARALLEL_GRIPPER_EE_DOF,
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
    def get_num_ee_dofs(self) -> int:
        return _VARIANT_TO_NUM_EE_DOFS[self.variant]

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
    def get_fk_ik_frame_name(self) -> str:
        return _LITE6_FK_IK_FRAME_NAME

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

    @override
    def ee_positions_to_plant_positions(self, ee_positions: np.ndarray) -> np.ndarray:
        # Vacuum: the URDF has no actuated EE joints, so the EE binary
        # on/off lives entirely in the proprioception channel and never
        # touches the plant's q vector.
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return np.zeros(0, dtype=np.float64)
        # Parallel gripper: the EE vector is the single opening width.
        # The URDF's two prismatic finger joints travel symmetrically
        # in opposite signs about the gripper centre line, so width w
        # maps to (left = +w/2, right = -w/2). See the URDF axis limits
        # ([0, +0.008] / [-0.008, 0]) and the calibration constants at
        # the top of this module for provenance.
        if ee_positions.shape != (LITE6_PARALLEL_GRIPPER_EE_DOF,):
            raise ValueError(
                f"Lite6 parallel gripper expects ee_positions of shape ({LITE6_PARALLEL_GRIPPER_EE_DOF},); "
                f"got {ee_positions.shape}"
            )
        half_width = float(ee_positions[0]) / 2.0
        return np.array([+half_width, -half_width], dtype=np.float64)

    @override
    def plant_positions_to_ee_positions(self, plant_ee_positions: np.ndarray) -> np.ndarray:
        # Vacuum gripper has no actuated EE joints in the plant, so the
        # plant block is empty and there's no q to project from. With
        # no separate latch path here, the best the sim can report is
        # "off" (zero); the binary state is supplied by the EE-command
        # channel, not derived from physics.
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return np.zeros(1, dtype=np.float64)
        if plant_ee_positions.shape != (LITE6_PARALLEL_GRIPPER_PLANT_DOF,):
            raise ValueError(
                f"Lite6 parallel gripper expects plant_ee_positions of shape "
                f"({LITE6_PARALLEL_GRIPPER_PLANT_DOF},); got {plant_ee_positions.shape}"
            )
        # Opening width = left - right (since the right finger travels
        # in the negative direction); equivalently, the difference of
        # the two signed positions.
        width = float(plant_ee_positions[0] - plant_ee_positions[1])
        return np.array([width], dtype=np.float64)

    @override
    def get_default_sim_pid_gains(self) -> PIDGains:
        # Tuned in the previous (deprecated) Lite6 sim against the
        # choreographer analysis plots. Arm joints carry mid-range
        # gains; the parallel-gripper fingers run much stiffer (the
        # 5 g finger links accelerate fast under gravity without
        # damping declared on those prismatic joints).
        arm_kp = [100.0] * LITE6_ARM_DOF
        arm_ki = [0.0] * LITE6_ARM_DOF
        arm_kd = [50.0, 50.0, 50.0, 75.0, 75.0, 75.0]
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return PIDGains(
                kp=np.array(arm_kp, dtype=np.float64),
                ki=np.array(arm_ki, dtype=np.float64),
                kd=np.array(arm_kd, dtype=np.float64),
            )
        gripper_kp = [500.0] * LITE6_PARALLEL_GRIPPER_PLANT_DOF
        gripper_ki = [50.0] * LITE6_PARALLEL_GRIPPER_PLANT_DOF
        gripper_kd = [500.0] * LITE6_PARALLEL_GRIPPER_PLANT_DOF
        return PIDGains(
            kp=np.array(arm_kp + gripper_kp, dtype=np.float64),
            ki=np.array(arm_ki + gripper_ki, dtype=np.float64),
            kd=np.array(arm_kd + gripper_kd, dtype=np.float64),
        )
