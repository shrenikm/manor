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
from manor.common.custom_types import (
    EEPositionsVector,
    EEVelocitiesVector,
    FilePath,
    JointPositionsVector,
    PlantEEPositionsVector,
    PlantEEVelocitiesVector,
)
from manor.common.model_utils import ROBOT_MODELS_DRAKE_URDF_DIRNAME, get_robot_models_directory_path
from manor.manipulators.lite6.joint_configurations import Lite6JointConfiguration
from manor.manipulators.lite6.variant import Lite6Variant
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType

LITE6_ARM_DOF = 6
# Number of EE joints in the Drake plant (URDF) for the parallel
# gripper variants: two prismatic finger joints with opposite signed
# travel (left in [0, +0.008], right in [-0.008, 0]).
LITE6_PARALLEL_GRIPPER_PLANT_DOF = 2
# Number of EE-level DOFs surfaced through IManipulatorModel for the
# parallel gripper variants. The EE vector is the opening width (one
# scalar) -- one EE DOF, not two.
LITE6_PARALLEL_GRIPPER_EE_DOF = 1

# Joint travel per finger (URDF axis range), shared across the normal
# and reverse parallel-gripper variants. 8 mm per finger; both fingers
# combined cover 16 mm of joint-space travel.
_LITE6_PARALLEL_FINGER_HALF_TRAVEL_M: float = 0.008

# URDF link-origin offset of each finger from the gripper centre line
# at q = 0. The two parallel-gripper variants differ ONLY in this
# constant: in normal mounting the finger heads point toward each
# other (small offset, jaws nearly touch at q=0); in reverse mounting
# the finger heads are unscrewed and flipped, mounting outward (larger
# offset, jaws have a built-in 27 mm gap at q=0). The joint travel is
# identical, so reverse mode reaches a wider physical opening.
#
# The EE-level "opening width" we surface to policies / controllers is
# the PHYSICAL jaw gap, which equals (left - right) + 2 * origin_offset.
# That makes width semantics consistent across the two mountings: the
# user always commands a real-world distance, and the per-variant
# offset table absorbs the URDF geometry difference.
_LITE6_NP_FINGER_LINK_ORIGIN_OFFSET_M: float = 0.0025
_LITE6_RP_FINGER_LINK_ORIGIN_OFFSET_M: float = 0.0135

_VARIANT_TO_FINGER_LINK_ORIGIN_OFFSET_M: dict[Lite6Variant, float] = {
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: _LITE6_NP_FINGER_LINK_ORIGIN_OFFSET_M,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: _LITE6_RP_FINGER_LINK_ORIGIN_OFFSET_M,
}

# Physical jaw-gap width range per variant. Closed = q at (0, 0)
# (smallest physical gap reachable in this mounting); open = q at
# (+travel, -travel) (largest gap). The numeric values track the URDF
# link-origin offsets above.
LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M: float = 2.0 * _LITE6_NP_FINGER_LINK_ORIGIN_OFFSET_M
LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M: float = (
    LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M + 2.0 * _LITE6_PARALLEL_FINGER_HALF_TRAVEL_M
)
LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M: float = 2.0 * _LITE6_RP_FINGER_LINK_ORIGIN_OFFSET_M
LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M: float = (
    LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M + 2.0 * _LITE6_PARALLEL_FINGER_HALF_TRAVEL_M
)

# Per-variant EE-position limits surfaced through
# IManipulatorModel.get_ee_position_limits. Lower is the smaller
# physical-width bound for the parallel-gripper variants and the
# "off" end of the binary range for vacuum; upper is the larger.
# Open / closed semantics are NOT encoded here -- those are
# policy-side conventions.
_VARIANT_TO_LOWER_EE_LIMIT: dict[Lite6Variant, float] = {
    Lite6Variant.VACUUM_GRIPPER: 0.0,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_NP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_RP_PARALLEL_GRIPPER_CLOSED_WIDTH_M,
}
_VARIANT_TO_UPPER_EE_LIMIT: dict[Lite6Variant, float] = {
    Lite6Variant.VACUUM_GRIPPER: 1.0,
    Lite6Variant.PARALLEL_GRIPPER_NORMAL: LITE6_NP_PARALLEL_GRIPPER_OPEN_WIDTH_M,
    Lite6Variant.PARALLEL_GRIPPER_REVERSE: LITE6_RP_PARALLEL_GRIPPER_OPEN_WIDTH_M,
}

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
    def get_ee_position_limits(self) -> tuple[EEPositionsVector, EEPositionsVector]:
        lower = np.array([_VARIANT_TO_LOWER_EE_LIMIT[self.variant]], dtype=np.float64)
        upper = np.array([_VARIANT_TO_UPPER_EE_LIMIT[self.variant]], dtype=np.float64)
        return lower, upper

    @override
    def ee_positions_to_plant_positions(self, ee_positions: EEPositionsVector) -> PlantEEPositionsVector:
        # Vacuum: the URDF has no actuated EE joints, so the EE binary
        # on/off lives entirely in the proprioception channel and never
        # touches the plant's q vector.
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return np.zeros(0, dtype=np.float64)
        # Parallel gripper: the EE vector is the single physical jaw
        # opening width in metres. Translate to URDF q space by
        # subtracting the variant-specific built-in gap (the URDF link
        # origin offset doubled across the two fingers), then split the
        # remaining travel symmetrically across the two prismatic
        # joints. See the module-level constants for the per-variant
        # offset and the URDF link-origin provenance.
        if ee_positions.shape != (LITE6_PARALLEL_GRIPPER_EE_DOF,):
            raise ValueError(
                f"Lite6 parallel gripper expects ee_positions of shape ({LITE6_PARALLEL_GRIPPER_EE_DOF},); "
                f"got {ee_positions.shape}"
            )
        offset = _VARIANT_TO_FINGER_LINK_ORIGIN_OFFSET_M[self.variant]
        half_q = (float(ee_positions[0]) - 2.0 * offset) / 2.0
        return np.array([+half_q, -half_q], dtype=np.float64)

    @override
    def plant_positions_to_ee_positions(self, plant_ee_positions: PlantEEPositionsVector) -> EEPositionsVector:
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
        # Physical width = (left - right) + 2 * origin_offset. The
        # second term is the constant gap baked into the URDF geometry
        # (and the only thing that distinguishes normal from reverse).
        offset = _VARIANT_TO_FINGER_LINK_ORIGIN_OFFSET_M[self.variant]
        width = float(plant_ee_positions[0] - plant_ee_positions[1]) + 2.0 * offset
        return np.array([width], dtype=np.float64)

    @override
    def ee_velocities_to_plant_velocities(self, ee_velocities: EEVelocitiesVector) -> PlantEEVelocitiesVector:
        # Vacuum: no actuated EE joints in the plant.
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return np.zeros(0, dtype=np.float64)
        # Parallel gripper: the URDF-origin offset is constant, so it
        # drops out of the time derivative -- this is pure
        # half-and-mirror, identical for normal and reverse mountings.
        if ee_velocities.shape != (LITE6_PARALLEL_GRIPPER_EE_DOF,):
            raise ValueError(
                f"Lite6 parallel gripper expects ee_velocities of shape ({LITE6_PARALLEL_GRIPPER_EE_DOF},); "
                f"got {ee_velocities.shape}"
            )
        half_v = float(ee_velocities[0]) / 2.0
        return np.array([+half_v, -half_v], dtype=np.float64)

    @override
    def plant_velocities_to_ee_velocities(self, plant_ee_velocities: PlantEEVelocitiesVector) -> EEVelocitiesVector:
        if self.variant is Lite6Variant.VACUUM_GRIPPER:
            return np.zeros(1, dtype=np.float64)
        if plant_ee_velocities.shape != (LITE6_PARALLEL_GRIPPER_PLANT_DOF,):
            raise ValueError(
                f"Lite6 parallel gripper expects plant_ee_velocities of shape "
                f"({LITE6_PARALLEL_GRIPPER_PLANT_DOF},); got {plant_ee_velocities.shape}"
            )
        return np.array([float(plant_ee_velocities[0] - plant_ee_velocities[1])], dtype=np.float64)

    @override
    def get_prime_plant_positions(self) -> JointPositionsVector:
        return self._plant_positions_at(Lite6JointConfiguration.PRIME)

    @override
    def get_rest_plant_positions(self) -> JointPositionsVector:
        return self._plant_positions_at(Lite6JointConfiguration.REST)

    def _plant_positions_at(self, configuration: Lite6JointConfiguration) -> JointPositionsVector:
        # Arm joints come from the named configuration; the EE block is the URDF q neutral state
        # (zeros). For the parallel-gripper variants the two prismatic fingers at q=0 sit at the
        # URDF link-origin offset, which is the variant's "closed" width -- a reasonable default
        # to start in. Vacuum variants have no actuated EE joints, so the EE block is empty.
        arm_q = configuration.get_joint_positions_vector()
        ee_q = np.zeros(self.get_num_positions() - LITE6_ARM_DOF, dtype=np.float64)
        return np.concatenate([arm_q, ee_q])

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
