"""
Concrete IManipulatorModel for the Seeed reBot B601 DM.

The arm has 6 revolute joints driven by Damiao DM motors plus an actuated parallel gripper whose two
prismatic finger joints travel outward symmetrically. The gripper fingertips touch exactly at the URDF
zero state (q = 0 is fully closed with zero jaw gap), so the EE-level opening width maps to the plant's
finger joints without any origin-offset bookkeeping: width = q_left + q_right.
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
from manor.manipulators.manipulator_model import IManipulatorModel
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import register_manipulator_model
from manor.manipulators.rebot_b601_dm.joint_configurations import RebotB601DmJointConfiguration
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant

REBOT_B601_DM_ARM_DOF = 6
# Number of EE joints in the Drake plant (URDF): two prismatic finger joints, each travelling outward
# from the closed state in [0, +0.0715].
REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF = 2
# Number of EE-level DOFs surfaced through IManipulatorModel: the jaw opening width (one scalar).
REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF = 1

# Joint travel per finger from the URDF prismatic limits. Both fingers move outward with positive q, so
# the physical jaw width is simply the sum of the two joint values.
_REBOT_B601_DM_FINGER_TRAVEL_M: float = 0.0715

# Physical jaw-gap width range. The fingertips touch exactly at q = (0, 0) (measured from the finger
# meshes: the inner tip faces meet at the gripper centerline), so the closed width is zero and the open
# width is the combined travel of both fingers.
REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M: float = 0.0
REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M: float = 2.0 * _REBOT_B601_DM_FINGER_TRAVEL_M

_REBOT_B601_DM_DESCRIPTION_DIRNAME = "rebot_b601_dm_description"

_REBOT_B601_DM_BASE_FRAME_NAME = "base_link"
_REBOT_B601_DM_FK_IK_FRAME_NAME = "link_eef_tip"

_VARIANT_TO_DESCRIPTION_FILENAME: dict[RebotB601DmVariant, str] = {
    RebotB601DmVariant.PARALLEL_GRIPPER: "rebot_b601_dm_with_gripper.urdf",
}


@register_manipulator_model(ManipulatorType.REBOT_B601_DM)
@attr.frozen
class RebotB601DmModel(IManipulatorModel):
    """
    reBot B601 DM model description selected by variant.
    """

    variant: RebotB601DmVariant

    @override
    def get_manipulator_type(self) -> ManipulatorType:
        return self.variant.get_manipulator_type()

    @override
    def get_variant(self) -> RebotB601DmVariant:
        return self.variant

    @override
    def get_num_dof(self) -> int:
        return REBOT_B601_DM_ARM_DOF

    @override
    def get_num_ee_dofs(self) -> int:
        return REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF

    @override
    def get_description_filepath(self) -> FilePath:
        return os.path.join(
            get_robot_models_directory_path(),
            _REBOT_B601_DM_DESCRIPTION_DIRNAME,
            ROBOT_MODELS_DRAKE_URDF_DIRNAME,
            _VARIANT_TO_DESCRIPTION_FILENAME[self.variant],
        )

    @override
    def get_base_frame_name(self) -> str:
        return _REBOT_B601_DM_BASE_FRAME_NAME

    @override
    def get_fk_ik_frame_name(self) -> str:
        return _REBOT_B601_DM_FK_IK_FRAME_NAME

    @override
    def get_num_positions(self) -> int:
        return REBOT_B601_DM_ARM_DOF + REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF

    @override
    def get_num_velocities(self) -> int:
        # All joints (arm revolute + gripper prismatic) are simple 1-DOF joints, so num_velocities equals
        # num_positions.
        return self.get_num_positions()

    @override
    def get_num_states(self) -> int:
        return self.get_num_positions() + self.get_num_velocities()

    @override
    def get_ee_position_limits(self) -> tuple[EEPositionsVector, EEPositionsVector]:
        lower = np.array([REBOT_B601_DM_PARALLEL_GRIPPER_CLOSED_WIDTH_M], dtype=np.float64)
        upper = np.array([REBOT_B601_DM_PARALLEL_GRIPPER_OPEN_WIDTH_M], dtype=np.float64)
        return lower, upper

    @override
    def ee_positions_to_plant_positions(self, ee_positions: EEPositionsVector) -> PlantEEPositionsVector:
        # The EE vector is the physical jaw opening width in metres. Both prismatic joints open outward
        # with positive q and the fingertips meet at q = (0, 0), so the width splits evenly across the two
        # joints with no offset term.
        if ee_positions.shape != (REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF,):
            raise ValueError(
                f"reBot B601 DM parallel gripper expects ee_positions of shape "
                f"({REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF},); got {ee_positions.shape}"
            )
        half_q = float(ee_positions[0]) / 2.0
        return np.array([half_q, half_q], dtype=np.float64)

    @override
    def plant_positions_to_ee_positions(self, plant_ee_positions: PlantEEPositionsVector) -> EEPositionsVector:
        if plant_ee_positions.shape != (REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF,):
            raise ValueError(
                f"reBot B601 DM parallel gripper expects plant_ee_positions of shape "
                f"({REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF},); got {plant_ee_positions.shape}"
            )
        width = float(plant_ee_positions[0] + plant_ee_positions[1])
        return np.array([width], dtype=np.float64)

    @override
    def ee_velocities_to_plant_velocities(self, ee_velocities: EEVelocitiesVector) -> PlantEEVelocitiesVector:
        if ee_velocities.shape != (REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF,):
            raise ValueError(
                f"reBot B601 DM parallel gripper expects ee_velocities of shape "
                f"({REBOT_B601_DM_PARALLEL_GRIPPER_EE_DOF},); got {ee_velocities.shape}"
            )
        half_v = float(ee_velocities[0]) / 2.0
        return np.array([half_v, half_v], dtype=np.float64)

    @override
    def plant_velocities_to_ee_velocities(self, plant_ee_velocities: PlantEEVelocitiesVector) -> EEVelocitiesVector:
        if plant_ee_velocities.shape != (REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF,):
            raise ValueError(
                f"reBot B601 DM parallel gripper expects plant_ee_velocities of shape "
                f"({REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF},); got {plant_ee_velocities.shape}"
            )
        return np.array([float(plant_ee_velocities[0] + plant_ee_velocities[1])], dtype=np.float64)

    def _plant_positions_at(self, configuration: RebotB601DmJointConfiguration) -> JointPositionsVector:
        # Arm joints come from the named configuration; the EE block is the URDF q neutral state (zeros),
        # which is the fully closed gripper.
        arm_q = configuration.get_joint_positions_vector()
        ee_q = np.zeros(REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF, dtype=np.float64)
        return np.concatenate([arm_q, ee_q])

    @override
    def get_prime_plant_positions(self) -> JointPositionsVector:
        return self._plant_positions_at(RebotB601DmJointConfiguration.PRIME)

    @override
    def get_rest_plant_positions(self) -> JointPositionsVector:
        return self._plant_positions_at(RebotB601DmJointConfiguration.REST)

    @override
    def get_default_sim_pid_gains(self) -> PIDGains:
        # Initial gains in the same regime as the Lite6 defaults, scaled down on kd for the lighter distal
        # joints. These have NOT been tuned against sim rollouts yet; treat them as a starting point when
        # the aegis sim backend comes online for this arm.
        arm_kp = [100.0] * REBOT_B601_DM_ARM_DOF
        arm_ki = [0.0] * REBOT_B601_DM_ARM_DOF
        arm_kd = [50.0, 50.0, 50.0, 25.0, 25.0, 25.0]
        gripper_kp = [500.0] * REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF
        gripper_ki = [50.0] * REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF
        gripper_kd = [500.0] * REBOT_B601_DM_PARALLEL_GRIPPER_PLANT_DOF
        return PIDGains(
            kp=np.array(arm_kp + gripper_kp, dtype=np.float64),
            ki=np.array(arm_ki + gripper_ki, dtype=np.float64),
            kd=np.array(arm_kd + gripper_kd, dtype=np.float64),
        )
