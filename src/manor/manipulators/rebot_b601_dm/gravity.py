"""
Drake gravity-torque model for the reBot B601 DM arm.

Gravity compensation feeds the per-joint static hold torque g(q) as the MIT tau feedforward so the motor
holds the arm against gravity without the position loop having to carry the load: the MIT torque becomes
kp * (p_des - p) + kd * (v_des - v) + g(q). With gravity cancelled the position gains can be soft and the
arm floats / backdrives while still holding its pose.

g(q) here is -CalcGravityGeneralizedForces on the corrected description (the arm inertials were fixed to
the vendor's own gravity-comp model; see the description package README), and is validated to match the
vendor's pinocchio gravity comp to machine precision.

Two facts make the feedforward a direct pass-through with no rescaling: the motor position frame matches
the URDF joint frame (bring-up streams URDF configuration vectors straight to the motors and they reach
the commanded pose), and the DM MIT tau is an output-side joint torque in N*m (the vendor feeds pinocchio
g(q) into it directly). A tau_scale below 1 at the call site absorbs geartrain friction and residual model
error.
"""

from __future__ import annotations

from functools import cached_property

import attr
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context

from manor.common.custom_types import JointPositionsVector
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.rebot_b601_dm.model import REBOT_B601_DM_ARM_DOF, RebotB601DmModel
from manor.manipulators.rebot_b601_dm.variant import RebotB601DmVariant

_ARM_JOINT_NAMES: tuple[str, ...] = tuple(f"joint{i}" for i in range(1, REBOT_B601_DM_ARM_DOF + 1))


@attr.frozen
class _GravityPlant:
    """
    The built Drake plant with a reusable context and the per-arm-joint position / velocity indices into
    the plant's generalized coordinates.
    """

    plant: MultibodyPlant
    context: Context
    position_indices: tuple[int, ...]
    velocity_indices: tuple[int, ...]


@attr.frozen
class RebotB601DmGravityModel:
    """
    Computes the arm gravity hold torque g(q) from the Drake description, for use as the MIT tau
    feedforward. The heavy plant is built once, lazily, on first use.
    """

    variant: RebotB601DmVariant = RebotB601DmVariant.PARALLEL_GRIPPER

    @cached_property
    def _gravity_plant(self) -> _GravityPlant:
        model = RebotB601DmModel(variant=self.variant)
        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        add_robot_models_to_package_map(parser.package_map())
        parser.AddModels(model.get_description_filepath())
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(model.get_base_frame_name()))
        plant.Finalize()
        joints = [plant.GetJointByName(name) for name in _ARM_JOINT_NAMES]
        return _GravityPlant(
            plant=plant,
            context=plant.CreateDefaultContext(),
            position_indices=tuple(joint.position_start() for joint in joints),
            velocity_indices=tuple(joint.velocity_start() for joint in joints),
        )

    def hold_torque(self, arm_positions: JointPositionsVector) -> np.ndarray:
        """
        The per-joint static hold torque g(q) for the six arm joints at arm_positions, in output-side N*m,
        computed as -CalcGravityGeneralizedForces with the gripper left at its neutral (closed) state. Feed
        this straight into the MIT tau feedforward to cancel gravity.
        """
        gravity_plant = self._gravity_plant
        plant, context = gravity_plant.plant, gravity_plant.context
        positions = plant.GetPositions(context).copy()
        for index, value in zip(gravity_plant.position_indices, arm_positions, strict=True):
            positions[index] = float(value)
        plant.SetPositions(context, positions)
        gravity_forces = plant.CalcGravityGeneralizedForces(context)
        return -np.array([gravity_forces[index] for index in gravity_plant.velocity_indices], dtype=np.float64)

    def warm(self) -> None:
        """
        Force the lazy plant build now, before the streamer starts, so the first control tick is not stalled
        by parsing the URDF and finalizing the plant.
        """
        _ = self.hold_torque(np.zeros(REBOT_B601_DM_ARM_DOF, dtype=np.float64))
