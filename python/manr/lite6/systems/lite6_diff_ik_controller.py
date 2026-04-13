import numpy as np
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsStatus,
    DoDifferentialInverseKinematics,
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from manr.common.exceptions import Lite6SystemError
from manr.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ModelGroups,
    Lite6ModelType,
    get_default_lite6_joint_positions_vector,
    get_lite6_urdf_eef_tip_frame_name,
)

IK_PE_IP_NAME = "pe_input"
IK_VE_IP_NAME = "ve_input"
IK_GVD_IP_NAME = "gvd_input"
IK_VD_OP_NAME = "vd_output"


class Lite6DiffIKController(LeafSystem):
    def __init__(
        self,
        lite6_model_type: Lite6ModelType,
        plant: MultibodyPlant,
    ):
        # Only supports unactuated gripper models
        if lite6_model_type not in Lite6ModelGroups.LITE6_ROBOT_WITH_UNACTUATED_GRIPPER_MODELS:
            raise Lite6SystemError("Invalid lite6 model type.")

        super().__init__()

        self._plant = plant

        # Only non actuated models are allowed to be used here.
        assert plant.num_positions() == LITE6_DOF
        assert plant.num_velocities() == LITE6_DOF
        assert plant.num_actuators() == LITE6_DOF

        self._context = plant.CreateDefaultContext()
        self._eef_tip_frame = plant.GetBodyByName(
            name=get_lite6_urdf_eef_tip_frame_name(
                lite6_model_type=lite6_model_type,
            ),
        ).body_frame()

        self._diff_ik_parameters = DifferentialInverseKinematicsParameters(
            num_positions=plant.num_positions(),
            num_velocities=plant.num_velocities(),
        )
        self._diff_ik_parameters.set_nominal_joint_position(
            get_default_lite6_joint_positions_vector(
                lite6_model_type=lite6_model_type,
            ),
        )
        self._diff_ik_parameters.set_time_step(dt=plant.time_step())
        self._diff_ik_parameters.set_joint_position_limits(
            (plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()),
        )
        self._diff_ik_parameters.set_joint_velocity_limits(
            (plant.GetVelocityLowerLimits(), plant.GetVelocityUpperLimits()),
        )
        self._diff_ik_parameters.set_joint_acceleration_limits(
            (plant.GetAccelerationLowerLimits(), plant.GetAccelerationUpperLimits()),
        )

        # Input ports.
        self.ik_pe_input_port = self.DeclareVectorInputPort(
            name=IK_PE_IP_NAME,
            size=LITE6_DOF,
        )
        self.ik_ve_input_port = self.DeclareVectorInputPort(
            name=IK_VE_IP_NAME,
            size=LITE6_DOF,
        )
        self.ik_gvd_input_port = self.DeclareVectorInputPort(
            name=IK_GVD_IP_NAME,
            size=LITE6_DOF,
        )

        self.ik_vd_output_port = self.DeclareVectorOutputPort(
            name=IK_VD_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_diff_ik_velocities_output,
        )

    def _compute_diff_ik_velocities_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        joint_q = self.ik_pe_input_port.Eval(context)
        joint_v = self.ik_ve_input_port.Eval(context)
        gripper_V = self.ik_gvd_input_port.Eval(context)

        self._plant.SetPositions(
            self._context,
            joint_q,
        )
        jacobian = self._plant.CalcJacobianSpatialVelocity(
            context=self._context,
            with_respect_to=JacobianWrtVariable.kV,
            frame_B=self._eef_tip_frame,
            p_BoBp_B=np.zeros(3, dtype=np.float64),
            frame_A=self._plant.world_frame(),
            frame_E=self._plant.world_frame(),
        )

        diff_ik_result = DoDifferentialInverseKinematics(
            joint_q,
            joint_v,
            gripper_V,
            jacobian,
            self._diff_ik_parameters,
        )

        if diff_ik_result.status != DifferentialInverseKinematicsStatus.kSolutionFound:
            raise Lite6SystemError(f"Diff IK failed! status: {diff_ik_result.status}")
        output_vector.SetFromVector(
            value=diff_ik_result.joint_velocities,
        )
