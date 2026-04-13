from enum import Enum, auto
from typing import Optional, Sequence

import attr
import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, Context, Diagram, LeafSystem

from manr.common.control.constructs import PIDGains
from manr.common.model_utils import ObjectModelConfig
from manr.lite6.utils.lite6_model_utils import (
    LITE6_DOF,
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    add_gripper_positions_and_velocities_to_lite6_state_vector,
    create_lite6_state_vector,
    get_gripper_positions_from_lite6_state_vector,
    get_gripper_status_from_lite6_state_vector,
    get_gripper_velocities_from_lite6_state_vector,
    get_joint_positions_vector_from_lite6_state_vector,
    get_joint_velocities_vector_from_lite6_state_vector,
    get_lite6_num_states,
)

LITE6_PLIANT_DEFAULT_MESHCAT_PORT = 7000

LITE6_PLIANT_SUPPORTED_MODEL_TYPES = (
    Lite6ModelType.ROBOT_WITH_ANP_GRIPPER,
    Lite6ModelType.ROBOT_WITH_ARP_GRIPPER,
)

LITE6_PLIANT_NAME = "Lite6Pliant"

# Positions desired
LITE6_PLIANT_PD_IP_NAME = "pd_input"
# Velocities desired
LITE6_PLIANT_VD_IP_NAME = "vd_input"
# Gripper status desired
LITE6_PLIANT_GSD_IP_NAME = "gsd_input"
# State estimated
LITE6_PLIANT_SE_IP_NAME = "se_input"

LITE6_PLIANT_PD_OP_NAME = "pd_output"
LITE6_PLIANT_VD_OP_NAME = "vd_output"
LITE6_PLIANT_GSD_OP_NAME = "gsd_output"

LITE6_PLIANT_SD_OP_NAME = "sd_output"
LITE6_PLIANT_PE_OP_NAME = "pe_output"
LITE6_PLIANT_VE_OP_NAME = "ve_output"
LITE6_PLIANT_GSE_OP_NAME = "gse_output"

LITE6_PLIANT_MULTIPLEXER_PREFIX = "mult_"
LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_PD_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VD_IP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GSD_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_SE_IP_NAME
)

LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_SD_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_PE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_PE_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_VE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_VE_OP_NAME
)
LITE6_PLIANT_MULTIPLEXER_GSE_OP_NAME = (
    LITE6_PLIANT_MULTIPLEXER_PREFIX + LITE6_PLIANT_GSE_OP_NAME
)


class Lite6PliantType(Enum):
    """
    Simulation or Hardware pliant type.
    """

    SIMULATION = auto()
    HARDWARE = auto()


@attr.frozen
class Lite6PliantConfig:
    lite6_model_type: Lite6ModelType
    lite6_control_type: Lite6ControlType
    lite6_pliant_type: Lite6PliantType
    inverse_dynamics_pid_gains: PIDGains
    plant_config: MultibodyPlantConfig
    # The estimation and command time step while running on actual hardware.
    hardware_control_loop_time_step: float
    object_model_configs: Optional[Sequence[ObjectModelConfig]] = None

    def get_name(self) -> str:
        return f"{LITE6_PLIANT_NAME}{self.lite6_pliant_type.name.title()}"


def get_tuned_pid_gains_for_pliant_id_controller(
    lite6_control_type: Lite6ControlType,
) -> PIDGains:
    if lite6_control_type == Lite6ControlType.VELOCITY:
        # Note that because of how velocity control is implemented in the pliant,
        # only the kd gains matter. This is because in the ID controller, desired
        # positions is set to estimated positions, and so this difference is always
        # zero.
        # The joint gains have been tuned using the analysis plots of the choreographer. (See lite6_pliant_analysis.py)
        # The gripper gains have been tuned for them to open/close as fast as the real robot (~200-250 ms)
        return PIDGains(
            kp=np.array(
                [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 500.0, 500.0],
                dtype=np.float64,
            ),
            ki=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 50.0], dtype=np.float64),
            kd=np.array(
                [50.0, 50.0, 50.0, 75.0, 75.0, 75.0, 500.0, 500.0], dtype=np.float64
            ),
        )
    else:
        # Has not been tuned for other control types.
        raise NotImplementedError(
            f"ID PID gains have not been tuned for control type {lite6_control_type}"
        )


def create_simulator_for_lite6_pliant(
    config: Lite6PliantConfig,
    diagram: Diagram,
) -> Simulator:
    """
    Note that diagram here isn't the diagram of the pliant, but rather the
    diagram of the entire setup including external controllers, planners, etc.
    """
    simulator = Simulator(diagram)
    # Set the rate to 1. for hardware testing on the actual robot.
    if config.lite6_pliant_type == Lite6PliantType.HARDWARE:
        simulator.set_target_realtime_rate(realtime_rate=1.0)

    # TODO: Optionally set other simulator config options here.

    return simulator


class Lite6PliantMultiplexer(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self.num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )
        self.pd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PD_IP_NAME,
            size=LITE6_DOF,
        )
        self.vd_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VD_IP_NAME,
            size=LITE6_DOF,
        )
        self.gsd_input_port = self.DeclareAbstractInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GSD_IP_NAME,
            model_value=Value(Lite6GripperStatus.CLOSED),
        )

        self.sd_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SD_OP_NAME,
            size=self.num_total_states,
            calc=self._compute_state_desired_output,
        )

    def _compute_state_desired_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_output_vector = np.zeros(self.num_total_states, dtype=np.float64)

        state_estimated_vector = self.se_input_port.Eval(context)
        positions_desired_vector = self.pd_input_port.Eval(context)
        velocities_desired_vector = self.vd_input_port.Eval(context)
        gripper_status_desired = self.gsd_input_port.Eval(context)

        if self.config.lite6_control_type == Lite6ControlType.STATE:
            # For full state control, we need to route the desired positions and
            # velocities into the non gripper state values of the output.
            # The gripper values are set according to the desired flag. Desired gripper
            # velocity will be 0 with the positions being open/closed according to the
            # GSD port input.
            state_output_vector = create_lite6_state_vector(
                lite6_model_type=self.config.lite6_model_type,
                joint_positions_vector=positions_desired_vector,
                joint_velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        elif self.config.lite6_control_type == Lite6ControlType.VELOCITY:
            # For velocity control, we route the desired velocities into the output
            # state's velocities. The output's positions are obtained from the input
            # estimated positions state. This is the only case where we use the SE input
            # port value.
            positions_estimated_vector = get_joint_positions_vector_from_lite6_state_vector(
                lite6_model_type=self.config.lite6_model_type,
                state_vector=state_estimated_vector,
            )
            state_output_vector = create_lite6_state_vector(
                lite6_model_type=self.config.lite6_model_type,
                joint_positions_vector=positions_estimated_vector,
                joint_velocities_vector=velocities_desired_vector,
                lite6_gripper_status=gripper_status_desired,
            )
        else:
            raise NotImplementedError

        # If the desired gripper position is neutral, we need to avoid actuating
        # the parallel gripper joints. To do this, we set the output state of the
        # gripper, the same as the estimated state so that the inverse dynamics
        # controller will cancel them out.
        if gripper_status_desired == Lite6GripperStatus.NEUTRAL:
            state_output_vector = add_gripper_positions_and_velocities_to_lite6_state_vector(
                lite6_model_type=self.config.lite6_model_type,
                state_vector=state_output_vector,
                gripper_positions=get_gripper_positions_from_lite6_state_vector(
                    lite6_model_type=self.config.lite6_model_type,
                    state_vector=state_estimated_vector,
                ),
                gripper_velocities=get_gripper_velocities_from_lite6_state_vector(
                    lite6_model_type=self.config.lite6_model_type,
                    state_vector=state_estimated_vector,
                ),
            )

        output_vector.SetFromVector(
            value=state_output_vector,
        )


class Lite6PliantDeMultiplexer(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
    ):
        super().__init__()

        self.config = config
        self.num_total_states = get_lite6_num_states(
            lite6_model_type=config.lite6_model_type,
        )

        self.se_input_port = self.DeclareVectorInputPort(
            name=LITE6_PLIANT_MULTIPLEXER_SE_IP_NAME,
            size=self.num_total_states,
        )

        self.pe_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_PE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_positions_estimated_output,
        )
        self.ve_output_port = self.DeclareVectorOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_VE_OP_NAME,
            size=LITE6_DOF,
            calc=self._compute_velocities_estimated_output,
        )
        self.gse_output_port = self.DeclareAbstractOutputPort(
            name=LITE6_PLIANT_MULTIPLEXER_GSE_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_estimated_output,
        )

    def _compute_positions_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        positions_output_vector = get_joint_positions_vector_from_lite6_state_vector(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=positions_output_vector,
        )

    def _compute_velocities_estimated_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        velocities_output_vector = get_joint_velocities_vector_from_lite6_state_vector(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )
        output_vector.SetFromVector(
            value=velocities_output_vector,
        )

    def _compute_gripper_status_estimated_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:
        state_estimated_vector = self.se_input_port.Eval(context)

        lite6_gripper_status = get_gripper_status_from_lite6_state_vector(
            lite6_model_type=self.config.lite6_model_type,
            state_vector=state_estimated_vector,
        )

        output_value.set_value(lite6_gripper_status)
