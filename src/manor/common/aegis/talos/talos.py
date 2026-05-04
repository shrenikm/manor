"""
Talos LeafSystem: sends joint+ee commands to the robot and publishes its
full proprioception state.

On each periodic tick Talos:
  1. forwards the latest JointEECommand on its input port to the backend,
  2. polls the backend for the current joint + EE state,
  3. runs forward kinematics on the joint state to compute Cartesian pose + twist,
  4. assembles a Proprioception message and writes it into abstract state.

The output port is a zero-order hold of that state.

Talos owns its own ``MultibodyPlant`` (built from ``manipulator_model``)
for FK; this plant is independent of Gaia's physics plant and of
Kyber's IK plant -- same URDF, three independent instances.

The current FK implementation is a stub (identity pose, zero twist);
once the per-system plant is consulted properly, only
``_compute_cartesian_pose`` / ``_compute_cartesian_twist`` need to change.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, Self, runtime_checkable

import attr
import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackendConfig
from manor.common.aegis.talos.stale_command_watchdog import StaleCommandWatchdogConfig
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel


class TalosPorts(StrEnum):
    """
    Named input / output ports exposed by Talos.
    """

    INPUT_JOINT_EE_COMMAND = "joint_ee_command"
    OUTPUT_PROPRIOCEPTION = "proprioception"


@attr.frozen
class TalosConfig:
    """
    Talos sub-system configuration.

    sim_backend_config and hardware_backend_config parametrise the
    per-mode manipulator backends. stale_command_watchdog_config
    parametrises the hardware-only StaleCommandWatchdog LeafSystem
    (consumed by run_kylos; ignored in sim mode). SYSTEM_NAME is the
    name applied to the Talos LeafSystem in the diagram.
    """

    SYSTEM_NAME: ClassVar[str] = "talos"

    publish_frequency_hz: float = 200.0
    sim_backend_config: SimManipulatorBackendConfig = attr.field(factory=SimManipulatorBackendConfig)
    # HardwareManipulatorBackendConfig.minimum_watchdog_frequency_hz has no default, so this field
    # is required (kw_only keeps it after the defaulted fields above without breaking attrs's
    # default-ordering rule). Every aegis YAML must declare hardware_backend_config with the
    # watchdog frequency the operator has consciously chosen for the policy on the wire.
    hardware_backend_config: HardwareManipulatorBackendConfig = attr.field(kw_only=True)
    stale_command_watchdog_config: StaleCommandWatchdogConfig = attr.field(factory=StaleCommandWatchdogConfig)

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the talos_config: block of an aegis YAML.
        """
        return cls(**parse_attrs_yaml(cls, d, "talos_config"))


@runtime_checkable
class ManipulatorBackend(Protocol):
    """
    Protocol for a manipulator actuation-and-state interface.

    Exactly one backend owns the robot's actual state at a time. Talos
    drives the backend on every tick by calling send_joint_ee_command
    and reads state back via read_joint_state / read_ee_state. Forward
    kinematics is Talos's responsibility, not the backend's.

    The hardware backend additionally exposes pet_watchdog so a sibling
    LeafSystem (StaleCommandWatchdog) can keep its staleness timer
    fresh without going through Talos. This method is not part of the
    Protocol because not every backend implements it (sim doesn't need
    a watchdog), and Talos itself does not call it.
    """

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None: ...

    def read_joint_state(self) -> JointState: ...

    def read_ee_state(self) -> EEState: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...


class Talos(LeafSystem):
    """
    Bridge between the Aegis graph and the manipulator. Consumes
    JointEECommand, publishes Proprioception.
    """

    def __init__(
        self,
        backend: ManipulatorBackend,
        manipulator_model: IManipulatorModel,
        publish_frequency: float,
    ) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self.backend = backend
        self.manipulator_model = manipulator_model
        self.publish_frequency = publish_frequency
        self.plant = self._build_plant(manipulator_model)
        self._plant_context = self.plant.CreateDefaultContext()
        self._tip_frame = self.plant.GetFrameByName(manipulator_model.get_fk_ik_frame_name())
        self._world_frame = self.plant.world_frame()

        self._joint_ee_command_input = self.DeclareAbstractInputPort(
            TalosPorts.INPUT_JOINT_EE_COMMAND,
            AbstractValue.Make(JointEECommand.construct_default()),
        )

        self._proprioception_state_index = self.DeclareAbstractState(
            AbstractValue.Make(Proprioception.construct_default()),
        )

        self.DeclareAbstractOutputPort(
            TalosPorts.OUTPUT_PROPRIOCEPTION,
            alloc=lambda: AbstractValue.Make(Proprioception.construct_default()),
            calc=self._calc_proprioception_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._proprioception_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @staticmethod
    def _build_plant(manipulator_model: IManipulatorModel) -> MultibodyPlant:
        # Talos's plant is FK-only; no scene graph, no env. Gaia and
        # Kyber each maintain their own independent plants from the
        # same URDF.
        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        add_robot_models_to_package_map(parser.package_map())
        model_index = parser.AddModels(manipulator_model.get_description_filepath())[0]
        base_frame = plant.GetFrameByName(manipulator_model.get_base_frame_name(), model_index)
        plant.WeldFrames(plant.world_frame(), base_frame)
        plant.Finalize()
        return plant

    def _calc_proprioception_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._proprioception_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        joint_ee_command: JointEECommand = self._joint_ee_command_input.Eval(context)
        self.backend.send_joint_ee_command(joint_ee_command)

        joint_state = self.backend.read_joint_state()
        ee_state = self.backend.read_ee_state()
        header = TimestampHeader.from_system_time()
        self._sync_plant_context(joint_state)
        cartesian_state = CartesianState(
            header=header,
            cartesian_pose=self._compute_cartesian_pose(header),
            cartesian_twist=self._compute_cartesian_twist(header),
        )

        proprioception = Proprioception(
            header=header,
            joint_state=joint_state,
            cartesian_state=cartesian_state,
            ee_state=ee_state,
        )
        state.get_mutable_abstract_state(self._proprioception_state_index).set_value(proprioception)
        return EventStatus.Succeeded()

    def _sync_plant_context(self, joint_state: JointState) -> None:
        # The backend may emit a default-empty joint_state during early
        # bring-up before any real read; in that case skip writing into
        # the plant context (the plant keeps its previous / default q).
        positions = joint_state.joint_positions.positions
        velocities = joint_state.joint_velocities.velocities
        if positions.shape[0] == self.plant.num_positions():
            self.plant.SetPositions(self._plant_context, positions)
        if velocities.shape[0] == self.plant.num_velocities():
            self.plant.SetVelocities(self._plant_context, velocities)

    def _compute_cartesian_pose(self, header: TimestampHeader) -> CartesianPose:
        pose = self.plant.CalcRelativeTransform(
            self._plant_context,
            self._world_frame,
            self._tip_frame,
        )
        translation = np.asarray(pose.translation(), dtype=np.float64).copy()
        # Drake's RotationMatrix.ToQuaternion returns wxyz order, which
        # matches CartesianPose's quaternion convention.
        q = pose.rotation().ToQuaternion()
        orientation = np.array([q.w(), q.x(), q.y(), q.z()], dtype=np.float64)
        return CartesianPose(header=header, translation=translation, orientation=orientation)

    def _compute_cartesian_twist(self, header: TimestampHeader) -> CartesianTwist:
        # Spatial Jacobian J in world * v -> 6-vector [angular; linear].
        jacobian = self.plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kV,
            self._tip_frame,
            np.zeros(3),
            self._world_frame,
            self._world_frame,
        )
        velocities = self.plant.GetVelocities(self._plant_context)
        spatial = jacobian @ velocities
        angular = np.asarray(spatial[:3], dtype=np.float64).copy()
        linear = np.asarray(spatial[3:], dtype=np.float64).copy()
        return CartesianTwist(header=header, linear=linear, angular=angular)
