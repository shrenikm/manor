"""
Abstract interface for a manipulator's model description.

A model bundles everything Drake (and downstream code) needs to
construct a MultibodyPlant for the manipulator and reason about its
state vector layout: the description file path, key frame names, joint
counts, and Drake-aligned position / velocity / state counts.
"""

from __future__ import annotations

import abc

from manor.common.control.pid import PIDGains
from manor.common.custom_types import (
    EEPositionsVector,
    EEVelocitiesVector,
    FilePath,
    PlantEEPositionsVector,
    PlantEEVelocitiesVector,
)
from manor.manipulators.manipulator_type import ManipulatorType
from manor.manipulators.manipulator_variant import IManipulatorVariant


class IManipulatorModel(abc.ABC):
    """
    Abstract description of a manipulator (one variant of one family).
    Implementations are typically frozen attrs classes parameterized by
    a concrete ``IManipulatorVariant``.
    """

    @abc.abstractmethod
    def get_manipulator_type(self) -> ManipulatorType: ...

    @abc.abstractmethod
    def get_variant(self) -> IManipulatorVariant: ...

    @abc.abstractmethod
    def get_num_dof(self) -> int:
        """
        Degrees of freedom of the main arm (excluding any end-effector
        DOFs). Independent of the variant for a fixed manipulator family.
        """
        ...

    @abc.abstractmethod
    def get_num_ee_dofs(self) -> int:
        """
        Number of end-effector generalized DOFs reported on
        ``EEPositions`` / ``EEVelocities`` for this variant. Zero for
        variants without an actuated EE; conventionally 1 for binary
        on/off vacuum grippers; equal to the gripper's prismatic-joint
        count for actuated parallel grippers.
        """
        ...

    @abc.abstractmethod
    def get_description_filepath(self) -> FilePath:
        """
        Absolute path to the description file (URDF today; SDF / MJCF
        later) that Drake's parser loads to build the MultibodyPlant.
        """
        ...

    @abc.abstractmethod
    def get_base_frame_name(self) -> str:
        """
        Name of the model's base frame (the one that gets welded to the
        world or to a mounting frame).
        """
        ...

    @abc.abstractmethod
    def get_fk_ik_frame_name(self) -> str:
        """
        Name of the frame used as the reference point for forward and
        inverse kinematics (typically the end-effector tip). FK reads
        out this frame's pose / spatial velocity; IK / diff-IK / IDC
        targeting solves for joint positions that place this frame at a
        desired pose.
        """
        ...

    @abc.abstractmethod
    def get_num_positions(self) -> int:
        """
        Number of generalized positions in the MultibodyPlant for this
        model (arm DOFs + any actuated EE DOFs). Used for sizing /
        slicing Drake state vectors.
        """
        ...

    @abc.abstractmethod
    def get_num_velocities(self) -> int:
        """
        Number of generalized velocities in the MultibodyPlant for this
        model.
        """
        ...

    @abc.abstractmethod
    def get_num_states(self) -> int:
        """
        Total number of MultibodyPlant state entries
        (``num_positions + num_velocities``).
        """
        ...

    @abc.abstractmethod
    def get_ee_fully_open_positions(self) -> EEPositionsVector:
        """
        EE-level positions that represent the "fully open" pose for
        this end-effector, sized to get_num_ee_dofs(). The semantics
        are per-EE: maximum opening width for a parallel gripper,
        "off" for a vacuum gripper, palm-flat finger angles for a
        dexterous hand. Implementations source this from the URDF's
        joint limits (or model-specific calibration) so the value
        always tracks the actual hardware capability.
        """
        ...

    @abc.abstractmethod
    def get_ee_fully_closed_positions(self) -> EEPositionsVector:
        """
        EE-level positions that represent the "fully closed" pose for
        this end-effector. Inverse pole of get_ee_fully_open_positions.
        """
        ...

    @abc.abstractmethod
    def ee_positions_to_plant_positions(self, ee_positions: EEPositionsVector) -> PlantEEPositionsVector:
        """
        Translate an EE-level positions vector (size get_num_ee_dofs())
        into the EE block of the plant's generalised-position vector q
        (size get_num_positions() - get_num_dof()).

        The EE-level representation is what the policy / controller
        layer talks about: e.g. an opening width for a parallel gripper,
        a binary on/off scalar for a vacuum gripper, finger-joint
        angles for a dexterous hand. The plant-side block is the
        URDF-defined layout Drake actually integrates: e.g. the two
        signed prismatic finger joints of a parallel gripper, or zero
        actuated joints for a vacuum gripper. Implementations own the
        URDF-specific encoding (joint signs, gear ratios, mimic links).

        Variants whose EE has no actuated joints in the plant return a
        length-0 array.
        """
        ...

    @abc.abstractmethod
    def plant_positions_to_ee_positions(self, plant_ee_positions: PlantEEPositionsVector) -> EEPositionsVector:
        """
        Inverse of ee_positions_to_plant_positions: take the EE block of
        the plant's q vector and project it back onto the EE-level
        positions vector (size get_num_ee_dofs()).

        For EE variants whose state is not represented in the plant
        (e.g. a binary vacuum gripper, where the URDF has no actuated
        EE joints), implementations may return a sensible default; the
        input is ignored in that case.
        """
        ...

    @abc.abstractmethod
    def ee_velocities_to_plant_velocities(self, ee_velocities: EEVelocitiesVector) -> PlantEEVelocitiesVector:
        """
        Velocity-side analogue of ee_positions_to_plant_positions:
        translate an EE-level velocities vector (size get_num_ee_dofs())
        into the EE block of the plant's generalised-velocity vector v.

        Implementations encode the time-derivative of the URDF
        position-side mapping. Constant offsets that show up in the
        position mapping (e.g. URDF link origins) drop out of the
        derivative, so the velocity mapping may be simpler than the
        position one. Variants whose EE has no actuated joints in the
        plant return a length-0 array.
        """
        ...

    @abc.abstractmethod
    def plant_velocities_to_ee_velocities(self, plant_ee_velocities: PlantEEVelocitiesVector) -> EEVelocitiesVector:
        """
        Inverse of ee_velocities_to_plant_velocities: take the EE block
        of the plant's v vector and project it back onto the EE-level
        velocities vector. Same fallback rules as
        plant_positions_to_ee_positions for EE variants without
        actuated plant joints.
        """
        ...

    @abc.abstractmethod
    def get_default_sim_pid_gains(self) -> PIDGains:
        """
        Per-joint PID gains for the in-sim ``InverseDynamicsController``
        that drives this manipulator's actuation port. Sized to
        ``get_num_positions()``. Concrete manipulator models tune
        these against their own inertia / damping / joint scales --
        no generic default lives here.

        Sim-only -- on hardware the real controller lives on the robot.
        """
        ...
