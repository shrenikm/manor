"""
Abstract interface for a manipulator's model description.

A model bundles everything Drake (and downstream code) needs to
construct a MultibodyPlant for the manipulator and reason about its
state vector layout: the description file path, key frame names, joint
counts, and Drake-aligned position / velocity / state counts.
"""

from __future__ import annotations

import abc

import numpy as np

from manor.common.control.pid import PIDGains
from manor.common.custom_types import FilePath
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
    def compute_gripper_joint_positions(self, ee_positions: np.ndarray) -> np.ndarray:
        """
        Translate an EE-level position vector (size get_num_ee_dofs()) into
        the corresponding gripper-side block of the plant's q vector
        (size get_num_positions() - get_num_dof()).

        The EE-level vector is what the gripper interface exposes (e.g. a
        single number for parallel-gripper opening width or vacuum on/off).
        The plant-side block is whatever generalised-position layout Drake
        needs for the URDF (e.g. two prismatic finger joints for the Lite6
        parallel gripper). Implementations encode the URDF-specific mapping.
        Variants without actuated gripper joints in the plant return a
        length-0 array.
        """
        ...

    @abc.abstractmethod
    def compute_ee_positions_from_gripper_joints(self, gripper_joint_positions: np.ndarray) -> np.ndarray:
        """
        Inverse of compute_gripper_joint_positions: project the gripper-side
        block of the plant's q vector back onto the EE-level position
        vector (size get_num_ee_dofs()).

        For variants whose EE state is not represented in the plant (e.g.
        a binary vacuum gripper), implementations may return the last
        commanded value or a sensible default; the input is ignored.
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
