"""
Abstract interface for a manipulator's model description.

A model bundles everything Drake (and downstream code) needs to
construct a MultibodyPlant for the manipulator and reason about its
state vector layout: the description file path, key frame names, joint
counts, and Drake-aligned position / velocity / state counts.
"""

from __future__ import annotations

import abc

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
    def get_description_path(self) -> FilePath:
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
    def get_eef_tip_frame_name(self) -> str:
        """
        Name of the end-effector tip frame (used for FK / IK targeting).
        """
        ...

    @abc.abstractmethod
    def get_num_positions(self) -> int:
        """
        Number of generalized positions in the MultibodyPlant for this
        model (arm DOFs + any actuated EEF DOFs). Used for sizing /
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
