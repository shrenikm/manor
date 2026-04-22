"""
Default message constructors used throughout Aegis.

Every Drake abstract port and abstract state slot in Aegis needs a model value
of the right concrete type. These factories produce neutral, zero-shaped
instances suitable for that purpose, and a ``system_time_header`` helper for
stamping fresh messages with live monotonic + system time.
"""

from __future__ import annotations

import time

import numpy as np

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.rgbd_image_data import RGBDImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.enums import DepthEncoding, ImageEncoding


def zero_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=0, system_ns=0)


def system_time_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=time.monotonic_ns(), system_ns=time.time_ns())


def default_joint_positions(num_joints: int = 0) -> JointPositions:
    return JointPositions(header=zero_header(), positions=np.zeros(num_joints, dtype=np.float64))


def default_joint_velocities(num_joints: int = 0) -> JointVelocities:
    return JointVelocities(header=zero_header(), velocities=np.zeros(num_joints, dtype=np.float64))


def default_joint_state(num_joints: int = 0) -> JointState:
    return JointState(
        header=zero_header(),
        joint_positions=default_joint_positions(num_joints),
        joint_velocities=default_joint_velocities(num_joints),
    )


def default_eef_positions(num_eef_dofs: int = 0) -> EEFPositions:
    return EEFPositions(header=zero_header(), positions=np.zeros(num_eef_dofs, dtype=np.float64))


def default_eef_velocities(num_eef_dofs: int = 0) -> EEFVelocities:
    return EEFVelocities(header=zero_header(), velocities=np.zeros(num_eef_dofs, dtype=np.float64))


def default_eef_state(num_eef_dofs: int = 0) -> EEFState:
    return EEFState(
        header=zero_header(),
        eef_positions=default_eef_positions(num_eef_dofs),
        eef_velocities=default_eef_velocities(num_eef_dofs),
    )


def default_eef_pose() -> EEFPose:
    # Identity quaternion (w, x, y, z) at the origin.
    return EEFPose(
        header=zero_header(),
        translation=np.zeros(3, dtype=np.float64),
        orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


def default_eef_twist() -> EEFTwist:
    return EEFTwist(
        header=zero_header(),
        linear=np.zeros(3, dtype=np.float64),
        angular=np.zeros(3, dtype=np.float64),
    )


def default_proprioception(num_joints: int = 0, num_eef_dofs: int = 0) -> Proprioception:
    return Proprioception(
        header=zero_header(),
        joint_state=default_joint_state(num_joints),
        eef_state=default_eef_state(num_eef_dofs),
        eef_pose=default_eef_pose(),
        eef_twist=default_eef_twist(),
    )


def default_action(num_joints: int = 0) -> Action:
    return Action(header=zero_header(), joint_positions=default_joint_positions(num_joints))


def default_command(num_joints: int = 0) -> Command:
    return Command(header=zero_header(), joint_positions=default_joint_positions(num_joints))


def default_rgb_image(height: int = 0, width: int = 0) -> RGBImageData:
    return RGBImageData(
        header=zero_header(),
        height=height,
        width=width,
        encoding=ImageEncoding.RAW_RGB8,
        data=b"",
    )


def default_depth_image(height: int = 0, width: int = 0) -> DepthImageData:
    return DepthImageData(
        header=zero_header(),
        height=height,
        width=width,
        encoding=DepthEncoding.RAW_FLOAT32_M,
        data=b"",
        depth_scale=1.0,
    )


def default_rgbd_image(height: int = 0, width: int = 0) -> RGBDImageData:
    return RGBDImageData(
        header=zero_header(),
        rgb=default_rgb_image(height, width),
        depth=default_depth_image(height, width),
    )


def default_observation(num_joints: int = 0, num_eef_dofs: int = 0) -> Observation:
    return Observation(
        header=zero_header(),
        proprioception=default_proprioception(num_joints, num_eef_dofs),
        rgb_image=default_rgb_image(),
        rgbd_image=None,
    )
