"""
Constructors for default / neutral instances of every streaming definition.

These exist so consumers that need a model value (e.g. Drake abstract port
declarations) can ask for a well-formed instance of any definition without
hand-wiring all of its nested fields. Every ``construct_*`` returns a fresh,
independently-owned object; numerical array sizes are parameterised.

``construct_system_time_header`` is the one live-clock helper in the module:
it reads real monotonic + system time at call time and is meant for stamping
freshly-produced messages.
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


def construct_zero_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=0, system_ns=0)


def construct_system_time_header() -> TimestampHeader:
    return TimestampHeader(monotonic_ns=time.monotonic_ns(), system_ns=time.time_ns())


def construct_joint_positions(num_joints: int = 0) -> JointPositions:
    return JointPositions(header=construct_zero_header(), positions=np.zeros(num_joints, dtype=np.float64))


def construct_joint_velocities(num_joints: int = 0) -> JointVelocities:
    return JointVelocities(header=construct_zero_header(), velocities=np.zeros(num_joints, dtype=np.float64))


def construct_joint_state(num_joints: int = 0) -> JointState:
    return JointState(
        header=construct_zero_header(),
        joint_positions=construct_joint_positions(num_joints),
        joint_velocities=construct_joint_velocities(num_joints),
    )


def construct_eef_positions(num_eef_dofs: int = 0) -> EEFPositions:
    return EEFPositions(header=construct_zero_header(), positions=np.zeros(num_eef_dofs, dtype=np.float64))


def construct_eef_velocities(num_eef_dofs: int = 0) -> EEFVelocities:
    return EEFVelocities(header=construct_zero_header(), velocities=np.zeros(num_eef_dofs, dtype=np.float64))


def construct_eef_state(num_eef_dofs: int = 0) -> EEFState:
    return EEFState(
        header=construct_zero_header(),
        eef_positions=construct_eef_positions(num_eef_dofs),
        eef_velocities=construct_eef_velocities(num_eef_dofs),
    )


def construct_eef_pose() -> EEFPose:
    # Identity quaternion (w, x, y, z) at the origin.
    return EEFPose(
        header=construct_zero_header(),
        translation=np.zeros(3, dtype=np.float64),
        orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


def construct_eef_twist() -> EEFTwist:
    return EEFTwist(
        header=construct_zero_header(),
        linear=np.zeros(3, dtype=np.float64),
        angular=np.zeros(3, dtype=np.float64),
    )


def construct_proprioception(num_joints: int = 0, num_eef_dofs: int = 0) -> Proprioception:
    return Proprioception(
        header=construct_zero_header(),
        joint_state=construct_joint_state(num_joints),
        eef_state=construct_eef_state(num_eef_dofs),
        eef_pose=construct_eef_pose(),
        eef_twist=construct_eef_twist(),
    )


def construct_action(num_joints: int = 0) -> Action:
    return Action(header=construct_zero_header(), joint_positions=construct_joint_positions(num_joints))


def construct_command(num_joints: int = 0) -> Command:
    return Command(header=construct_zero_header(), joint_positions=construct_joint_positions(num_joints))


def construct_rgb_image(height: int = 0, width: int = 0) -> RGBImageData:
    return RGBImageData(
        header=construct_zero_header(),
        height=height,
        width=width,
        encoding=ImageEncoding.RAW_RGB8,
        data=b"",
    )


def construct_depth_image(height: int = 0, width: int = 0) -> DepthImageData:
    return DepthImageData(
        header=construct_zero_header(),
        height=height,
        width=width,
        encoding=DepthEncoding.RAW_FLOAT32_M,
        data=b"",
        depth_scale=1.0,
    )


def construct_rgbd_image(height: int = 0, width: int = 0) -> RGBDImageData:
    return RGBDImageData(
        header=construct_zero_header(),
        rgb=construct_rgb_image(height, width),
        depth=construct_depth_image(height, width),
    )


def construct_observation(num_joints: int = 0, num_eef_dofs: int = 0) -> Observation:
    return Observation(
        header=construct_zero_header(),
        proprioception=construct_proprioception(num_joints, num_eef_dofs),
        rgb_image=construct_rgb_image(),
        rgbd_image=None,
    )
