"""
Random instance factories for every definition.

Each factory takes a numpy Generator and returns a fresh instance populated
with random data. Dimensions are randomized within reasonable bounds so tests
exercise varied sizes across runs (fixed seed keeps it reproducible).
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_pose_trajectory import EEFPoseTrajectory
from manor.common.definitions.eef_positions import EEFPositions
from manor.common.definitions.eef_positions_trajectory import EEFPositionsTrajectory
from manor.common.definitions.eef_state import EEFState
from manor.common.definitions.eef_state_trajectory import EEFStateTrajectory
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.eef_twist_trajectory import EEFTwistTrajectory
from manor.common.definitions.eef_velocities import EEFVelocities
from manor.common.definitions.eef_velocities_trajectory import EEFVelocitiesTrajectory
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_state_trajectory import JointStateTrajectory
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.joint_velocities_trajectory import JointVelocitiesTrajectory
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.rgbd_image_data import RGBDImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.enums import DepthEncoding, ImageEncoding


def _random_int(rng: np.random.Generator, low: int, high: int) -> int:
    return int(rng.integers(low, high))


def random_timestamp_header(rng: np.random.Generator) -> TimestampHeader:
    return TimestampHeader(
        monotonic_ns=_random_int(rng, 0, 2**40),
        system_ns=_random_int(rng, 0, 2**40),
    )


def random_joint_positions(rng: np.random.Generator, num_joints: int | None = None) -> JointPositions:
    n = num_joints if num_joints is not None else _random_int(rng, 1, 10)
    return JointPositions(
        header=random_timestamp_header(rng),
        positions=rng.uniform(-np.pi, np.pi, size=n),
    )


def random_joint_velocities(rng: np.random.Generator, num_joints: int | None = None) -> JointVelocities:
    n = num_joints if num_joints is not None else _random_int(rng, 1, 10)
    return JointVelocities(
        header=random_timestamp_header(rng),
        velocities=rng.uniform(-5.0, 5.0, size=n),
    )


def random_joint_state(rng: np.random.Generator) -> JointState:
    n = _random_int(rng, 1, 10)
    return JointState(
        header=random_timestamp_header(rng),
        joint_positions=random_joint_positions(rng, num_joints=n),
        joint_velocities=random_joint_velocities(rng, num_joints=n),
    )


def random_joint_positions_trajectory(rng: np.random.Generator) -> JointPositionsTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_joints = _random_int(rng, 1, 10)
    return JointPositionsTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        joint_positions_array=rng.uniform(-np.pi, np.pi, size=(num_steps, num_joints)),
    )


def random_joint_velocities_trajectory(rng: np.random.Generator) -> JointVelocitiesTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_joints = _random_int(rng, 1, 10)
    return JointVelocitiesTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        joint_velocities_array=rng.uniform(-5.0, 5.0, size=(num_steps, num_joints)),
    )


def random_joint_state_trajectory(rng: np.random.Generator) -> JointStateTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_joints = _random_int(rng, 1, 10)
    return JointStateTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        joint_positions_array=rng.uniform(-np.pi, np.pi, size=(num_steps, num_joints)),
        joint_velocities_array=rng.uniform(-5.0, 5.0, size=(num_steps, num_joints)),
    )


def random_eef_positions(rng: np.random.Generator, num_coords: int | None = None) -> EEFPositions:
    n = num_coords if num_coords is not None else _random_int(rng, 1, 6)
    return EEFPositions(
        header=random_timestamp_header(rng),
        positions=rng.uniform(0.0, 0.1, size=n),
    )


def random_eef_velocities(rng: np.random.Generator, num_coords: int | None = None) -> EEFVelocities:
    n = num_coords if num_coords is not None else _random_int(rng, 1, 6)
    return EEFVelocities(
        header=random_timestamp_header(rng),
        velocities=rng.uniform(-0.5, 0.5, size=n),
    )


def random_eef_state(rng: np.random.Generator) -> EEFState:
    n = _random_int(rng, 1, 6)
    return EEFState(
        header=random_timestamp_header(rng),
        eef_positions=random_eef_positions(rng, num_coords=n),
        eef_velocities=random_eef_velocities(rng, num_coords=n),
    )


def random_eef_positions_trajectory(rng: np.random.Generator) -> EEFPositionsTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEFPositionsTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        eef_positions_array=rng.uniform(0.0, 0.1, size=(num_steps, num_coords)),
    )


def random_eef_velocities_trajectory(rng: np.random.Generator) -> EEFVelocitiesTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEFVelocitiesTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        eef_velocities_array=rng.uniform(-0.5, 0.5, size=(num_steps, num_coords)),
    )


def random_eef_state_trajectory(rng: np.random.Generator) -> EEFStateTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEFStateTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        eef_positions_array=rng.uniform(0.0, 0.1, size=(num_steps, num_coords)),
        eef_velocities_array=rng.uniform(-0.5, 0.5, size=(num_steps, num_coords)),
    )


def random_eef_pose(rng: np.random.Generator) -> EEFPose:
    q = rng.normal(size=4)
    q = q / np.linalg.norm(q)
    return EEFPose(
        header=random_timestamp_header(rng),
        translation=rng.uniform(-1.0, 1.0, size=3),
        orientation=q,
    )


def random_eef_twist(rng: np.random.Generator) -> EEFTwist:
    return EEFTwist(
        header=random_timestamp_header(rng),
        linear=rng.uniform(-1.0, 1.0, size=3),
        angular=rng.uniform(-1.0, 1.0, size=3),
    )


def random_eef_pose_trajectory(rng: np.random.Generator) -> EEFPoseTrajectory:
    num_steps = _random_int(rng, 1, 20)
    translations = rng.uniform(-1.0, 1.0, size=(num_steps, 3))
    orientations = rng.normal(size=(num_steps, 4))
    orientations = orientations / np.linalg.norm(orientations, axis=1, keepdims=True)
    return EEFPoseTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        translations_array=translations,
        orientations_array=orientations,
    )


def random_eef_twist_trajectory(rng: np.random.Generator) -> EEFTwistTrajectory:
    num_steps = _random_int(rng, 1, 20)
    return EEFTwistTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        linear_array=rng.uniform(-1.0, 1.0, size=(num_steps, 3)),
        angular_array=rng.uniform(-1.0, 1.0, size=(num_steps, 3)),
    )


def random_rgb_image_data(rng: np.random.Generator) -> RGBImageData:
    encoding = ImageEncoding(rng.choice(list(ImageEncoding)))
    height = _random_int(rng, 1, 32)
    width = _random_int(rng, 1, 32)
    if encoding in (ImageEncoding.RAW_RGB8, ImageEncoding.RAW_BGR8):
        n = height * width * 3
    else:
        n = _random_int(rng, 16, 256)
    return RGBImageData(
        header=random_timestamp_header(rng),
        height=height,
        width=width,
        encoding=encoding,
        data=rng.bytes(n),
    )


def random_depth_image_data(rng: np.random.Generator) -> DepthImageData:
    encoding = DepthEncoding(rng.choice(list(DepthEncoding)))
    height = _random_int(rng, 1, 32)
    width = _random_int(rng, 1, 32)
    if encoding == DepthEncoding.RAW_FLOAT32_M:
        n = height * width * 4
        depth_scale = 1.0
    elif encoding == DepthEncoding.RAW_UINT16_MM:
        n = height * width * 2
        depth_scale = 1e-3
    else:
        n = _random_int(rng, 16, 256)
        depth_scale = 1e-3
    return DepthImageData(
        header=random_timestamp_header(rng),
        height=height,
        width=width,
        encoding=encoding,
        data=rng.bytes(n),
        depth_scale=depth_scale,
    )


def random_rgbd_image_data(rng: np.random.Generator) -> RGBDImageData:
    return RGBDImageData(
        header=random_timestamp_header(rng),
        rgb=random_rgb_image_data(rng),
        depth=random_depth_image_data(rng),
    )


def random_proprioception(rng: np.random.Generator, all_none: bool) -> Proprioception:
    """
    Proprioception with either all optional fields populated or all None.
    """
    if all_none:
        return Proprioception(
            header=random_timestamp_header(rng),
            joint_state=random_joint_state(rng),
        )
    return Proprioception(
        header=random_timestamp_header(rng),
        joint_state=random_joint_state(rng),
        eef_state=random_eef_state(rng),
        eef_pose=random_eef_pose(rng),
        eef_twist=random_eef_twist(rng),
    )


def random_observation(rng: np.random.Generator, all_none: bool) -> Observation:
    """
    Observation with either all optional channels populated or all None.
    """
    if all_none:
        return Observation(header=random_timestamp_header(rng))
    return Observation(
        header=random_timestamp_header(rng),
        proprioception=random_proprioception(rng, all_none=False),
        rgb_image=random_rgb_image_data(rng),
        rgbd_image=random_rgbd_image_data(rng),
    )


_ACTION_VARIANT_FACTORIES = {
    "joint_positions": random_joint_positions,
    "joint_positions_trajectory": random_joint_positions_trajectory,
    "joint_velocities": random_joint_velocities,
    "joint_velocities_trajectory": random_joint_velocities_trajectory,
    "eef_pose": random_eef_pose,
    "eef_pose_trajectory": random_eef_pose_trajectory,
    "eef_twist": random_eef_twist,
    "eef_twist_trajectory": random_eef_twist_trajectory,
}


_COMMAND_VARIANT_FACTORIES = {
    "joint_positions": random_joint_positions,
    "joint_velocities": random_joint_velocities,
    "eef_pose": random_eef_pose,
    "eef_twist": random_eef_twist,
}


def random_action(rng: np.random.Generator, variant_field: str) -> Action:
    return Action(
        header=random_timestamp_header(rng),
        **{variant_field: _ACTION_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_command(rng: np.random.Generator, variant_field: str) -> Command:
    return Command(
        header=random_timestamp_header(rng),
        **{variant_field: _COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


ACTION_VARIANT_FIELDS = tuple(_ACTION_VARIANT_FACTORIES)
COMMAND_VARIANT_FIELDS = tuple(_COMMAND_VARIANT_FACTORIES)
