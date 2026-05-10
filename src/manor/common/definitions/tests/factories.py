"""
Random instance factories for every definition.

Each factory takes a numpy Generator and returns a fresh instance populated
with random data. Dimensions are randomized within reasonable bounds so tests
exercise varied sizes across runs (fixed seed keeps it reproducible).
"""

from __future__ import annotations

import numpy as np

from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_pose_trajectory import CartesianPoseTrajectory
from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.cartesian_state_trajectory import CartesianStateTrajectory
from manor.common.definitions.cartesian_trajectory_command import CartesianTrajectoryCommand
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.cartesian_twist_trajectory import CartesianTwistTrajectory
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_positions_trajectory import EEPositionsTrajectory
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_state_trajectory import EEStateTrajectory
from manor.common.definitions.ee_trajectory_command import EETrajectoryCommand
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.ee_velocities_trajectory import EEVelocitiesTrajectory
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_positions_trajectory import JointPositionsTrajectory
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_state_trajectory import JointStateTrajectory
from manor.common.definitions.joint_trajectory_command import JointTrajectoryCommand
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


def random_ee_positions(rng: np.random.Generator, num_coords: int | None = None) -> EEPositions:
    n = num_coords if num_coords is not None else _random_int(rng, 1, 6)
    return EEPositions(
        header=random_timestamp_header(rng),
        positions=rng.uniform(0.0, 0.1, size=n),
    )


def random_ee_velocities(rng: np.random.Generator, num_coords: int | None = None) -> EEVelocities:
    n = num_coords if num_coords is not None else _random_int(rng, 1, 6)
    return EEVelocities(
        header=random_timestamp_header(rng),
        velocities=rng.uniform(-0.5, 0.5, size=n),
    )


def random_ee_state(rng: np.random.Generator) -> EEState:
    n = _random_int(rng, 1, 6)
    return EEState(
        header=random_timestamp_header(rng),
        ee_positions=random_ee_positions(rng, num_coords=n),
        ee_velocities=random_ee_velocities(rng, num_coords=n),
    )


def random_ee_positions_trajectory(rng: np.random.Generator) -> EEPositionsTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEPositionsTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        ee_positions_array=rng.uniform(0.0, 0.1, size=(num_steps, num_coords)),
    )


def random_ee_velocities_trajectory(rng: np.random.Generator) -> EEVelocitiesTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEVelocitiesTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        ee_velocities_array=rng.uniform(-0.5, 0.5, size=(num_steps, num_coords)),
    )


def random_ee_state_trajectory(rng: np.random.Generator) -> EEStateTrajectory:
    num_steps = _random_int(rng, 1, 20)
    num_coords = _random_int(rng, 1, 6)
    return EEStateTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        ee_positions_array=rng.uniform(0.0, 0.1, size=(num_steps, num_coords)),
        ee_velocities_array=rng.uniform(-0.5, 0.5, size=(num_steps, num_coords)),
    )


def random_cartesian_pose(rng: np.random.Generator) -> CartesianPose:
    q = rng.normal(size=4)
    q = q / np.linalg.norm(q)
    return CartesianPose(
        header=random_timestamp_header(rng),
        translation=rng.uniform(-1.0, 1.0, size=3),
        orientation=q,
    )


def random_cartesian_twist(rng: np.random.Generator) -> CartesianTwist:
    return CartesianTwist(
        header=random_timestamp_header(rng),
        linear=rng.uniform(-1.0, 1.0, size=3),
        angular=rng.uniform(-1.0, 1.0, size=3),
    )


def random_cartesian_state(rng: np.random.Generator) -> CartesianState:
    return CartesianState(
        header=random_timestamp_header(rng),
        cartesian_pose=random_cartesian_pose(rng),
        cartesian_twist=random_cartesian_twist(rng),
    )


def random_cartesian_pose_trajectory(rng: np.random.Generator) -> CartesianPoseTrajectory:
    num_steps = _random_int(rng, 1, 20)
    translations = rng.uniform(-1.0, 1.0, size=(num_steps, 3))
    orientations = rng.normal(size=(num_steps, 4))
    orientations = orientations / np.linalg.norm(orientations, axis=1, keepdims=True)
    return CartesianPoseTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        translations_array=translations,
        orientations_array=orientations,
    )


def random_cartesian_state_trajectory(rng: np.random.Generator) -> CartesianStateTrajectory:
    num_steps = _random_int(rng, 1, 20)
    orientations = rng.normal(size=(num_steps, 4))
    orientations = orientations / np.linalg.norm(orientations, axis=1, keepdims=True)
    return CartesianStateTrajectory(
        header=random_timestamp_header(rng),
        times=np.sort(rng.uniform(0.0, 10.0, size=num_steps)),
        translations_array=rng.uniform(-1.0, 1.0, size=(num_steps, 3)),
        orientations_array=orientations,
        linear_array=rng.uniform(-1.0, 1.0, size=(num_steps, 3)),
        angular_array=rng.uniform(-1.0, 1.0, size=(num_steps, 3)),
    )


def random_cartesian_twist_trajectory(rng: np.random.Generator) -> CartesianTwistTrajectory:
    num_steps = _random_int(rng, 1, 20)
    return CartesianTwistTrajectory(
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
    Proprioception with either all optional sub-states populated or all None.
    """
    if all_none:
        return Proprioception(
            header=random_timestamp_header(rng),
            joint_state=random_joint_state(rng),
        )
    return Proprioception(
        header=random_timestamp_header(rng),
        joint_state=random_joint_state(rng),
        cartesian_state=random_cartesian_state(rng),
        ee_state=random_ee_state(rng),
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


_JOINT_COMMAND_VARIANT_FACTORIES = {
    "joint_positions": random_joint_positions,
    "joint_velocities": random_joint_velocities,
}


_JOINT_TRAJECTORY_COMMAND_VARIANT_FACTORIES = {
    "joint_positions_trajectory": random_joint_positions_trajectory,
    "joint_velocities_trajectory": random_joint_velocities_trajectory,
}


_CARTESIAN_COMMAND_VARIANT_FACTORIES = {
    "cartesian_pose": random_cartesian_pose,
    "cartesian_twist": random_cartesian_twist,
}


_CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FACTORIES = {
    "cartesian_pose_trajectory": random_cartesian_pose_trajectory,
    "cartesian_twist_trajectory": random_cartesian_twist_trajectory,
}


_EE_COMMAND_VARIANT_FACTORIES = {
    "ee_positions": random_ee_positions,
    "ee_velocities": random_ee_velocities,
}


_EE_TRAJECTORY_COMMAND_VARIANT_FACTORIES = {
    "ee_positions_trajectory": random_ee_positions_trajectory,
    "ee_velocities_trajectory": random_ee_velocities_trajectory,
}


def random_joint_command(rng: np.random.Generator, variant_field: str) -> JointCommand:
    return JointCommand(
        header=random_timestamp_header(rng),
        **{variant_field: _JOINT_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_joint_trajectory_command(rng: np.random.Generator, variant_field: str) -> JointTrajectoryCommand:
    return JointTrajectoryCommand(
        header=random_timestamp_header(rng),
        **{variant_field: _JOINT_TRAJECTORY_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_cartesian_command(rng: np.random.Generator, variant_field: str) -> CartesianCommand:
    return CartesianCommand(
        header=random_timestamp_header(rng),
        **{variant_field: _CARTESIAN_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_cartesian_trajectory_command(rng: np.random.Generator, variant_field: str) -> CartesianTrajectoryCommand:
    return CartesianTrajectoryCommand(
        header=random_timestamp_header(rng),
        **{variant_field: _CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_ee_command(rng: np.random.Generator, variant_field: str) -> EECommand:
    return EECommand(
        header=random_timestamp_header(rng),
        **{variant_field: _EE_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_ee_trajectory_command(rng: np.random.Generator, variant_field: str) -> EETrajectoryCommand:
    return EETrajectoryCommand(
        header=random_timestamp_header(rng),
        **{variant_field: _EE_TRAJECTORY_COMMAND_VARIANT_FACTORIES[variant_field](rng)},
    )


def random_joint_ee_command(
    rng: np.random.Generator,
    joint_variant_field: str,
    ee_variant_field: str | None = None,
) -> JointEECommand:
    joint_command = random_joint_command(rng, joint_variant_field)
    ee_command = random_ee_command(rng, ee_variant_field) if ee_variant_field is not None else None
    return JointEECommand(
        header=random_timestamp_header(rng),
        joint_command=joint_command,
        ee_command=ee_command,
    )


JOINT_COMMAND_VARIANT_FIELDS = tuple(_JOINT_COMMAND_VARIANT_FACTORIES)
JOINT_TRAJECTORY_COMMAND_VARIANT_FIELDS = tuple(_JOINT_TRAJECTORY_COMMAND_VARIANT_FACTORIES)
CARTESIAN_COMMAND_VARIANT_FIELDS = tuple(_CARTESIAN_COMMAND_VARIANT_FACTORIES)
CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FIELDS = tuple(_CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FACTORIES)
EE_COMMAND_VARIANT_FIELDS = tuple(_EE_COMMAND_VARIANT_FACTORIES)
EE_TRAJECTORY_COMMAND_VARIANT_FIELDS = tuple(_EE_TRAJECTORY_COMMAND_VARIANT_FACTORIES)


_ACTION_ARM_FIELD_FACTORIES = {
    "joint_command": (random_joint_command, JOINT_COMMAND_VARIANT_FIELDS),
    "joint_trajectory_command": (random_joint_trajectory_command, JOINT_TRAJECTORY_COMMAND_VARIANT_FIELDS),
    "cartesian_command": (random_cartesian_command, CARTESIAN_COMMAND_VARIANT_FIELDS),
    "cartesian_trajectory_command": (random_cartesian_trajectory_command, CARTESIAN_TRAJECTORY_COMMAND_VARIANT_FIELDS),
}


_ACTION_EE_FIELD_FACTORIES = {
    "ee_command": (random_ee_command, EE_COMMAND_VARIANT_FIELDS),
    "ee_trajectory_command": (random_ee_trajectory_command, EE_TRAJECTORY_COMMAND_VARIANT_FIELDS),
}


def random_action(
    rng: np.random.Generator,
    arm_field: str,
    ee_field: str | None = None,
) -> Action:
    arm_factory, arm_variants = _ACTION_ARM_FIELD_FACTORIES[arm_field]
    arm_variant = arm_variants[int(rng.integers(0, len(arm_variants)))]
    kwargs: dict = {arm_field: arm_factory(rng, arm_variant)}
    if ee_field is not None:
        ee_factory, ee_variants = _ACTION_EE_FIELD_FACTORIES[ee_field]
        ee_variant = ee_variants[int(rng.integers(0, len(ee_variants)))]
        kwargs[ee_field] = ee_factory(rng, ee_variant)
    return Action(header=random_timestamp_header(rng), **kwargs)


ACTION_ARM_FIELDS = tuple(_ACTION_ARM_FIELD_FACTORIES)
ACTION_EE_FIELDS = tuple(_ACTION_EE_FIELD_FACTORIES)
