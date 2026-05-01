"""
Tests for IKPassthroughController: joint and EE pass-through, Cartesian
pose -> IK, Cartesian twist -> diff IK, trajectory fallback.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.aegis.kyber.controllers.controller_manager import KyberController, KyberControllerType
from manor.common.aegis.kyber.controllers.ik_passthrough_controller import (
    IKPassthroughController,
    IKPassthroughControllerConfig,
)
from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_trajectory_command import CartesianTrajectoryCommand
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.ee_command import EECommand
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_trajectory_command import JointTrajectoryCommand
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_model() -> Lite6Model:
    return Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL)


def _make_proprioception(model: Lite6Model, q_arm: np.ndarray | None = None) -> Proprioception:
    n_q = model.get_num_positions()
    n_v = model.get_num_velocities()
    if q_arm is None:
        q_arm = np.zeros(LITE6_ARM_DOF, dtype=np.float64)
    q = np.zeros(n_q, dtype=np.float64)
    q[: q_arm.shape[0]] = q_arm
    h = TimestampHeader.from_system_time()
    joint_state = JointState(
        header=h,
        joint_positions=JointPositions(header=h, positions=q),
        joint_velocities=JointVelocities(header=h, velocities=np.zeros(n_v, dtype=np.float64)),
    )
    return Proprioception(header=h, joint_state=joint_state)


def _build_controller(orientation_theta_bound_rad: float = 1e-3) -> IKPassthroughController:
    return IKPassthroughController.build(
        config=IKPassthroughControllerConfig(orientation_theta_bound_rad=orientation_theta_bound_rad),
        manipulator_model=_make_model(),
    )


class TestProtocolCompliance:
    def test_satisfies_kyber_controller_protocol(self) -> None:
        assert isinstance(_build_controller(), KyberController)

    def test_config_pins_enum(self) -> None:
        assert IKPassthroughControllerConfig.CONTROLLER_TYPE is KyberControllerType.IK_PASSTHROUGH


class TestJointCommandPassthrough:
    def test_passes_joint_positions_through(self) -> None:
        controller = _build_controller()
        h = TimestampHeader.from_system_time()
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        action = Action(
            header=h,
            joint_command=JointCommand(
                header=h,
                joint_positions=JointPositions(header=h, positions=positions),
            ),
        )
        cmd = controller.step(action, _make_proprioception(_make_model()))
        np.testing.assert_array_equal(cmd.joint_command.joint_positions.positions, positions)
        assert cmd.ee_command is None

    def test_passes_ee_command_through(self) -> None:
        controller = _build_controller()
        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            joint_command=JointCommand(
                header=h,
                joint_positions=JointPositions(header=h, positions=np.zeros(LITE6_ARM_DOF, dtype=np.float64)),
            ),
            ee_command=EECommand(
                header=h,
                ee_positions=EEPositions(header=h, positions=np.array([0.04], dtype=np.float64)),
            ),
        )
        cmd = controller.step(action, _make_proprioception(_make_model()))
        assert cmd.ee_command is not None
        np.testing.assert_array_equal(cmd.ee_command.ee_positions.positions, [0.04])


class TestCartesianPoseIK:
    def test_solves_for_arm_q_at_current_pose(self) -> None:
        # Run FK on the controller's own plant, then ask IK to land at
        # the same pose. The solution should agree with the seed
        # (current q) within the IK tolerance, and be sized to num_dof
        # (i.e., gripper joints are not in the returned command).
        model = _make_model()
        controller = _build_controller(orientation_theta_bound_rad=0.05)
        q_arm = np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0], dtype=np.float64)
        proprio = _make_proprioception(model, q_arm=q_arm)

        controller.plant.SetPositions(controller._plant_context, proprio.joint_state.joint_positions.positions)
        tip = controller.plant.GetFrameByName(model.get_fk_ik_frame_name())
        T = controller.plant.CalcRelativeTransform(controller._plant_context, controller.plant.world_frame(), tip)
        translation = np.asarray(T.translation(), dtype=np.float64).copy()
        q = T.rotation().ToQuaternion()
        orientation = np.array([q.w(), q.x(), q.y(), q.z()], dtype=np.float64)

        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            cartesian_command=CartesianCommand(
                header=h,
                cartesian_pose=CartesianPose(header=h, translation=translation, orientation=orientation),
            ),
        )
        cmd = controller.step(action, proprio)
        assert cmd.joint_command.joint_positions is not None
        assert cmd.joint_command.joint_positions.positions.shape == (LITE6_ARM_DOF,)
        np.testing.assert_allclose(cmd.joint_command.joint_positions.positions, q_arm, atol=1e-2)


class TestCartesianTwistDiffIK:
    def test_zero_twist_emits_zero_velocity(self) -> None:
        model = _make_model()
        controller = _build_controller()
        proprio = _make_proprioception(model, q_arm=np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0], dtype=np.float64))
        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            cartesian_command=CartesianCommand(
                header=h,
                cartesian_twist=CartesianTwist(header=h, linear=np.zeros(3), angular=np.zeros(3)),
            ),
        )
        cmd = controller.step(action, proprio)
        assert cmd.joint_command.joint_velocities is not None
        assert cmd.joint_command.joint_velocities.velocities.shape == (LITE6_ARM_DOF,)
        np.testing.assert_allclose(cmd.joint_command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF), atol=1e-6)

    def test_pure_linear_twist_produces_nonzero_arm_velocity(self) -> None:
        # A non-singular configuration plus a small linear x-twist
        # should result in a non-trivial arm-velocity command.
        model = _make_model()
        controller = _build_controller()
        q_arm = np.array([0.0, 0.3, 0.3, 0.0, 0.0, 0.0], dtype=np.float64)
        proprio = _make_proprioception(model, q_arm=q_arm)
        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            cartesian_command=CartesianCommand(
                header=h,
                cartesian_twist=CartesianTwist(
                    header=h,
                    linear=np.array([0.05, 0.0, 0.0]),
                    angular=np.zeros(3),
                ),
            ),
        )
        cmd = controller.step(action, proprio)
        assert cmd.joint_command.joint_velocities is not None
        velocities = cmd.joint_command.joint_velocities.velocities
        assert velocities.shape == (LITE6_ARM_DOF,)
        assert np.linalg.norm(velocities) > 1e-3


class TestTrajectoryFallback:
    def test_joint_trajectory_falls_back_to_zero_velocity(self) -> None:
        model = _make_model()
        controller = _build_controller()
        proprio = _make_proprioception(model)
        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            joint_trajectory_command=JointTrajectoryCommand.construct_default(),
        )
        cmd = controller.step(action, proprio)
        assert cmd.joint_command.joint_velocities is not None
        np.testing.assert_array_equal(cmd.joint_command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))

    def test_cartesian_trajectory_falls_back_to_zero_velocity(self) -> None:
        model = _make_model()
        controller = _build_controller()
        proprio = _make_proprioception(model)
        h = TimestampHeader.from_system_time()
        action = Action(
            header=h,
            cartesian_trajectory_command=CartesianTrajectoryCommand.construct_default(),
        )
        cmd = controller.step(action, proprio)
        assert cmd.joint_command.joint_velocities is not None
        np.testing.assert_array_equal(cmd.joint_command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))


if __name__ == "__main__":
    run_manor_tests()
