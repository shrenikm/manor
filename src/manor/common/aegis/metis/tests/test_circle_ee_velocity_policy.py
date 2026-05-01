"""
Tests for CircleEEVelocityPolicy.
"""

from __future__ import annotations

import time

import numpy as np

from manor.common.aegis.metis.policies.circle_ee_velocity_policy import (
    CircleEEVelocityPolicy,
    CircleEEVelocityPolicyConfig,
)
from manor.common.aegis.metis.policies.policy_manager import (
    MetisPolicy,
    MetisPolicyManager,
    MetisPolicyType,
)
from manor.common.definitions.cartesian_pose import CartesianPose
from manor.common.definitions.cartesian_state import CartesianState
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.testing_utils import run_manor_tests


def _make_observation(translation: np.ndarray | None) -> Observation:
    h = TimestampHeader.from_system_time()
    if translation is None:
        return Observation(header=h)
    cartesian_state = CartesianState(
        header=h,
        cartesian_pose=CartesianPose(
            header=h,
            translation=translation,
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        ),
        cartesian_twist=CartesianTwist(
            header=h,
            linear=np.zeros(3),
            angular=np.zeros(3),
        ),
    )
    n = 6
    joint_state = JointState(
        header=h,
        joint_positions=JointPositions(header=h, positions=np.zeros(n)),
        joint_velocities=JointVelocities(header=h, velocities=np.zeros(n)),
    )
    proprioception = Proprioception(
        header=h,
        joint_state=joint_state,
        cartesian_state=cartesian_state,
    )
    return Observation(header=h, proprioception=proprioception)


class TestProtocolCompliance:
    def test_satisfies_metis_policy_protocol(self) -> None:
        policy = CircleEEVelocityPolicy.from_config(CircleEEVelocityPolicyConfig())
        assert isinstance(policy, MetisPolicy)

    def test_config_pins_enum(self) -> None:
        assert CircleEEVelocityPolicyConfig.POLICY_TYPE is MetisPolicyType.CIRCLE_EE_VELOCITY


class TestLatching:
    def test_zero_twist_when_no_proprioception(self) -> None:
        policy = CircleEEVelocityPolicy.from_config(
            CircleEEVelocityPolicyConfig(radius=0.05, velocity_magnitude=0.05, duration_seconds=10.0)
        )
        action = policy.step(_make_observation(translation=None))
        twist = action.cartesian_command.cartesian_twist
        np.testing.assert_array_equal(twist.linear, np.zeros(3))
        np.testing.assert_array_equal(twist.angular, np.zeros(3))


class TestTangentDirection:
    def test_initial_tangent_points_in_plus_x(self) -> None:
        # At the bottom of the circle, anticlockwise from +z view, the
        # tangent should be in +x.
        policy = CircleEEVelocityPolicy.from_config(
            CircleEEVelocityPolicyConfig(radius=0.05, velocity_magnitude=0.05, duration_seconds=10.0)
        )
        translation = np.array([0.2, 0.0, 0.3], dtype=np.float64)
        action = policy.step(_make_observation(translation))
        twist = action.cartesian_command.cartesian_twist
        # First tick has elapsed ~0, so tangent ≈ (+vx, 0, 0).
        assert twist.linear[0] > 0.04
        assert abs(twist.linear[1]) < 1e-3
        assert abs(twist.linear[2]) < 1e-12
        # Magnitude matches velocity_magnitude.
        np.testing.assert_allclose(np.linalg.norm(twist.linear), 0.05, atol=1e-3)

    def test_velocity_hold_after_duration(self) -> None:
        policy = CircleEEVelocityPolicy.from_config(
            CircleEEVelocityPolicyConfig(radius=0.05, velocity_magnitude=0.05, duration_seconds=0.05)
        )
        translation = np.array([0.2, 0.0, 0.3], dtype=np.float64)
        policy.step(_make_observation(translation))
        time.sleep(0.07)
        action = policy.step(_make_observation(translation))
        twist = action.cartesian_command.cartesian_twist
        np.testing.assert_array_equal(twist.linear, np.zeros(3))
        np.testing.assert_array_equal(twist.angular, np.zeros(3))


class TestPolicyManagerDispatch:
    def test_from_config(self) -> None:
        policy = MetisPolicyManager.from_config(CircleEEVelocityPolicyConfig())
        assert isinstance(policy, CircleEEVelocityPolicy)

    def test_config_from_yaml(self) -> None:
        config = MetisPolicyManager.config_from_yaml_dict(
            {"type": "circle_ee_velocity", "radius": 0.07, "velocity_magnitude": 0.04, "duration_seconds": 6.0}
        )
        assert isinstance(config, CircleEEVelocityPolicyConfig)
        assert config.radius == 0.07
        assert config.velocity_magnitude == 0.04
        assert config.duration_seconds == 6.0


if __name__ == "__main__":
    run_manor_tests()
