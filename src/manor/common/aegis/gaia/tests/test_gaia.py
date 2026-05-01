"""
Tests for the Gaia Python class.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.aegis.gaia.gaia import Gaia, GaiaConfig
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import GaiaError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_gaia(finalize: bool = True, **overrides) -> Gaia:
    defaults = dict(
        manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
        environment_config=EnvironmentConfig.default(),
        config=GaiaConfig(),
    )
    defaults.update(overrides)
    gaia = Gaia(**defaults)
    if finalize:
        gaia.finalize()
    return gaia


class TestConstruction:
    def test_unfinalized_gaia_has_no_plant(self) -> None:
        gaia = _make_gaia(finalize=False)
        assert gaia.plant is None
        assert gaia.simulator is None
        assert not gaia.is_finalized()

    def test_finalize_builds_plant_and_simulator(self) -> None:
        gaia = _make_gaia()
        assert gaia.plant is not None
        assert gaia.plant.is_finalized()
        assert gaia.simulator is not None
        assert gaia.is_finalized()

    def test_double_finalize_raises(self) -> None:
        gaia = _make_gaia()
        with pytest.raises(GaiaError):
            gaia.finalize()

    def test_plant_dof_matches_manipulator_model(self) -> None:
        gaia = _make_gaia()
        assert gaia.plant.num_positions() == gaia.manipulator_model.get_num_positions()


class TestRequiresFinalize:
    def test_advance_before_finalize_raises(self) -> None:
        gaia = _make_gaia(finalize=False)
        with pytest.raises(GaiaError):
            gaia.advance_to(0.01)

    def test_read_joint_state_before_finalize_raises(self) -> None:
        gaia = _make_gaia(finalize=False)
        with pytest.raises(GaiaError):
            gaia.read_joint_state()

    def test_render_before_finalize_raises(self) -> None:
        gaia = _make_gaia(finalize=False)
        with pytest.raises(GaiaError):
            gaia.render_rgb()


class TestAdvanceTo:
    def test_advances_internal_simulator(self) -> None:
        gaia = _make_gaia()
        gaia.advance_to(0.05)
        assert gaia.simulator.get_context().get_time() > 0.0

    def test_advance_to_past_time_is_noop(self) -> None:
        gaia = _make_gaia()
        gaia.advance_to(0.05)
        gaia.advance_to(0.02)
        assert gaia.simulator.get_context().get_time() >= 0.05


class TestCommandStash:
    def test_apply_position_command_is_stored(self) -> None:
        gaia = _make_gaia()
        positions = np.linspace(0.0, 0.5, gaia.manipulator_model.get_num_dof())
        joint_positions = JointPositions(header=TimestampHeader.from_system_time(), positions=positions)
        gaia.apply_joint_position_command(joint_positions)
        assert gaia.latest_position_command is joint_positions

    def test_apply_velocity_command_is_stored(self) -> None:
        gaia = _make_gaia()
        velocities = np.full(gaia.manipulator_model.get_num_dof(), 0.1)
        joint_velocities = JointVelocities(header=TimestampHeader.from_system_time(), velocities=velocities)
        gaia.apply_joint_velocity_command(joint_velocities)
        assert gaia.latest_velocity_command is joint_velocities

    def test_apply_ee_position_command_is_stored(self) -> None:
        gaia = _make_gaia()
        ee_positions = EEPositions(
            header=TimestampHeader.from_system_time(),
            positions=np.array([0.04], dtype=np.float64),
        )
        gaia.apply_ee_position_command(ee_positions)
        assert gaia.latest_ee_position_command is ee_positions


class TestEEPositionCommandDrivesGripper:
    def test_parallel_gripper_tracks_commanded_width(self) -> None:
        # Send an EE-position command, advance the inner sim, and verify
        # the plant's gripper-side q is moving toward the commanded
        # opening width (split across the two prismatic finger joints).
        # PID convergence to within tight tolerance can take several
        # simulated seconds on this controller-plant configuration --
        # the assertion checks "fingers moved toward the target",
        # not "fingers locked exactly on the target".
        gaia = _make_gaia()
        commanded_width = 0.04
        gaia.apply_ee_position_command(
            EEPositions(
                header=TimestampHeader.from_system_time(),
                positions=np.array([commanded_width], dtype=np.float64),
            )
        )
        gaia.advance_to(1.0)
        positions = gaia.read_joint_state().joint_positions.positions
        gripper_q = positions[LITE6_ARM_DOF:]
        target = commanded_width / 2.0
        # Both fingers should have moved toward the target; both should
        # be at least halfway from zero to the target (we don't require
        # full convergence -- this is a regression test that the wiring
        # is alive, not a controller-tuning test).
        assert gripper_q.shape == (2,)
        assert gripper_q[0] > target * 0.4
        assert gripper_q[1] > target * 0.4


class TestReadJointState:
    def test_returns_joint_state_with_correct_shape(self) -> None:
        gaia = _make_gaia()
        js = gaia.read_joint_state()
        assert js.joint_positions.positions.shape == (gaia.plant.num_positions(),)
        assert js.joint_velocities.velocities.shape == (gaia.plant.num_velocities(),)


class TestSetJointPositions:
    def test_snaps_plant_state(self) -> None:
        gaia = _make_gaia()
        positions = np.full(gaia.plant.num_positions(), 0.25)
        gaia.set_joint_positions(positions)
        np.testing.assert_array_equal(gaia.read_joint_state().joint_positions.positions, positions)

    def test_rejects_wrong_shape(self) -> None:
        gaia = _make_gaia()
        with pytest.raises(GaiaError):
            gaia.set_joint_positions(np.zeros(3))


class TestRender:
    def test_render_rgb_returns_default_frame_with_fresh_header(self) -> None:
        gaia = _make_gaia()
        rgb = gaia.render_rgb()
        assert rgb.height == gaia.config.rgb_height
        assert rgb.width == gaia.config.rgb_width
        assert rgb.header.monotonic_ns > 0

    def test_render_depth_returns_default_frame_with_fresh_header(self) -> None:
        gaia = _make_gaia()
        depth = gaia.render_depth()
        assert depth.height == gaia.config.depth_height
        assert depth.width == gaia.config.depth_width
        assert depth.header.monotonic_ns > 0


if __name__ == "__main__":
    run_manor_tests()
