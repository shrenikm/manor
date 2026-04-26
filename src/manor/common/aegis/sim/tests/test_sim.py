"""
Tests for the Sim Python class.
"""

from __future__ import annotations

import numpy as np
import pytest

from manor.common.aegis.sim.env_config import EnvironmentConfig
from manor.common.aegis.sim.sim import Sim, SimConfig
from manor.common.exceptions import SimError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_sim(**overrides) -> Sim:
    defaults = dict(
        manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
        environment_config=EnvironmentConfig.default(),
        config=SimConfig(),
    )
    defaults.update(overrides)
    return Sim(**defaults)


class TestConstruction:
    def test_builds_finalized_plant(self) -> None:
        sim = _make_sim()
        assert sim.plant.is_finalized()

    def test_plant_dof_matches_manipulator_model(self) -> None:
        sim = _make_sim()
        assert sim.plant.num_positions() == sim.manipulator_model.get_num_positions()


class TestAdvanceTo:
    def test_advances_internal_simulator(self) -> None:
        sim = _make_sim()
        sim.advance_to(0.05)
        assert sim.simulator.get_context().get_time() > 0.0

    def test_advance_to_past_time_is_noop(self) -> None:
        sim = _make_sim()
        sim.advance_to(0.05)
        sim.advance_to(0.02)
        assert sim.simulator.get_context().get_time() >= 0.05


class TestCommandStash:
    def test_apply_position_command_is_stored(self) -> None:
        sim = _make_sim()
        positions = np.linspace(0.0, 0.5, sim.plant.num_positions())
        sim.apply_joint_position_command(positions)
        np.testing.assert_array_equal(sim.get_latest_position_command(), positions)

    def test_apply_velocity_command_is_stored(self) -> None:
        sim = _make_sim()
        velocities = np.full(sim.plant.num_velocities(), 0.1)
        sim.apply_joint_velocity_command(velocities)
        np.testing.assert_array_equal(sim.get_latest_velocity_command(), velocities)


class TestReadJointState:
    def test_returns_joint_state_with_correct_shape(self) -> None:
        sim = _make_sim()
        js = sim.read_joint_state()
        assert js.joint_positions.positions.shape == (sim.plant.num_positions(),)
        assert js.joint_velocities.velocities.shape == (sim.plant.num_velocities(),)


class TestSetJointPositions:
    def test_snaps_plant_state(self) -> None:
        sim = _make_sim()
        positions = np.full(sim.plant.num_positions(), 0.25)
        sim.set_joint_positions(positions)
        np.testing.assert_array_equal(sim.read_joint_state().joint_positions.positions, positions)

    def test_rejects_wrong_shape(self) -> None:
        sim = _make_sim()
        with pytest.raises(SimError):
            sim.set_joint_positions(np.zeros(3))


class TestRender:
    def test_render_rgb_returns_default_frame_with_fresh_header(self) -> None:
        sim = _make_sim()
        rgb = sim.render_rgb()
        assert rgb.height == sim._config.rgb_height
        assert rgb.width == sim._config.rgb_width
        assert rgb.header.monotonic_ns > 0

    def test_render_depth_returns_default_frame_with_fresh_header(self) -> None:
        sim = _make_sim()
        depth = sim.render_depth()
        assert depth.height == sim._config.depth_height
        assert depth.width == sim._config.depth_width
        assert depth.header.monotonic_ns > 0


if __name__ == "__main__":
    run_manor_tests()
