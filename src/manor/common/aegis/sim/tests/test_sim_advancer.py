"""
Tests for SimAdvancer.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator

from manor.common.aegis.sim.sim import Sim
from manor.common.aegis.sim.sim_advancer import SimAdvancer
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_sim() -> Sim:
    return Sim(manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL))


class TestConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        sim = _make_sim()
        with pytest.raises(ValueError):
            SimAdvancer(sim=sim, advance_frequency_hz=0.0)

    def test_stores_sim_and_frequency(self) -> None:
        sim = _make_sim()
        advancer = SimAdvancer(sim=sim, advance_frequency_hz=200.0)
        assert advancer.sim is sim
        assert advancer.advance_frequency_hz == 200.0


class TestPeriodicUpdate:
    def test_advances_sim_clock_in_step_with_diagram(self) -> None:
        sim = _make_sim()
        advancer = SimAdvancer(sim=sim, advance_frequency_hz=200.0)

        simulator = Simulator(advancer)
        simulator.AdvanceTo(0.05)

        # After AdvanceTo, the Sim's internal clock should have moved
        # forward (to the latest periodic-update time, ~0.05s).
        assert sim.simulator.get_context().get_time() > 0.0


if __name__ == "__main__":
    run_manor_tests()
