"""
Smoke tests for the top-level Aegis diagram builder.

These verify that the graph wires up and ticks in both SIM and HARDWARE
modes. They do not assert behavioural correctness: the backends are
stubs and the policy produces zero-velocity actions. Success here
means the messages flow through the full Helios / Talos / Metis /
Kyber cycle (including LCM publisher / subscriber adapters) without
schema mismatches or algebraic-loop errors.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram

from manor.common.aegis.aegis import AegisBuildConfig, AegisFrequencies, AegisSystems, build_aegis
from manor.common.aegis.mode import AegisMode
from manor.common.testing_utils import run_manor_tests


def _fast_freqs() -> AegisFrequencies:
    # Drop frequencies so AdvanceTo doesn't churn too many events for
    # a smoke test, while still exercising the periodic-update plumbing.
    return AegisFrequencies(
        helios_hz=10.0,
        talos_hz=20.0,
        metis_hz=10.0,
        kyber_hz=50.0,
        sim_advancer_hz=100.0,
    )


def _build(mode: AegisMode) -> tuple[Diagram, AegisSystems]:
    config = AegisBuildConfig.default_lite6(mode)
    return build_aegis(
        AegisBuildConfig(
            mode=config.mode,
            manipulator_model=config.manipulator_model,
            frequencies=_fast_freqs(),
        )
    )


class TestBuildAegis:
    @pytest.mark.parametrize("mode", [AegisMode.SIM, AegisMode.HARDWARE])
    def test_build_returns_diagram_and_systems(self, mode: AegisMode) -> None:
        diagram, systems = _build(mode)
        assert isinstance(diagram, Diagram)
        assert isinstance(systems, AegisSystems)
        assert systems.helios.get_name() == "helios"
        assert systems.talos.get_name() == "talos"
        assert systems.metis.get_name() == "metis"
        assert systems.kyber.get_name() == "kyber"

    def test_sim_mode_attaches_sim_and_advancer(self) -> None:
        _, systems = _build(AegisMode.SIM)
        assert systems.sim is not None
        assert systems.sim_advancer is not None
        assert systems.sim_advancer.get_name() == "sim_advancer"

    def test_hardware_mode_has_no_sim(self) -> None:
        _, systems = _build(AegisMode.HARDWARE)
        assert systems.sim is None
        assert systems.sim_advancer is None

    def test_sim_diagram_advances_without_error(self) -> None:
        # Hardware mode requires a real (or mocked) ``Lite6Driver`` that's
        # been primed before AdvanceTo can succeed; only SIM is exercisable
        # end-to-end without external state, so the smoke test stays here.
        diagram, _ = _build(AegisMode.SIM)
        context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(0.1)


if __name__ == "__main__":
    run_manor_tests()
