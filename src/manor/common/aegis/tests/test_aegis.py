"""
Smoke tests for the top-level Aegis diagram builder.

These verify that the graph wires up and ticks in both SIM and HARDWARE
modes. They do not assert behavioural correctness: the backends are stubs
and the identity policy produces zero-joint actions from zero-joint
proprioception. Success here means the messages flow through the full
Helios / Talos / Metis / Kyber cycle without schema mismatches or
algebraic-loop errors.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram

from manor.common.aegis.aegis import AegisFrequencies, AegisSystems, build_aegis
from manor.common.aegis.metis.policies import IdentityPolicy
from manor.common.aegis.mode import AegisMode
from manor.common.testing_utils import run_manor_tests


def _fast_freqs() -> AegisFrequencies:
    # Turn everything down so AdvanceTo doesn't have to churn too many events
    # for a smoke test while still exercising the periodic-update plumbing.
    return AegisFrequencies(helios_hz=10.0, talos_hz=20.0, metis_hz=10.0, kyber_hz=50.0)


class TestBuildAegis:
    @pytest.mark.parametrize("mode", [AegisMode.SIM, AegisMode.HARDWARE])
    def test_build_returns_diagram_and_systems(self, mode: AegisMode) -> None:
        diagram, systems = build_aegis(mode=mode, policy=IdentityPolicy(), frequencies=_fast_freqs())
        assert isinstance(diagram, Diagram)
        assert isinstance(systems, AegisSystems)
        assert systems.helios.get_name() == "helios"
        assert systems.talos.get_name() == "talos"
        assert systems.metis.get_name() == "metis"
        assert systems.kyber.get_name() == "kyber"

    @pytest.mark.parametrize("mode", [AegisMode.SIM, AegisMode.HARDWARE])
    def test_diagram_advances_without_error(self, mode: AegisMode) -> None:
        diagram, _ = build_aegis(mode=mode, policy=IdentityPolicy(), frequencies=_fast_freqs())
        context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(0.2)


if __name__ == "__main__":
    run_manor_tests()
