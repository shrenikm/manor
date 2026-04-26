"""
Smoke tests for the top-level Aegis diagram builder.

These verify that the graph wires up and ticks in both SIM and HARDWARE
modes. They do not assert behavioural correctness: the backends are
stubs and the policy / controller produce zero-velocity actions.
Success here means the messages flow through the full Helios / Talos /
Metis / Kyber cycle (including LCM publisher / subscriber adapters)
without schema mismatches or algebraic-loop errors.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram

from manor.common.aegis.aegis import (
    AegisBuildConfig,
    AegisSystemName,
    AegisSystems,
    build_aegis,
)
from manor.common.aegis.helios.helios import HeliosConfig
from manor.common.aegis.kyber.kyber import KyberConfig
from manor.common.aegis.metis.metis import MetisConfig
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.talos import TalosConfig
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.variant import Lite6Variant


def _fast_config(mode: AegisMode) -> AegisBuildConfig:
    base = AegisBuildConfig.from_lite6(mode, Lite6Variant.PARALLEL_GRIPPER_NORMAL)
    return AegisBuildConfig(
        mode=base.mode,
        manipulator_model=base.manipulator_model,
        helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=10.0),
        talos_config=TalosConfig(publish_frequency_hz=20.0),
        metis_config=MetisConfig(publish_frequency_hz=10.0),
        kyber_config=KyberConfig(publish_frequency_hz=50.0),
    )


def _build(mode: AegisMode) -> tuple[Diagram, AegisSystems]:
    return build_aegis(_fast_config(mode))


class TestBuildAegis:
    @pytest.mark.parametrize("mode", [AegisMode.SIM, AegisMode.HARDWARE])
    def test_build_returns_diagram_and_systems(self, mode: AegisMode) -> None:
        diagram, systems = _build(mode)
        assert isinstance(diagram, Diagram)
        assert isinstance(systems, AegisSystems)
        assert systems.helios.get_name() == AegisSystemName.HELIOS
        assert systems.talos.get_name() == AegisSystemName.TALOS
        assert systems.metis.get_name() == AegisSystemName.METIS
        assert systems.kyber.get_name() == AegisSystemName.KYBER

    def test_sim_mode_attaches_gaia_and_advancer(self) -> None:
        _, systems = _build(AegisMode.SIM)
        assert systems.gaia is not None
        assert systems.gaia_advancer is not None
        assert systems.gaia_advancer.get_name() == AegisSystemName.GAIA_ADVANCER

    def test_hardware_mode_has_no_gaia(self) -> None:
        _, systems = _build(AegisMode.HARDWARE)
        assert systems.gaia is None
        assert systems.gaia_advancer is None

    def test_sim_diagram_advances_without_error(self) -> None:
        diagram, _ = _build(AegisMode.SIM)
        context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(0.1)


class TestHeliosFrequencyFlags:
    def test_dummy_helios_builds_without_image_adapters(self) -> None:
        base = AegisBuildConfig.from_lite6(AegisMode.SIM, Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        config = AegisBuildConfig(
            mode=base.mode,
            manipulator_model=base.manipulator_model,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=0.0, publish_depth_frequency_hz=0.0),
            talos_config=TalosConfig(publish_frequency_hz=20.0),
            metis_config=MetisConfig(publish_frequency_hz=10.0),
            kyber_config=KyberConfig(publish_frequency_hz=50.0),
        )
        diagram, systems = build_aegis(config)
        assert isinstance(diagram, Diagram)
        assert systems.helios.num_output_ports() == 0

    def test_rgb_only_helios_omits_depth(self) -> None:
        base = AegisBuildConfig.from_lite6(AegisMode.SIM, Lite6Variant.PARALLEL_GRIPPER_NORMAL)
        config = AegisBuildConfig(
            mode=base.mode,
            manipulator_model=base.manipulator_model,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=0.0),
            talos_config=TalosConfig(publish_frequency_hz=20.0),
            metis_config=MetisConfig(publish_frequency_hz=10.0),
            kyber_config=KyberConfig(publish_frequency_hz=50.0),
        )
        _, systems = build_aegis(config)
        assert systems.helios.num_output_ports() == 1


if __name__ == "__main__":
    run_manor_tests()
