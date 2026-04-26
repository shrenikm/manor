"""
Smoke tests for the top-level Aegis diagram builder.

These verify that the graph wires up and ticks in both SIM and HARDWARE
modes. They do not assert behavioural correctness: the backends are
stubs and the policy / controller produce zero-velocity actions.
Success here means the messages flow through the full Helios / Talos /
Metis / Kyber cycle (including LCM publisher / subscriber adapters)
without schema mismatches or algebraic-loop errors.

YAML round-trip coverage lives at the bottom: the bundled
``configs/aegis/lite6_default.yaml`` must load cleanly, and a
top-level dict round-trip must produce a buildable diagram.
"""

from __future__ import annotations

import os

import pytest
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram

from manor.common.aegis.aegis import (
    AegisBuildConfig,
    AegisSystems,
    build_aegis,
)
from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancerConfig
from manor.common.aegis.helios.helios import HeliosConfig
from manor.common.aegis.kyber.controllers.zero_velocity_controller import ZeroVelocityControllerConfig
from manor.common.aegis.kyber.kyber import KyberConfig
from manor.common.aegis.metis.metis import MetisConfig
from manor.common.aegis.metis.policies.zero_velocity_policy import ZeroVelocityPolicyConfig
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.talos import TalosConfig
from manor.common.exceptions import AegisConfigError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import LITE6_ARM_DOF
from manor.manipulators.lite6.variant import Lite6Variant


def _fast_metis_config() -> MetisConfig:
    return MetisConfig(
        publish_frequency_hz=10.0,
        policy_config=ZeroVelocityPolicyConfig(num_joints=LITE6_ARM_DOF),
    )


def _fast_kyber_config() -> KyberConfig:
    return KyberConfig(
        publish_frequency_hz=50.0,
        controller_config=ZeroVelocityControllerConfig(num_dof=LITE6_ARM_DOF),
    )


def _fast_config(mode: AegisMode) -> AegisBuildConfig:
    base = AegisBuildConfig.from_lite6(
        mode,
        Lite6Variant.PARALLEL_GRIPPER_NORMAL,
        metis_config=_fast_metis_config(),
        kyber_config=_fast_kyber_config(),
    )
    return AegisBuildConfig(
        mode=base.mode,
        manipulator_model=base.manipulator_model,
        metis_config=base.metis_config,
        kyber_config=base.kyber_config,
        helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=10.0),
        talos_config=TalosConfig(publish_frequency_hz=20.0),
    )


def _build(mode: AegisMode) -> tuple[Diagram, AegisSystems]:
    return build_aegis(_fast_config(mode))


class TestBuildAegis:
    @pytest.mark.parametrize("mode", [AegisMode.SIM, AegisMode.HARDWARE])
    def test_build_returns_diagram_and_systems(self, mode: AegisMode) -> None:
        diagram, systems = _build(mode)
        assert isinstance(diagram, Diagram)
        assert isinstance(systems, AegisSystems)
        assert systems.helios.get_name() == HeliosConfig.SYSTEM_NAME
        assert systems.talos.get_name() == TalosConfig.SYSTEM_NAME
        assert systems.metis.get_name() == MetisConfig.SYSTEM_NAME
        assert systems.kyber.get_name() == KyberConfig.SYSTEM_NAME

    def test_sim_mode_attaches_gaia_and_advancer(self) -> None:
        _, systems = _build(AegisMode.SIM)
        assert systems.gaia is not None
        assert systems.gaia_advancer is not None
        assert systems.gaia_advancer.get_name() == GaiaAdvancerConfig.SYSTEM_NAME

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
        base = AegisBuildConfig.from_lite6(
            AegisMode.SIM,
            Lite6Variant.PARALLEL_GRIPPER_NORMAL,
            metis_config=_fast_metis_config(),
            kyber_config=_fast_kyber_config(),
        )
        config = AegisBuildConfig(
            mode=base.mode,
            manipulator_model=base.manipulator_model,
            metis_config=base.metis_config,
            kyber_config=base.kyber_config,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=0.0, publish_depth_frequency_hz=0.0),
            talos_config=TalosConfig(publish_frequency_hz=20.0),
        )
        diagram, systems = build_aegis(config)
        assert isinstance(diagram, Diagram)
        assert systems.helios.num_output_ports() == 0

    def test_rgb_only_helios_omits_depth(self) -> None:
        base = AegisBuildConfig.from_lite6(
            AegisMode.SIM,
            Lite6Variant.PARALLEL_GRIPPER_NORMAL,
            metis_config=_fast_metis_config(),
            kyber_config=_fast_kyber_config(),
        )
        config = AegisBuildConfig(
            mode=base.mode,
            manipulator_model=base.manipulator_model,
            metis_config=base.metis_config,
            kyber_config=base.kyber_config,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=0.0),
            talos_config=TalosConfig(publish_frequency_hz=20.0),
        )
        _, systems = build_aegis(config)
        assert systems.helios.num_output_ports() == 1


def _minimal_yaml_dict(mode: str = "sim") -> dict:
    return {
        "mode": mode,
        "manipulator": {"type": "lite6", "variant": "parallel_gripper_normal"},
        "metis": {
            "publish_frequency_hz": 10.0,
            "policy_config": {"type": "zero_velocity", "num_joints": LITE6_ARM_DOF},
        },
        "kyber": {
            "publish_frequency_hz": 50.0,
            "controller_config": {"type": "zero_velocity", "num_dof": LITE6_ARM_DOF},
        },
    }


class TestAegisYamlDict:
    def test_minimal_round_trip_sim(self) -> None:
        config = AegisBuildConfig.from_yaml_dict(_minimal_yaml_dict(mode="sim"))
        assert config.mode is AegisMode.SIM
        assert isinstance(config.metis_config.policy_config, ZeroVelocityPolicyConfig)
        assert isinstance(config.kyber_config.controller_config, ZeroVelocityControllerConfig)
        # Optional blocks default cleanly when absent.
        assert config.environment_config is None
        assert config.gaia_config is None

    def test_full_block_with_environment_and_helios(self) -> None:
        d = _minimal_yaml_dict()
        d["environment_config"] = {
            "manipulator_base_xyz": [0.0, 0.0, 0.7366],
            "extra_models": [],
        }
        d["helios"] = {"publish_rgb_frequency_hz": 5.0, "publish_depth_frequency_hz": 0.0}
        config = AegisBuildConfig.from_yaml_dict(d)
        assert config.environment_config is not None
        assert config.helios_config.publish_rgb_frequency_hz == 5.0
        assert config.helios_config.publish_depth_frequency_hz == 0.0

    def test_rejects_missing_metis(self) -> None:
        d = _minimal_yaml_dict()
        del d["metis"]
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)

    def test_rejects_missing_kyber(self) -> None:
        d = _minimal_yaml_dict()
        del d["kyber"]
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)

    def test_rejects_unknown_top_level_keys(self) -> None:
        d = _minimal_yaml_dict()
        d["bogus"] = 1
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)

    def test_rejects_unknown_mode(self) -> None:
        d = _minimal_yaml_dict(mode="not_a_mode")
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)

    def test_rejects_unknown_manipulator_type(self) -> None:
        d = _minimal_yaml_dict()
        d["manipulator"] = {"type": "whatever", "variant": "parallel_gripper_normal"}
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)

    def test_rejects_unknown_variant(self) -> None:
        d = _minimal_yaml_dict()
        d["manipulator"] = {"type": "lite6", "variant": "purple_gripper"}
        with pytest.raises(AegisConfigError):
            AegisBuildConfig.from_yaml_dict(d)


class TestBundledLite6DefaultYaml:
    def test_bundled_yaml_round_trips_and_builds(self) -> None:
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
        path = os.path.join(repo_root, "configs", "aegis", "lite6_default.yaml")
        assert os.path.exists(path), f"bundled default yaml missing at {path}"
        config = AegisBuildConfig.from_yaml(path)
        diagram, systems = build_aegis(config)
        assert isinstance(diagram, Diagram)
        assert systems.gaia is not None


if __name__ == "__main__":
    run_manor_tests()
