"""
Smoke tests for the top-level Aegis diagram builder.

These verify that the graph wires up and ticks in both SIM and HARDWARE
modes. They do not assert behavioural correctness: the backends are
stubs and the policy / controller produce zero-velocity actions.
Success here means the messages flow through the full Helios / Talos /
Metis / Kyber cycle (including LCM publisher / subscriber adapters)
without schema mismatches or algebraic-loop errors.

YAML round-trip coverage lives at the bottom: the bundled
``configs/aegis/lite6_ac.yaml`` must load cleanly, and a
top-level dict round-trip must produce a buildable diagram.
"""

from __future__ import annotations

import os

import attr
import numpy as np
import pytest
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram

from manor.common.aegis.aegis import (
    AegisConfig,
    AegisSystems,
    build_aegis,
)
from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.aegis.gaia.gaia import GaiaConfig
from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancerConfig
from manor.common.aegis.helios.helios import HeliosConfig
from manor.common.aegis.kyber.controllers.zero_velocity_controller import ZeroVelocityControllerConfig
from manor.common.aegis.kyber.kyber import KyberConfig
from manor.common.aegis.metis.metis import MetisConfig
from manor.common.aegis.metis.policies.constant_policies import ConstantJointVelocitiesPolicyConfig
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackendConfig
from manor.common.aegis.talos.talos import TalosConfig
from manor.common.exceptions import AegisConfigError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.driver import Lite6DriverConfig
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant

# Sentinel speed / accel limits for the hardware-mode build smoke tests. Required fields on
# Lite6DriverConfig (no defaults), included in every hardware_backend_config the tests construct.
_TEST_JOINT_SPEED_LIMIT_RAD_S = 1.0
_TEST_JOINT_ACC_LIMIT_RAD_S2 = 2.0


def _fast_metis_config() -> MetisConfig:
    return MetisConfig(
        publish_frequency_hz=10.0,
        policy_config=ConstantJointVelocitiesPolicyConfig(velocities=np.zeros(LITE6_ARM_DOF, dtype=np.float64)),
    )


def _fast_kyber_config() -> KyberConfig:
    return KyberConfig(
        publish_frequency_hz=50.0,
        controller_config=ZeroVelocityControllerConfig(num_dof=LITE6_ARM_DOF),
    )


def _fast_config(mode: AegisMode) -> AegisConfig:
    return AegisConfig(
        mode=mode,
        manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL),
        metis_config=_fast_metis_config(),
        kyber_config=_fast_kyber_config(),
        environment_config=EnvironmentConfig(),
        helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=10.0),
        talos_config=TalosConfig(
            publish_frequency_hz=20.0,
            hardware_backend_config=HardwareManipulatorBackendConfig(
                minimum_watchdog_frequency_hz=3.0,
                lite6_driver_config=Lite6DriverConfig(
                    joint_speed_limit_rad_s=_TEST_JOINT_SPEED_LIMIT_RAD_S,
                    joint_acc_limit_rad_s2=_TEST_JOINT_ACC_LIMIT_RAD_S2,
                ),
            ),
        ),
        gaia_advancer_config=GaiaAdvancerConfig(),
        gaia_config=GaiaConfig(),
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
        base = _fast_config(AegisMode.SIM)
        config = attr.evolve(
            base,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=0.0, publish_depth_frequency_hz=0.0),
        )
        diagram, systems = build_aegis(config)
        assert isinstance(diagram, Diagram)
        assert systems.helios.num_output_ports() == 0

    def test_rgb_only_helios_omits_depth(self) -> None:
        base = _fast_config(AegisMode.SIM)
        config = attr.evolve(
            base,
            helios_config=HeliosConfig(publish_rgb_frequency_hz=10.0, publish_depth_frequency_hz=0.0),
        )
        _, systems = build_aegis(config)
        assert systems.helios.num_output_ports() == 1


def _full_yaml_dict(mode: str = "sim") -> dict:
    return {
        "mode": mode,
        "manipulator_model": {"type": "lite6", "variant": "parallel_gripper_normal"},
        "metis_config": {
            "publish_frequency_hz": 10.0,
            "policy_config": {
                "type": "constant_joint_velocities",
                "velocities": [0.0] * LITE6_ARM_DOF,
            },
        },
        "kyber_config": {
            "publish_frequency_hz": 50.0,
            "controller_config": {"type": "zero_velocity", "num_dof": LITE6_ARM_DOF},
        },
        "environment_config": {},
        "helios_config": {},
        "talos_config": {
            "hardware_backend_config": {
                "minimum_watchdog_frequency_hz": 3.0,
                "lite6_driver_config": {
                    "joint_speed_limit_rad_s": _TEST_JOINT_SPEED_LIMIT_RAD_S,
                    "joint_acc_limit_rad_s2": _TEST_JOINT_ACC_LIMIT_RAD_S2,
                },
            }
        },
        "gaia_advancer_config": {},
        "gaia_config": {},
    }


class TestAegisYamlDict:
    def test_full_round_trip_sim(self) -> None:
        config = AegisConfig.from_yaml_dict(_full_yaml_dict(mode="sim"))
        assert config.mode is AegisMode.SIM
        assert isinstance(config.metis_config.policy_config, ConstantJointVelocitiesPolicyConfig)
        assert isinstance(config.kyber_config.controller_config, ZeroVelocityControllerConfig)
        # Required blocks are now always populated; inner fields default cleanly.
        assert isinstance(config.environment_config, EnvironmentConfig)
        assert isinstance(config.gaia_config, GaiaConfig)

    def test_full_block_with_environment_and_helios(self) -> None:
        d = _full_yaml_dict()
        d["environment_config"] = {
            "manipulator_base_xyz": [0.0, 0.0, 0.7366],
            "extra_models": [],
        }
        d["helios_config"] = {"publish_rgb_frequency_hz": 5.0, "publish_depth_frequency_hz": 0.0}
        config = AegisConfig.from_yaml_dict(d)
        assert config.helios_config.publish_rgb_frequency_hz == 5.0
        assert config.helios_config.publish_depth_frequency_hz == 0.0

    @pytest.mark.parametrize(
        "missing_key",
        [
            "metis_config",
            "kyber_config",
            "environment_config",
            "helios_config",
            "talos_config",
            "gaia_advancer_config",
            "gaia_config",
        ],
    )
    def test_rejects_missing_required_block(self, missing_key: str) -> None:
        d = _full_yaml_dict()
        del d[missing_key]
        with pytest.raises(AegisConfigError):
            AegisConfig.from_yaml_dict(d)

    def test_rejects_unknown_top_level_keys(self) -> None:
        d = _full_yaml_dict()
        d["bogus"] = 1
        with pytest.raises(AegisConfigError):
            AegisConfig.from_yaml_dict(d)

    def test_rejects_unknown_mode(self) -> None:
        d = _full_yaml_dict(mode="not_a_mode")
        with pytest.raises(AegisConfigError):
            AegisConfig.from_yaml_dict(d)

    def test_rejects_unknown_manipulator_type(self) -> None:
        d = _full_yaml_dict()
        d["manipulator_model"] = {"type": "whatever", "variant": "parallel_gripper_normal"}
        with pytest.raises(AegisConfigError):
            AegisConfig.from_yaml_dict(d)

    def test_rejects_unknown_variant(self) -> None:
        d = _full_yaml_dict()
        d["manipulator_model"] = {"type": "lite6", "variant": "purple_gripper"}
        with pytest.raises(AegisConfigError):
            AegisConfig.from_yaml_dict(d)


class TestBundledLite6DefaultYaml:
    def test_bundled_yaml_round_trips_and_builds(self) -> None:
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
        path = os.path.join(repo_root, "configs", "aegis", "lite6_ac.yaml")
        assert os.path.exists(path), f"bundled lite6 yaml missing at {path}"
        config = AegisConfig.from_yaml(path)
        # Force meshcat off for the smoke test -- the bundled YAML pins
        # the meshcat server to port 7000, which collides with other
        # tests in the same suite that also load the bundled YAML.
        config = attr.evolve(config, gaia_config=attr.evolve(config.gaia_config, enable_meshcat=False))
        diagram, systems = build_aegis(config)
        assert isinstance(diagram, Diagram)
        # gaia presence is mode-dependent: sim mode owns a gaia plant; hardware mode skips it. The
        # bundled YAML's mode flag is the source of truth -- this test just verifies the YAML
        # round-trips and the resulting AegisSystems matches what build_aegis produces for that mode.
        if config.mode is AegisMode.SIM:
            assert systems.gaia is not None
        else:
            assert systems.gaia is None


if __name__ == "__main__":
    run_manor_tests()
