"""
Smoke tests for the per-block aegis runners.

These verify each runner builds its diagram and the resulting Drake
``Simulator`` advances cleanly. The end-to-end "metis + gylos talk
to each other over LCM" path is exercised via the CLI tests; these
are pure builder smoke tests.
"""

from __future__ import annotations

import os

import attr
import pytest
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis import AegisConfig
from manor.common.aegis.aegis_adapters import (
    AegisAdapterPorts,
    AegisLCMPublisherAdapter,
    AegisLCMSubscriberAdapter,
)
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerType
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyManager, MetisPolicyType
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.testing_utils import run_manor_tests


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", ".."))


def _bundled_config(
    filename: str = "lite6_ac.yaml",
    *,
    policy_type: MetisPolicyType | None = None,
    controller_type: KyberControllerType | None = None,
) -> AegisConfig:
    """
    Load the bundled aegis base YAML through the composer, optionally
    swapping in a different policy / controller type for the load.
    Used by the per-policy / per-controller round-trip tests so each
    bundled sub-YAML is exercised end-to-end. The swap goes through
    a temp YAML written next to the base so the composer's
    sibling-directory resolution still finds ``policies/`` and
    ``controllers/``.
    """
    import tempfile

    import yaml as _yaml

    from manor.common.aegis.aegis import compose_aegis_yaml_dict

    path = os.path.join(_repo_root(), "configs", "aegis", filename)
    if policy_type is None and controller_type is None:
        config = AegisConfig.from_yaml(path)
    else:
        with open(path, "r") as fp:
            base = _yaml.safe_load(fp) or {}
        if policy_type is not None:
            base["metis_config"]["policy_type"] = policy_type.value
        if controller_type is not None:
            base["kyber_config"]["controller_type"] = controller_type.value
        with tempfile.NamedTemporaryFile(
            mode="w",
            suffix="_ac.yaml",
            dir=os.path.dirname(path),
            delete=False,
        ) as fp:
            _yaml.safe_dump(base, fp)
            tmp_path = fp.name
        try:
            raw = compose_aegis_yaml_dict(tmp_path)
            config = AegisConfig.from_yaml_dict(raw)
        finally:
            os.unlink(tmp_path)
    # Disable meshcat for tests so the pinned port-7000 server doesn't
    # collide with other tests that also load the bundled YAML.
    return attr.evolve(config, gaia_config=attr.evolve(config.gaia_config, enable_meshcat=False))


def _build_metis_diagram(config: AegisConfig, lcm: DrakeLcm):
    """
    Mirror ``run_metis`` up to (but not including) ``advance_until_signal``.
    Tests use this to assert the diagram builds cleanly without
    spinning a wall-clock-paced loop.
    """
    metis_config = config.metis_config
    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))
    policy = MetisPolicyManager.from_config(metis_config.policy_config, manipulator_model=config.manipulator_model)
    metis = builder.AddSystem(Metis(policy=policy, publish_frequency=metis_config.publish_frequency_hz))
    metis.set_name(MetisConfig.SYSTEM_NAME)

    proprioception_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=Proprioception, channel=AegisChannel.PROPRIOCEPTION, lcm=lcm
        )
    )
    rgb_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(definition_cls=RGBImageData, channel=AegisChannel.RGB_IMAGE, lcm=lcm)
    )
    depth_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=DepthImageData, channel=AegisChannel.DEPTH_IMAGE, lcm=lcm
        )
    )
    action_publisher = builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=Action,
            channel=AegisChannel.ACTION,
            lcm=lcm,
            publish_period=1.0 / metis_config.publish_frequency_hz,
        )
    )
    builder.Connect(
        proprioception_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        rgb_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE),
    )
    builder.Connect(
        depth_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE),
    )
    builder.Connect(
        metis.GetOutputPort(MetisPorts.OUTPUT_ACTION),
        action_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    return builder.Build()


class TestMetisRunner:
    def test_diagram_builds_and_advances(self) -> None:
        config = _bundled_config()
        diagram = _build_metis_diagram(config, DrakeLcm())
        simulator = Simulator(diagram)
        simulator.Initialize()
        # Tiny advance is enough to confirm there are no algebraic-loop
        # or wiring errors; we don't care about LCM message flow here.
        simulator.AdvanceTo(0.05)


class TestGylosRunner:
    def test_run_gylos_function_builds(self) -> None:
        # Import lazily so a hard failure in gylos's import path
        # surfaces as a test failure rather than a collection error.
        from manor.common.aegis.run.run_gylos import run_gylos  # noqa: F401

    def test_gylos_diagram_builds_and_advances(self) -> None:
        # Reproduce gylos's wiring up to the simulator init step.
        from manor.common.aegis.gaia.gaia import Gaia
        from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancer, GaiaAdvancerConfig
        from manor.common.aegis.helios.helios import Helios, HeliosConfig, HeliosPorts
        from manor.common.aegis.helios.sim_backend import SimSensorBackend
        from manor.common.aegis.kyber.controllers.controller_manager import KyberControllerManager
        from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
        from manor.common.aegis.talos.sim_backend import SimManipulatorBackend
        from manor.common.aegis.talos.talos import Talos, TalosConfig, TalosPorts

        config = _bundled_config()
        lcm = DrakeLcm()
        gaia = Gaia(
            manipulator_model=config.manipulator_model,
            environment_config=config.environment_config,
            config=config.gaia_config,
        )
        gaia.finalize()

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))
        helios = builder.AddSystem(
            Helios(
                backend=SimSensorBackend(gaia=gaia, config=config.helios_config.sim_backend_config),
                publish_rgb_frequency_hz=config.helios_config.publish_rgb_frequency_hz,
                publish_depth_frequency_hz=config.helios_config.publish_depth_frequency_hz,
            )
        )
        helios.set_name(HeliosConfig.SYSTEM_NAME)
        talos = builder.AddSystem(
            Talos(
                backend=SimManipulatorBackend(gaia=gaia, config=config.talos_config.sim_backend_config),
                manipulator_model=config.manipulator_model,
                publish_frequency=config.talos_config.publish_frequency_hz,
            )
        )
        talos.set_name(TalosConfig.SYSTEM_NAME)
        controller = KyberControllerManager.from_config(
            config.kyber_config.controller_config, manipulator_model=config.manipulator_model
        )
        kyber = builder.AddSystem(
            Kyber(controller=controller, publish_frequency=config.kyber_config.publish_frequency_hz)
        )
        kyber.set_name(KyberConfig.SYSTEM_NAME)
        advancer = builder.AddSystem(
            GaiaAdvancer(gaia=gaia, advance_frequency_hz=config.gaia_advancer_config.advance_frequency_hz)
        )
        advancer.set_name(GaiaAdvancerConfig.SYSTEM_NAME)

        action_subscriber = builder.AddSystem(
            AegisLCMSubscriberAdapter.from_lcm_type(definition_cls=Action, channel=AegisChannel.ACTION, lcm=lcm)
        )
        builder.Connect(
            action_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
            kyber.GetInputPort(KyberPorts.INPUT_ACTION),
        )
        builder.Connect(
            talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
            kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
        )
        builder.Connect(
            kyber.GetOutputPort(KyberPorts.OUTPUT_JOINT_EE_COMMAND),
            talos.GetInputPort(TalosPorts.INPUT_JOINT_EE_COMMAND),
        )
        rgb_publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=RGBImageData,
                channel=AegisChannel.RGB_IMAGE,
                lcm=lcm,
                publish_period=1.0 / config.helios_config.publish_rgb_frequency_hz,
            )
        )
        builder.Connect(
            helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE),
            rgb_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        proprioception_publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=Proprioception,
                channel=AegisChannel.PROPRIOCEPTION,
                lcm=lcm,
                publish_period=1.0 / config.talos_config.publish_frequency_hz,
            )
        )
        builder.Connect(
            talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
            proprioception_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )

        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.Initialize()
        simulator.AdvanceTo(0.05)


@pytest.mark.parametrize(
    "module",
    [
        "manor.common.aegis.run.run_metis",
        "manor.common.aegis.run.run_gylos",
        "manor.common.aegis.run.run_helios",
        "manor.common.aegis.run.run_kylos",
    ],
)
def test_runner_module_imports_cleanly(module: str) -> None:
    __import__(module)


@pytest.mark.parametrize("policy_type", list(MetisPolicyType))
def test_every_bundled_policy_yaml_parses(policy_type: MetisPolicyType) -> None:
    # Each MetisPolicyType must have a matching ``policies/<name>_ac.yaml``
    # body that round-trips through the composer + AegisConfig parser
    # cleanly. Catches schema drift between policy configs and their
    # bundled YAMLs without hand-maintaining a parametrize list.
    config = _bundled_config(policy_type=policy_type)
    assert config.metis_config.policy_config is not None
    assert config.metis_config.policy_config.POLICY_TYPE is policy_type


@pytest.mark.parametrize("controller_type", list(KyberControllerType))
def test_every_bundled_controller_yaml_parses(controller_type: KyberControllerType) -> None:
    # Mirror of the policy version: each KyberControllerType must
    # have a matching ``controllers/<name>_ac.yaml`` body that
    # round-trips through the composer.
    config = _bundled_config(controller_type=controller_type)
    assert config.kyber_config.controller_config is not None
    assert config.kyber_config.controller_config.CONTROLLER_TYPE is controller_type


if __name__ == "__main__":
    run_manor_tests()
