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
import yaml
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
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyManager
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.testing_utils import run_manor_tests


def _bundled_config() -> AegisConfig:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", ".."))
    path = os.path.join(repo_root, "configs", "aegis", "default_ac.yaml")
    with open(path, "r") as fp:
        config = AegisConfig.from_yaml_dict(yaml.safe_load(fp))
    # Disable meshcat for tests so the pinned port-7000 server doesn't
    # collide with other tests that also load the bundled YAML.
    return attr.evolve(config, gaia_config=attr.evolve(config.gaia_config, enable_meshcat=False))


def _build_metis_diagram(metis_config: MetisConfig, lcm: DrakeLcm):
    """
    Mirror ``run_metis`` up to (but not including) ``advance_until_signal``.
    Tests use this to assert the diagram builds cleanly without
    spinning a wall-clock-paced loop.
    """
    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))
    policy = MetisPolicyManager.from_config(metis_config.policy_config)
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
        diagram = _build_metis_diagram(config.metis_config, DrakeLcm())
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


if __name__ == "__main__":
    run_manor_tests()
