"""
Metis-process runner.

Builds the slice of the aegis diagram that lives inside the metis
process: an ``Metis`` LeafSystem, three LCM subscriber adapters
(proprioception / rgb / depth), and one LCM publisher adapter
(action). The diagram runs forever; the supervisor stops it with
SIGTERM.

Standalone usage:

    python -m manor.common.aegis.run.run_metis < /tmp/aegis.json

where ``/tmp/aegis.json`` is the YAML re-encoded as JSON (see the
``manor.common.aegis.run`` package docstring for the one-line
``yaml.safe_load(...) -> json.dumps(...)`` recipe).

The JSON payload is the full parsed-AegisConfig dict (the same shape
the YAML produces). The runner only consumes ``metis_config``; the
rest is ignored. Passing the full dict keeps every per-block runner
on the same payload contract.
"""

from __future__ import annotations

import json
import sys

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import (
    AegisAdapterPorts,
    AegisLCMPublisherAdapter,
    AegisLCMSubscriberAdapter,
)
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.aegis.metis.metis import Metis, MetisConfig, MetisPorts
from manor.common.aegis.metis.policies.policy_manager import MetisPolicyManager
from manor.common.aegis.run.run_utils import advance_until_signal
from manor.common.definitions.action import Action
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData


def run_metis(metis_config: MetisConfig, lcm: DrakeLcm | None = None) -> None:
    """
    Build the metis-process diagram, ``Initialize`` its Simulator,
    and advance forever (until SIGTERM / SIGINT).
    """
    lcm = lcm if lcm is not None else DrakeLcm()
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

    diagram = builder.Build()
    diagram.set_name("aegis_metis_process")

    # Realtime pacing keeps Metis's periodic policy step in lockstep
    # with wall-clock so action publish cadence matches what the rest
    # of the system expects.
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    advance_until_signal(simulator)


def _main() -> None:
    from manor.common.aegis.aegis import AegisConfig

    raw = json.loads(sys.stdin.read())
    config = AegisConfig.from_yaml_dict(raw)
    run_metis(metis_config=config.metis_config)


if __name__ == "__main__":
    _main()
