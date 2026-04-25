"""
Kyber wrapped with LCM connectivity.

Builds a Drake diagram that subscribes to Proprioception + Action LCM
messages, feeds them to a Kyber instance, and publishes its Command output
back over LCM at Kyber's publish frequency. Intended as the "downstream"
half of a two-process experiment with ``lcm_source.py``.
"""

from __future__ import annotations

import argparse

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import AegisLCMPublisherAdapter, AegisLCMSubscriberAdapter
from manor.common.aegis.aegis_constants import AegisAdapterPorts, AegisChannel
from manor.common.aegis.kyber.kyber import Kyber, KyberPorts
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.proprioception import Proprioception

_DEFAULT_KYBER_FREQUENCY_HZ = 50.0


def build_kyber_lcm_diagram(lcm: DrakeLcm, kyber_publish_frequency_hz: float) -> Diagram:
    """
    Build the Kyber-over-LCM diagram.

    The caller owns ``lcm``. This function does NOT add an LcmInterfaceSystem;
    the outer run wires one in so subscriber pumping is handled once. When a
    single outer diagram composes this with ``build_lcm_source_diagram``, one
    LcmInterfaceSystem at the top is sufficient for both.
    """

    builder = DiagramBuilder()

    proprioception_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=Proprioception,
            channel=AegisChannel.PROPRIOCEPTION,
            lcm=lcm,
        )
    )

    action_subscriber = builder.AddSystem(
        AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=Action,
            channel=AegisChannel.ACTION,
            lcm=lcm,
        )
    )

    kyber = builder.AddSystem(Kyber(publish_frequency=kyber_publish_frequency_hz))

    command_publisher = builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=Command,
            channel=AegisChannel.COMMAND,
            lcm=lcm,
            publish_period=1.0 / kyber_publish_frequency_hz,
        )
    )

    builder.Connect(
        proprioception_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        action_subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )
    builder.Connect(
        kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND),
        command_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )

    return builder.Build()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run Kyber over LCM: subscribe to Proprioception + Action, publish Command."
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=_DEFAULT_KYBER_FREQUENCY_HZ,
        help="Kyber publish rate in Hz (intentionally distinct from the source rate for frequency verification).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0e6,
        help="Run duration in simulated (real-time) seconds. Defaults to effectively forever.",
    )
    args = parser.parse_args()

    lcm = DrakeLcm()

    outer_builder = DiagramBuilder()
    outer_builder.AddSystem(LcmInterfaceSystem(lcm))
    outer_builder.AddSystem(build_kyber_lcm_diagram(lcm=lcm, kyber_publish_frequency_hz=args.frequency))
    diagram = outer_builder.Build()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    print(
        f"Kyber subscribed on '{AegisChannel.PROPRIOCEPTION}' and '{AegisChannel.ACTION}'; "
        f"publishing on '{AegisChannel.COMMAND}' at {args.frequency} Hz. Ctrl+C to stop."
    )
    simulator.AdvanceTo(args.duration)


if __name__ == "__main__":
    main()
