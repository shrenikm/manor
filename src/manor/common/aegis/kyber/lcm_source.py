"""
Standalone LCM source for Kyber's upstream messages.

Builds a Drake diagram that publishes Proprioception and Action messages over
LCM at a configurable rate. Intended as the "upstream" half of a two-process
experiment: run this in one terminal and ``kyber_lcm.py`` in another, then
watch all three channels (proprioception / action in / command out) with
lcm-spy or similar.

The message bodies are zero-valued default instances whose TimestampHeaders
are refreshed on every evaluation, so consumers can confirm that messages
are actually being re-emitted rather than cached.
"""

from __future__ import annotations

import argparse
from enum import StrEnum

import attr
from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import AegisAdapterPorts, AegisLCMPublisherAdapter
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.definitions.action import Action
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader

_DEMO_NUM_JOINTS = 6
_DEFAULT_PUBLISH_FREQUENCY_HZ = 10.0


class _FreshDefaultMessageSourcePorts(StrEnum):
    OUTPUT_PROPRIOCEPTION = "proprioception"
    OUTPUT_ACTION = "action"


class _FreshDefaultMessageSource(LeafSystem):
    """
    Emits default Proprioception + Action whose TimestampHeaders are
    regenerated at every output evaluation. Stateless by design: the
    timestamp read lives inside the calc callback so every publish tick
    produces a genuinely fresh message.
    """

    def __init__(self) -> None:
        super().__init__()
        self.DeclareAbstractOutputPort(
            _FreshDefaultMessageSourcePorts.OUTPUT_PROPRIOCEPTION,
            alloc=lambda: AbstractValue.Make(Proprioception.construct_default(num_joints=_DEMO_NUM_JOINTS)),
            calc=self._calc_proprioception,
        )
        self.DeclareAbstractOutputPort(
            _FreshDefaultMessageSourcePorts.OUTPUT_ACTION,
            alloc=lambda: AbstractValue.Make(Action.construct_default(num_joints=_DEMO_NUM_JOINTS)),
            calc=self._calc_action,
        )

    def _calc_proprioception(self, context: Context, output: AbstractValue) -> None:
        output.set_value(
            attr.evolve(
                Proprioception.construct_default(num_joints=_DEMO_NUM_JOINTS),
                header=TimestampHeader.from_system_time(),
            )
        )

    def _calc_action(self, context: Context, output: AbstractValue) -> None:
        output.set_value(
            attr.evolve(
                Action.construct_default(num_joints=_DEMO_NUM_JOINTS),
                header=TimestampHeader.from_system_time(),
            )
        )


def build_lcm_source_diagram(lcm: DrakeLcm, publish_frequency_hz: float) -> Diagram:
    """
    Build the LCM source diagram.

    The caller owns ``lcm``. This function does NOT add an LcmInterfaceSystem --
    publishers don't need the receive thread, and the outer main() wraps
    everything in one so a standalone run still has the queue pumped.
    """

    builder = DiagramBuilder()

    source = builder.AddSystem(_FreshDefaultMessageSource())

    publish_period_sec = 1.0 / publish_frequency_hz
    proprioception_publisher = builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=Proprioception,
            channel=AegisChannel.PROPRIOCEPTION,
            lcm=lcm,
            publish_period=publish_period_sec,
        )
    )

    action_publisher = builder.AddSystem(
        AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=Action,
            channel=AegisChannel.ACTION,
            lcm=lcm,
            publish_period=publish_period_sec,
        )
    )

    builder.Connect(
        source.GetOutputPort(_FreshDefaultMessageSourcePorts.OUTPUT_PROPRIOCEPTION),
        proprioception_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )
    builder.Connect(
        source.GetOutputPort(_FreshDefaultMessageSourcePorts.OUTPUT_ACTION),
        action_publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
    )

    return builder.Build()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish fresh Proprioception + Action LCM messages at a fixed frequency."
    )
    parser.add_argument("--frequency", type=float, default=_DEFAULT_PUBLISH_FREQUENCY_HZ, help="Publish rate in Hz.")
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0e6,
        help="Run duration in simulated (real-time) seconds. Defaults to effectively forever.",
    )
    args = parser.parse_args()

    lcm = DrakeLcm()

    # Outer diagram owns the LcmInterfaceSystem so a single process has one
    # receive pump regardless of how the inner diagram is laid out. For a
    # publisher-only source this is belt-and-suspenders -- but it matches the
    # kyber_lcm.py layout, which needs the pump for its subscribers.
    outer_builder = DiagramBuilder()
    outer_builder.AddSystem(LcmInterfaceSystem(lcm))
    outer_builder.AddSystem(build_lcm_source_diagram(lcm=lcm, publish_frequency_hz=args.frequency))
    diagram = outer_builder.Build()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    print(
        f"Publishing at {args.frequency} Hz on "
        f"'{AegisChannel.PROPRIOCEPTION}' and '{AegisChannel.ACTION}'. "
        "Ctrl+C to stop."
    )
    simulator.AdvanceTo(args.duration)


if __name__ == "__main__":
    main()
