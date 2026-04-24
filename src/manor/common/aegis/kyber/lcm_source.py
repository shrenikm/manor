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

Channels:
    AEGIS_PROPRIOCEPTION   -> lcmt_proprioception
    AEGIS_ACTION           -> lcmt_action
"""

from __future__ import annotations

import argparse

import attr
from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmInterfaceSystem, LcmPublisherSystem

from manor.common.definitions.lcmtypes.lcmt_action import lcmt_action
from manor.common.definitions.lcmtypes.lcmt_proprioception import lcmt_proprioception
from manor.common.definitions.utils.defaults import (
    construct_default_action,
    construct_default_proprioception,
    construct_system_time_header,
)

AEGIS_PROPRIOCEPTION_CHANNEL = "AEGIS_PROPRIOCEPTION"
AEGIS_ACTION_CHANNEL = "AEGIS_ACTION"

_DEMO_NUM_JOINTS = 6
_DEFAULT_PUBLISH_FREQUENCY_HZ = 10.0


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
            "proprioception",
            alloc=lambda: AbstractValue.Make(construct_default_proprioception(num_joints=_DEMO_NUM_JOINTS)),
            calc=self._calc_proprioception,
        )
        self.DeclareAbstractOutputPort(
            "action",
            alloc=lambda: AbstractValue.Make(construct_default_action(num_joints=_DEMO_NUM_JOINTS)),
            calc=self._calc_action,
        )

    def _calc_proprioception(self, context: Context, output: AbstractValue) -> None:
        output.set_value(
            attr.evolve(
                construct_default_proprioception(num_joints=_DEMO_NUM_JOINTS),
                header=construct_system_time_header(),
            )
        )

    def _calc_action(self, context: Context, output: AbstractValue) -> None:
        output.set_value(
            attr.evolve(
                construct_default_action(num_joints=_DEMO_NUM_JOINTS),
                header=construct_system_time_header(),
            )
        )


class _AegisToLcmMessageSystem(LeafSystem):
    """
    Translates an aegis attrs message to its LCM counterpart.

    Input:  aegis message of the same concrete type as ``model_value``.
    Output: generated LCM message produced via ``to_lcm_message()``.
    """

    def __init__(self, model_value) -> None:
        super().__init__()
        lcm_model_value = model_value.to_lcm_message()
        self._input = self.DeclareAbstractInputPort("in", AbstractValue.Make(model_value))
        self.DeclareAbstractOutputPort(
            "out",
            alloc=lambda: AbstractValue.Make(lcm_model_value),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._input.Eval(context).to_lcm_message())


def build_lcm_source_diagram(lcm: DrakeLcm, publish_frequency_hz: float) -> Diagram:
    """
    Build the LCM source diagram.

    The caller owns ``lcm``. This function does NOT add an LcmInterfaceSystem --
    publishers don't need the receive thread, and the outer main() wraps
    everything in one so a standalone run still has the queue pumped.
    """

    builder = DiagramBuilder()

    source = builder.AddSystem(_FreshDefaultMessageSource())
    source.set_name("fresh_default_source")

    proprioception_to_lcm = builder.AddSystem(
        _AegisToLcmMessageSystem(construct_default_proprioception(num_joints=_DEMO_NUM_JOINTS))
    )
    proprioception_to_lcm.set_name("proprioception_to_lcm")

    action_to_lcm = builder.AddSystem(_AegisToLcmMessageSystem(construct_default_action(num_joints=_DEMO_NUM_JOINTS)))
    action_to_lcm.set_name("action_to_lcm")

    publish_period_sec = 1.0 / publish_frequency_hz
    proprioception_publisher = builder.AddSystem(
        LcmPublisherSystem.Make(
            channel=AEGIS_PROPRIOCEPTION_CHANNEL,
            lcm_type=lcmt_proprioception,
            lcm=lcm,
            publish_period=publish_period_sec,
        )
    )
    proprioception_publisher.set_name("proprioception_publisher")

    action_publisher = builder.AddSystem(
        LcmPublisherSystem.Make(
            channel=AEGIS_ACTION_CHANNEL,
            lcm_type=lcmt_action,
            lcm=lcm,
            publish_period=publish_period_sec,
        )
    )
    action_publisher.set_name("action_publisher")

    builder.Connect(source.GetOutputPort("proprioception"), proprioception_to_lcm.GetInputPort("in"))
    builder.Connect(source.GetOutputPort("action"), action_to_lcm.GetInputPort("in"))
    builder.Connect(proprioception_to_lcm.GetOutputPort("out"), proprioception_publisher.get_input_port())
    builder.Connect(action_to_lcm.GetOutputPort("out"), action_publisher.get_input_port())

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
        f"'{AEGIS_PROPRIOCEPTION_CHANNEL}' and '{AEGIS_ACTION_CHANNEL}'. "
        "Ctrl+C to stop."
    )
    simulator.AdvanceTo(args.duration)


if __name__ == "__main__":
    main()
