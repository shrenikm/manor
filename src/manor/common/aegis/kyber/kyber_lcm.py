"""
Kyber wrapped with LCM connectivity.

Builds a Drake diagram that subscribes to Proprioception + Action LCM
messages, feeds them to a Kyber instance, and publishes its Command output
back over LCM at Kyber's publish frequency. Intended as the "downstream"
half of a two-process experiment with ``lcm_source.py``.

Input channels:
    AEGIS_PROPRIOCEPTION -> lcmt_proprioception
    AEGIS_ACTION         -> lcmt_action

Output channel:
    AEGIS_COMMAND        -> lcmt_command
"""

from __future__ import annotations

import argparse

from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmInterfaceSystem, LcmPublisherSystem, LcmSubscriberSystem

from manor.common.aegis.kyber.kyber import Kyber, KyberPorts
from manor.common.aegis.kyber.lcm_source import AEGIS_ACTION_CHANNEL, AEGIS_PROPRIOCEPTION_CHANNEL
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.lcmtypes.lcmt_action import lcmt_action
from manor.common.definitions.lcmtypes.lcmt_command import lcmt_command
from manor.common.definitions.lcmtypes.lcmt_proprioception import lcmt_proprioception
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.utils.defaults import (
    construct_default_action,
    construct_default_command,
    construct_default_proprioception,
)

AEGIS_COMMAND_CHANNEL = "AEGIS_COMMAND"

_DEFAULT_KYBER_FREQUENCY_HZ = 50.0


class _LcmToAegisMessageSystem(LeafSystem):
    """
    Translates an LCM message to its aegis attrs counterpart.

    Input:  LCM message of the same type as ``lcm_model_value``.
    Output: aegis message produced via ``aegis_cls.from_lcm_message(...)``.
    """

    def __init__(self, aegis_cls, lcm_model_value, aegis_model_value) -> None:
        super().__init__()
        self._aegis_cls = aegis_cls
        self._input = self.DeclareAbstractInputPort("in", AbstractValue.Make(lcm_model_value))
        self.DeclareAbstractOutputPort(
            "out",
            alloc=lambda: AbstractValue.Make(aegis_model_value),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._aegis_cls.from_lcm_message(self._input.Eval(context)))


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
        LcmSubscriberSystem.Make(
            channel=AEGIS_PROPRIOCEPTION_CHANNEL,
            lcm_type=lcmt_proprioception,
            lcm=lcm,
        )
    )
    proprioception_subscriber.set_name("proprioception_subscriber")

    action_subscriber = builder.AddSystem(
        LcmSubscriberSystem.Make(
            channel=AEGIS_ACTION_CHANNEL,
            lcm_type=lcmt_action,
            lcm=lcm,
        )
    )
    action_subscriber.set_name("action_subscriber")

    proprioception_from_lcm = builder.AddSystem(
        _LcmToAegisMessageSystem(
            aegis_cls=Proprioception,
            lcm_model_value=lcmt_proprioception(),
            aegis_model_value=construct_default_proprioception(),
        )
    )
    proprioception_from_lcm.set_name("proprioception_from_lcm")

    action_from_lcm = builder.AddSystem(
        _LcmToAegisMessageSystem(
            aegis_cls=Action,
            lcm_model_value=lcmt_action(),
            aegis_model_value=construct_default_action(),
        )
    )
    action_from_lcm.set_name("action_from_lcm")

    kyber = builder.AddSystem(Kyber(publish_frequency=kyber_publish_frequency_hz))
    kyber.set_name("kyber")

    command_to_lcm = builder.AddSystem(_AegisToLcmMessageSystem(construct_default_command()))
    command_to_lcm.set_name("command_to_lcm")

    command_publisher = builder.AddSystem(
        LcmPublisherSystem.Make(
            channel=AEGIS_COMMAND_CHANNEL,
            lcm_type=lcmt_command,
            lcm=lcm,
            publish_period=1.0 / kyber_publish_frequency_hz,
        )
    )
    command_publisher.set_name("command_publisher")

    builder.Connect(proprioception_subscriber.get_output_port(), proprioception_from_lcm.GetInputPort("in"))
    builder.Connect(action_subscriber.get_output_port(), action_from_lcm.GetInputPort("in"))
    builder.Connect(
        proprioception_from_lcm.GetOutputPort("out"),
        kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        action_from_lcm.GetOutputPort("out"),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )
    builder.Connect(kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND), command_to_lcm.GetInputPort("in"))
    builder.Connect(command_to_lcm.GetOutputPort("out"), command_publisher.get_input_port())

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
        f"Kyber subscribed on '{AEGIS_PROPRIOCEPTION_CHANNEL}' and '{AEGIS_ACTION_CHANNEL}'; "
        f"publishing on '{AEGIS_COMMAND_CHANNEL}' at {args.frequency} Hz. Ctrl+C to stop."
    )
    simulator.AdvanceTo(args.duration)


if __name__ == "__main__":
    main()
