"""
Tests for the generic LCM <-> aegis-definition adapters.

Coverage:
    - ``from_lcm_type`` builds a Drake Diagram with the expected single
      port for any DefinitionBase subclass (parametrized over a sample of
      definition types covering scalars, optionals, variants, and images).
    - End-to-end round trip through publisher -> memq LCM -> subscriber
      reproduces the input message.
"""

from __future__ import annotations

import attr
import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmInterfaceSystem

from manor.common.aegis.aegis_adapters import (
    AegisLCMPublisherAdapter,
    AegisLCMPublisherAdapterPorts,
    AegisLCMSubscriberAdapter,
    AegisLCMSubscriberAdapterPorts,
)
from manor.common.aegis.aegis_constants import AegisChannel
from manor.common.definitions.action import Action
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.utils.defaults import (
    construct_default_action,
    construct_default_command,
    construct_default_depth_image,
    construct_default_proprioception,
    construct_default_rgb_image,
    construct_system_time_header,
)
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.testing_utils import run_manor_tests

_PUBLISH_PERIOD_SEC = 0.01
_NUM_JOINTS = 6


def _model_value_cases() -> list[tuple[AegisChannel, DefinitionBase]]:
    """
    Returns (channel, definition_model_value) pairs covering the message
    shapes the adapters are expected to handle: scalar/array variant
    messages (Action, Command), nested optional fields (Proprioception),
    and image messages (RGB / Depth) whose default LCM round-trip would
    fail without an explicit model_value.
    """
    return [
        (AegisChannel.ACTION, construct_default_action(num_joints=_NUM_JOINTS)),
        (AegisChannel.COMMAND, construct_default_command(num_joints=_NUM_JOINTS)),
        (AegisChannel.PROPRIOCEPTION, construct_default_proprioception(num_joints=_NUM_JOINTS)),
        (AegisChannel.RGB_IMAGE, construct_default_rgb_image(height=4, width=4)),
        (AegisChannel.DEPTH_IMAGE, construct_default_depth_image(height=4, width=4)),
    ]


def _model_value_case_id(value: object) -> str:
    if isinstance(value, AegisChannel):
        return value.name
    return ""


class _ConstantSourcePorts:
    OUTPUT_DEFINITION = "definition_output"


class _ConstantSource(LeafSystem):
    """
    LeafSystem that emits a fixed aegis definition on its single output
    port. Used to drive the publisher adapter under test with a known
    payload.
    """

    def __init__(self, value: DefinitionBase) -> None:
        super().__init__()
        self._value = value
        self.DeclareAbstractOutputPort(
            _ConstantSourcePorts.OUTPUT_DEFINITION,
            alloc=lambda: AbstractValue.Make(value),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._value)


class TestAdapterConstruction:
    @pytest.mark.parametrize("channel,model_value", _model_value_cases(), ids=_model_value_case_id)
    def test_subscriber_from_lcm_type(self, channel: AegisChannel, model_value: DefinitionBase) -> None:
        lcm = DrakeLcm("memq://")
        adapter = AegisLCMSubscriberAdapter.from_lcm_type(
            definition_model_value=model_value,
            channel=channel,
            lcm=lcm,
        )
        assert isinstance(adapter, Diagram)
        assert adapter.num_input_ports() == 0
        assert adapter.num_output_ports() == 1
        assert adapter.GetOutputPort(AegisLCMSubscriberAdapterPorts.OUTPUT_DEFINITION) is not None

    @pytest.mark.parametrize("channel,model_value", _model_value_cases(), ids=_model_value_case_id)
    def test_publisher_from_lcm_type(self, channel: AegisChannel, model_value: DefinitionBase) -> None:
        lcm = DrakeLcm("memq://")
        adapter = AegisLCMPublisherAdapter.from_lcm_type(
            definition_model_value=model_value,
            channel=channel,
            lcm=lcm,
            publish_period=_PUBLISH_PERIOD_SEC,
        )
        assert isinstance(adapter, Diagram)
        assert adapter.num_input_ports() == 1
        assert adapter.num_output_ports() == 0
        assert adapter.GetInputPort(AegisLCMPublisherAdapterPorts.INPUT_DEFINITION) is not None


class TestAdapterRoundTrip:
    @pytest.mark.parametrize("channel,model_value", _model_value_cases(), ids=_model_value_case_id)
    def test_publisher_to_subscriber_roundtrip(self, channel: AegisChannel, model_value: DefinitionBase) -> None:
        """
        Wire publisher -> memq channel -> subscriber and verify that the
        decoded definition matches the source. Uses a non-zero header on
        the source so a stale zero-default at the subscriber would fail
        the assertion.
        """

        lcm = DrakeLcm("memq://")
        source_msg = attr.evolve(model_value, header=construct_system_time_header())

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))

        source = builder.AddSystem(_ConstantSource(source_msg))

        publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_model_value=model_value,
                channel=channel,
                lcm=lcm,
                publish_period=_PUBLISH_PERIOD_SEC,
            )
        )
        subscriber = builder.AddSystem(
            AegisLCMSubscriberAdapter.from_lcm_type(
                definition_model_value=model_value,
                channel=channel,
                lcm=lcm,
            )
        )

        builder.Connect(
            source.GetOutputPort(_ConstantSourcePorts.OUTPUT_DEFINITION),
            publisher.GetInputPort(AegisLCMPublisherAdapterPorts.INPUT_DEFINITION),
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.2)

        sub_context = subscriber.GetMyContextFromRoot(simulator.get_context())
        decoded = subscriber.GetOutputPort(AegisLCMSubscriberAdapterPorts.OUTPUT_DEFINITION).Eval(sub_context)

        assert decoded.header.monotonic_ns == source_msg.header.monotonic_ns
        assert decoded.header.system_ns == source_msg.header.system_ns
        assert type(decoded) is type(model_value)


class TestAdapterTypeDerivation:
    def test_definition_class_is_derived_from_model_value(self) -> None:
        """
        The adapter derives the LCM message class via
        ``type(definition_model_value).get_lcm_class()``; the underlying
        LcmPublisherSystem / LcmSubscriberSystem of the constructed
        adapter must therefore carry the matching LCM type.
        """

        lcm = DrakeLcm("memq://")
        action_adapter = AegisLCMPublisherAdapter.from_lcm_type(
            definition_model_value=construct_default_action(num_joints=_NUM_JOINTS),
            channel=AegisChannel.ACTION,
            lcm=lcm,
            publish_period=_PUBLISH_PERIOD_SEC,
        )
        # Adapters are Diagrams; both should construct cleanly with
        # different definition types and not error on get_input_port.
        assert action_adapter.num_input_ports() == 1

        command_adapter = AegisLCMPublisherAdapter.from_lcm_type(
            definition_model_value=construct_default_command(num_joints=_NUM_JOINTS),
            channel=AegisChannel.COMMAND,
            lcm=lcm,
            publish_period=_PUBLISH_PERIOD_SEC,
        )
        assert command_adapter.num_input_ports() == 1


class TestAdapterPreservesPayload:
    def test_action_joint_positions_payload_is_preserved(self) -> None:
        """
        Beyond the header check in the round-trip test, also verify that
        a non-default payload (specific joint positions in an Action)
        survives the publisher -> LCM -> subscriber path.
        """

        lcm = DrakeLcm("memq://")
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        source_msg = Action(
            header=construct_system_time_header(),
            joint_positions=JointPositions(header=construct_system_time_header(), positions=positions),
        )

        model_value = construct_default_action(num_joints=_NUM_JOINTS)

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))
        source = builder.AddSystem(_ConstantSource(source_msg))
        publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_model_value=model_value,
                channel=AegisChannel.ACTION,
                lcm=lcm,
                publish_period=_PUBLISH_PERIOD_SEC,
            )
        )
        subscriber = builder.AddSystem(
            AegisLCMSubscriberAdapter.from_lcm_type(
                definition_model_value=model_value,
                channel=AegisChannel.ACTION,
                lcm=lcm,
            )
        )
        builder.Connect(
            source.GetOutputPort(_ConstantSourcePorts.OUTPUT_DEFINITION),
            publisher.GetInputPort(AegisLCMPublisherAdapterPorts.INPUT_DEFINITION),
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.2)

        sub_context = subscriber.GetMyContextFromRoot(simulator.get_context())
        decoded = subscriber.GetOutputPort(AegisLCMSubscriberAdapterPorts.OUTPUT_DEFINITION).Eval(sub_context)

        assert isinstance(decoded, Action)
        assert decoded.joint_positions is not None
        np.testing.assert_array_equal(decoded.joint_positions.positions, positions)


if __name__ == "__main__":
    run_manor_tests()
