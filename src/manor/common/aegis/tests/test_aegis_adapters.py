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
    AegisAdapterPorts,
    AegisLCMPublisherAdapter,
    AegisLCMSubscriberAdapter,
)
from manor.common.aegis.aegis_utils import AegisChannel
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.definitions.utils.interfaces import DefinitionBase
from manor.common.testing_utils import run_manor_tests

_PUBLISH_PERIOD_SEC = 0.01
_NUM_JOINTS = 6


def _definition_cases() -> list[tuple[AegisChannel, type[DefinitionBase]]]:
    """
    Returns (channel, definition_cls) pairs covering the message shapes
    the adapters are expected to handle: scalar/array variant messages
    (Action, Command), nested optional fields (Proprioception), and image
    messages (RGB / Depth) that have strict enum fields.
    """
    return [
        (AegisChannel.ACTION, Action),
        (AegisChannel.COMMAND, Command),
        (AegisChannel.PROPRIOCEPTION, Proprioception),
        (AegisChannel.RGB_IMAGE, RGBImageData),
        (AegisChannel.DEPTH_IMAGE, DepthImageData),
    ]


def _case_id(value: object) -> str:
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


class _FreshHeaderSource(LeafSystem):
    """
    Like ``_ConstantSource`` but stamps a new TimestampHeader on every
    output evaluation. Cadence tests use the unique header on each
    publish to count distinct deliveries on the probe side.
    """

    def __init__(self, template: DefinitionBase) -> None:
        super().__init__()
        self._template = template
        self.DeclareAbstractOutputPort(
            _ConstantSourcePorts.OUTPUT_DEFINITION,
            alloc=lambda: AbstractValue.Make(template),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(attr.evolve(self._template, header=TimestampHeader.from_system_time()))


class TestAdapterConstruction:
    @pytest.mark.parametrize("channel,definition_cls", _definition_cases(), ids=_case_id)
    def test_subscriber_from_lcm_type(self, channel: AegisChannel, definition_cls: type[DefinitionBase]) -> None:
        lcm = DrakeLcm("memq://")
        adapter = AegisLCMSubscriberAdapter.from_lcm_type(
            definition_cls=definition_cls,
            channel=channel,
            lcm=lcm,
        )
        assert isinstance(adapter, Diagram)
        assert adapter.num_input_ports() == 0
        assert adapter.num_output_ports() == 1
        assert adapter.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT) is not None

    @pytest.mark.parametrize("channel,definition_cls", _definition_cases(), ids=_case_id)
    def test_publisher_from_lcm_type(self, channel: AegisChannel, definition_cls: type[DefinitionBase]) -> None:
        lcm = DrakeLcm("memq://")
        adapter = AegisLCMPublisherAdapter.from_lcm_type(
            definition_cls=definition_cls,
            channel=channel,
            lcm=lcm,
            publish_period=_PUBLISH_PERIOD_SEC,
        )
        assert isinstance(adapter, Diagram)
        assert adapter.num_input_ports() == 1
        assert adapter.num_output_ports() == 0
        assert adapter.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT) is not None


class TestAdapterRoundTrip:
    @pytest.mark.parametrize("channel,definition_cls", _definition_cases(), ids=_case_id)
    def test_publisher_to_subscriber_roundtrip(
        self, channel: AegisChannel, definition_cls: type[DefinitionBase]
    ) -> None:
        """
        Wire publisher -> memq channel -> subscriber and verify that the
        decoded definition matches the source. Uses a non-zero header on
        the source so a stale zero-default at the subscriber would fail
        the assertion.
        """

        lcm = DrakeLcm("memq://")
        # Build a default instance and stamp it with system time so the
        # subscriber's decoded header will differ from a stale zero
        # default if no real publish reached it.
        default_msg = (
            definition_cls.construct_default(num_joints=_NUM_JOINTS)
            if definition_cls in (Action, Command, Proprioception)
            else definition_cls.construct_default()
        )
        source_msg = attr.evolve(default_msg, header=TimestampHeader.from_system_time())

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))

        source = builder.AddSystem(_ConstantSource(source_msg))

        publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=definition_cls,
                channel=channel,
                lcm=lcm,
                publish_period=_PUBLISH_PERIOD_SEC,
            )
        )
        subscriber = builder.AddSystem(
            AegisLCMSubscriberAdapter.from_lcm_type(
                definition_cls=definition_cls,
                channel=channel,
                lcm=lcm,
            )
        )

        builder.Connect(
            source.GetOutputPort(_ConstantSourcePorts.OUTPUT_DEFINITION),
            publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.2)

        sub_context = subscriber.GetMyContextFromRoot(simulator.get_context())
        decoded = subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT).Eval(sub_context)

        assert decoded.header.monotonic_ns == source_msg.header.monotonic_ns
        assert decoded.header.system_ns == source_msg.header.system_ns
        assert type(decoded) is definition_cls


class TestAdapterPreservesPayload:
    def test_action_joint_positions_payload_is_preserved(self) -> None:
        """
        Beyond the header check in the round-trip test, also verify that
        a non-default payload (specific joint positions in an Action)
        survives the publisher -> LCM -> subscriber path even when the
        adapter's internal model_value has zero shape.
        """

        lcm = DrakeLcm("memq://")
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        source_msg = Action(
            header=TimestampHeader.from_system_time(),
            joint_positions=JointPositions(header=TimestampHeader.from_system_time(), positions=positions),
        )

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))
        source = builder.AddSystem(_ConstantSource(source_msg))
        publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=Action,
                channel=AegisChannel.ACTION,
                lcm=lcm,
                publish_period=_PUBLISH_PERIOD_SEC,
            )
        )
        subscriber = builder.AddSystem(
            AegisLCMSubscriberAdapter.from_lcm_type(
                definition_cls=Action,
                channel=AegisChannel.ACTION,
                lcm=lcm,
            )
        )
        builder.Connect(
            source.GetOutputPort(_ConstantSourcePorts.OUTPUT_DEFINITION),
            publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.2)

        sub_context = subscriber.GetMyContextFromRoot(simulator.get_context())
        decoded = subscriber.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT).Eval(sub_context)

        assert isinstance(decoded, Action)
        assert decoded.joint_positions is not None
        np.testing.assert_array_equal(decoded.joint_positions.positions, positions)


class TestAdapterPublishCadence:
    def test_publish_period_matches_configured_rate(self) -> None:
        """
        Frequency sanity check. Wire a publisher adapter at a known rate
        through a memq channel into a probe ``LcmSubscriberSystem`` and
        count distinct messages observed across a fixed-duration sweep.
        The source restamps the header on every evaluation so each
        publish carries a unique timestamp; the probe samples and we
        de-duplicate by ``monotonic_ns``. Expected count is roughly
        ``frequency_hz * duration_sec`` -- a loose floor + ceiling
        guards against missed publishes (publisher not ticking) and
        runaway over-publishing (period leakage).
        """
        from pydrake.systems.lcm import LcmSubscriberSystem

        publish_frequency_hz = 50.0
        publish_period_sec = 1.0 / publish_frequency_hz
        total_duration_sec = 1.0
        sample_period_sec = 1.0 / (2.0 * publish_frequency_hz)

        lcm = DrakeLcm("memq://")
        template = Action.construct_default(num_joints=_NUM_JOINTS)

        builder = DiagramBuilder()
        builder.AddSystem(LcmInterfaceSystem(lcm))
        source = builder.AddSystem(_FreshHeaderSource(template))
        publisher = builder.AddSystem(
            AegisLCMPublisherAdapter.from_lcm_type(
                definition_cls=Action,
                channel=AegisChannel.ACTION,
                lcm=lcm,
                publish_period=publish_period_sec,
            )
        )
        probe = builder.AddSystem(
            LcmSubscriberSystem.Make(
                channel=AegisChannel.ACTION,
                lcm_type=Action.get_lcm_class(),
                lcm=lcm,
            )
        )
        builder.Connect(
            source.GetOutputPort(_ConstantSourcePorts.OUTPUT_DEFINITION),
            publisher.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
        )
        diagram = builder.Build()

        simulator = Simulator(diagram)
        seen_stamps: set[int] = set()
        t = 0.0
        while t < total_duration_sec:
            t = min(t + sample_period_sec, total_duration_sec)
            simulator.AdvanceTo(t)
            decoded = probe.get_output_port().Eval(probe.GetMyContextFromRoot(simulator.get_context()))
            if decoded.header.monotonic_ns > 0:
                seen_stamps.add(int(decoded.header.monotonic_ns))

        expected = publish_frequency_hz * total_duration_sec
        assert len(seen_stamps) >= expected / 3, (
            f"Expected ~{expected} adapter publishes, saw {len(seen_stamps)} -- is the publisher ticking?"
        )
        assert len(seen_stamps) <= expected * 3, (
            f"Saw {len(seen_stamps)} publishes for expected ~{expected} -- publisher over-publishing?"
        )


if __name__ == "__main__":
    run_manor_tests()
