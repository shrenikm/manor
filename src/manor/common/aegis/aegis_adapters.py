"""
Adapters bridging aegis definitions with Drake LCM pub/sub.

Each adapter is a Drake ``Diagram`` that wraps an LCM publisher or
subscriber together with a small translator ``LeafSystem``. Callers drop
the adapter into a ``DiagramBuilder`` and connect its single
definition-typed port directly to a sub-system port, instead of
hand-wiring the LCM <-> definition bridge each time.

Construct adapters via the ``from_lcm_type`` classmethod, which works for
any ``DefinitionBase`` subclass: pass the definition class itself and the
adapter discovers both the LCM message class via
``definition_cls.get_lcm_class()`` and the abstract-port allocator
template via ``definition_cls.construct_default()``.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmPublisherSystem, LcmSubscriberSystem

from manor.common.aegis.aegis_utils import AegisAdapterSystemRole, AegisChannel
from manor.common.definitions.utils.interfaces import DefinitionBase


class AegisAdapterPorts(StrEnum):
    """
    Shared port-name enum for all aegis adapters.

    Subscriber adapters expose ``DEFINITION_OUTPUT``; publisher adapters
    expose ``DEFINITION_INPUT``. The translator LeafSystems inside the
    LCM adapter pair use the LCM_* members for their LCM-typed sides.
    Future non-LCM adapters can add members here rather than introducing
    parallel enums.
    """

    LCM_INPUT = "lcm_input"
    LCM_OUTPUT = "lcm_output"
    DEFINITION_INPUT = "definition_input"
    DEFINITION_OUTPUT = "definition_output"


class _LcmToDefinitionTranslator(LeafSystem):
    """
    LeafSystem that converts an LCM message to its aegis definition
    counterpart via ``definition_cls.from_lcm_message``.
    """

    def __init__(self, definition_cls: type[DefinitionBase]) -> None:
        super().__init__()
        self._definition_cls = definition_cls
        self._input = self.DeclareAbstractInputPort(
            AegisAdapterPorts.LCM_INPUT,
            AbstractValue.Make(definition_cls.get_lcm_class()()),
        )
        self.DeclareAbstractOutputPort(
            AegisAdapterPorts.DEFINITION_OUTPUT,
            alloc=lambda: AbstractValue.Make(definition_cls.construct_default()),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._definition_cls.from_lcm_message(self._input.Eval(context)))


class _DefinitionToLcmTranslator(LeafSystem):
    """
    LeafSystem that converts an aegis definition to its LCM counterpart
    via ``definition.to_lcm_message()``.
    """

    def __init__(self, definition_cls: type[DefinitionBase]) -> None:
        super().__init__()
        default_definition = definition_cls.construct_default()
        self._input = self.DeclareAbstractInputPort(
            AegisAdapterPorts.DEFINITION_INPUT,
            AbstractValue.Make(default_definition),
        )
        self.DeclareAbstractOutputPort(
            AegisAdapterPorts.LCM_OUTPUT,
            alloc=lambda: AbstractValue.Make(default_definition.to_lcm_message()),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._input.Eval(context).to_lcm_message())


class AegisLCMSubscriberAdapter(Diagram):
    """
    Diagram that subscribes to an aegis LCM channel and exposes the
    decoded definition on a single abstract output port.

    Construct via ``AegisLCMSubscriberAdapter.from_lcm_type(...)``. The
    output port is named by ``AegisAdapterPorts.DEFINITION_OUTPUT``.
    """

    def __init__(
        self,
        definition_cls: type[DefinitionBase],
        channel: AegisChannel,
        lcm: DrakeLcm,
    ) -> None:
        super().__init__()
        builder = DiagramBuilder()

        subscriber = builder.AddSystem(
            LcmSubscriberSystem.Make(
                channel=channel,
                lcm_type=definition_cls.get_lcm_class(),
                lcm=lcm,
            )
        )
        subscriber.set_name(AegisAdapterSystemRole.SUBSCRIBER)

        translator = builder.AddSystem(_LcmToDefinitionTranslator(definition_cls=definition_cls))
        translator.set_name(AegisAdapterSystemRole.LCM_TO_DEFINITION_TRANSLATOR)

        builder.Connect(
            subscriber.get_output_port(),
            translator.GetInputPort(AegisAdapterPorts.LCM_INPUT),
        )
        builder.ExportOutput(
            translator.GetOutputPort(AegisAdapterPorts.DEFINITION_OUTPUT),
            AegisAdapterPorts.DEFINITION_OUTPUT,
        )

        builder.BuildInto(self)
        # Channel-qualified name keeps multiple adapter instances in the
        # same outer diagram uniquely identifiable, which Drake requires
        # for sibling subsystems.
        self.set_name(f"{type(self).__name__}_{channel.name}")

    @classmethod
    def from_lcm_type(
        cls,
        definition_cls: type[DefinitionBase],
        channel: AegisChannel,
        lcm: DrakeLcm,
    ) -> Self:
        """
        Construct a subscriber adapter for ``definition_cls``.

        The output port's allocator template is derived internally via
        ``definition_cls.construct_default()``; the model_value's shape
        does not constrain what messages can flow through (Drake uses it
        only for type identity).
        """
        return cls(definition_cls=definition_cls, channel=channel, lcm=lcm)


class AegisLCMPublisherAdapter(Diagram):
    """
    Diagram that takes an aegis definition on a single abstract input
    port, encodes it to its LCM counterpart, and publishes it on an aegis
    LCM channel at the given period.

    Construct via ``AegisLCMPublisherAdapter.from_lcm_type(...)``. The
    input port is named by ``AegisAdapterPorts.DEFINITION_INPUT``.
    """

    def __init__(
        self,
        definition_cls: type[DefinitionBase],
        channel: AegisChannel,
        lcm: DrakeLcm,
        publish_period: float,
    ) -> None:
        super().__init__()
        builder = DiagramBuilder()

        translator = builder.AddSystem(_DefinitionToLcmTranslator(definition_cls=definition_cls))
        translator.set_name(AegisAdapterSystemRole.DEFINITION_TO_LCM_TRANSLATOR)

        publisher = builder.AddSystem(
            LcmPublisherSystem.Make(
                channel=channel,
                lcm_type=definition_cls.get_lcm_class(),
                lcm=lcm,
                publish_period=publish_period,
            )
        )
        publisher.set_name(AegisAdapterSystemRole.PUBLISHER)

        builder.Connect(
            translator.GetOutputPort(AegisAdapterPorts.LCM_OUTPUT),
            publisher.get_input_port(),
        )
        builder.ExportInput(
            translator.GetInputPort(AegisAdapterPorts.DEFINITION_INPUT),
            AegisAdapterPorts.DEFINITION_INPUT,
        )

        builder.BuildInto(self)
        self.set_name(f"{type(self).__name__}_{channel.name}")

    @classmethod
    def from_lcm_type(
        cls,
        definition_cls: type[DefinitionBase],
        channel: AegisChannel,
        lcm: DrakeLcm,
        publish_period: float,
    ) -> Self:
        """
        Construct a publisher adapter for ``definition_cls``.

        ``publish_period`` is forwarded to the underlying
        ``LcmPublisherSystem``. The input port's allocator template is
        derived internally via ``definition_cls.construct_default()``.
        """
        return cls(definition_cls=definition_cls, channel=channel, lcm=lcm, publish_period=publish_period)
