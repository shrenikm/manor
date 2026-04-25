"""
Adapters bridging aegis definitions with Drake LCM pub/sub.

Each adapter is a Drake ``Diagram`` that wraps an LCM publisher or
subscriber together with a small translator ``LeafSystem``. Callers drop
the adapter into a ``DiagramBuilder`` and connect its single
definition-typed port directly to a sub-system port, instead of
hand-wiring the LCM <-> definition bridge each time.

Construct adapters via the ``from_lcm_type`` classmethod, which works for
any ``DefinitionBase`` subclass: pass a fully-constructed model value of
the desired definition (typically from
``manor.common.definitions.utils.defaults``) and the adapter derives the
LCM message class from it via ``type(model_value).get_lcm_class()``.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Self

from pydrake.common.value import AbstractValue
from pydrake.lcm import DrakeLcm
from pydrake.systems.framework import Context, Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.lcm import LcmPublisherSystem, LcmSubscriberSystem

from manor.common.aegis.aegis_constants import AegisAdapterSystemRole, AegisChannel
from manor.common.definitions.utils.interfaces import DefinitionBase


class _LcmToDefinitionTranslatorPorts(StrEnum):
    INPUT_LCM = "lcm_input"
    OUTPUT_DEFINITION = "definition_output"


class _DefinitionToLcmTranslatorPorts(StrEnum):
    INPUT_DEFINITION = "definition_input"
    OUTPUT_LCM = "lcm_output"


class AegisLCMSubscriberAdapterPorts(StrEnum):
    OUTPUT_DEFINITION = "definition_output"


class AegisLCMPublisherAdapterPorts(StrEnum):
    INPUT_DEFINITION = "definition_input"


class _LcmToDefinitionTranslator(LeafSystem):
    """
    LeafSystem that converts an LCM message to its aegis definition
    counterpart. Aegis definitions ARE the LCM-message types as far as
    consumers are concerned: ``definition_cls.from_lcm_message`` does the
    decode.
    """

    def __init__(self, definition_cls: type[DefinitionBase], definition_model_value: DefinitionBase) -> None:
        super().__init__()
        self._definition_cls = definition_cls
        lcm_cls = definition_cls.get_lcm_class()
        self._input = self.DeclareAbstractInputPort(
            _LcmToDefinitionTranslatorPorts.INPUT_LCM,
            AbstractValue.Make(lcm_cls()),
        )
        self.DeclareAbstractOutputPort(
            _LcmToDefinitionTranslatorPorts.OUTPUT_DEFINITION,
            alloc=lambda: AbstractValue.Make(definition_model_value),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._definition_cls.from_lcm_message(self._input.Eval(context)))


class _DefinitionToLcmTranslator(LeafSystem):
    """
    LeafSystem that converts an aegis definition to its LCM counterpart
    via ``definition.to_lcm_message()``.
    """

    def __init__(self, definition_model_value: DefinitionBase) -> None:
        super().__init__()
        self._input = self.DeclareAbstractInputPort(
            _DefinitionToLcmTranslatorPorts.INPUT_DEFINITION,
            AbstractValue.Make(definition_model_value),
        )
        self.DeclareAbstractOutputPort(
            _DefinitionToLcmTranslatorPorts.OUTPUT_LCM,
            alloc=lambda: AbstractValue.Make(definition_model_value.to_lcm_message()),
            calc=self._calc,
        )

    def _calc(self, context: Context, output: AbstractValue) -> None:
        output.set_value(self._input.Eval(context).to_lcm_message())


class AegisLCMSubscriberAdapter(Diagram):
    """
    Diagram that subscribes to an aegis LCM channel and exposes the
    decoded definition on a single abstract output port.

    Construct via ``AegisLCMSubscriberAdapter.from_lcm_type(...)``. The
    output port is named by ``AegisLCMSubscriberAdapterPorts.OUTPUT_DEFINITION``.
    """

    def __init__(
        self,
        definition_model_value: DefinitionBase,
        channel: AegisChannel,
        lcm: DrakeLcm,
    ) -> None:
        super().__init__()
        definition_cls = type(definition_model_value)
        builder = DiagramBuilder()

        subscriber = builder.AddSystem(
            LcmSubscriberSystem.Make(
                channel=channel,
                lcm_type=definition_cls.get_lcm_class(),
                lcm=lcm,
            )
        )
        subscriber.set_name(AegisAdapterSystemRole.SUBSCRIBER)

        translator = builder.AddSystem(
            _LcmToDefinitionTranslator(
                definition_cls=definition_cls,
                definition_model_value=definition_model_value,
            )
        )
        translator.set_name(AegisAdapterSystemRole.LCM_TO_DEFINITION_TRANSLATOR)

        builder.Connect(
            subscriber.get_output_port(),
            translator.GetInputPort(_LcmToDefinitionTranslatorPorts.INPUT_LCM),
        )
        builder.ExportOutput(
            translator.GetOutputPort(_LcmToDefinitionTranslatorPorts.OUTPUT_DEFINITION),
            AegisLCMSubscriberAdapterPorts.OUTPUT_DEFINITION,
        )

        builder.BuildInto(self)
        # Channel-qualified name keeps multiple adapter instances in the
        # same outer diagram uniquely identifiable, which Drake requires
        # for sibling subsystems.
        self.set_name(f"{type(self).__name__}_{channel.name}")

    @classmethod
    def from_lcm_type(
        cls,
        definition_model_value: DefinitionBase,
        channel: AegisChannel,
        lcm: DrakeLcm,
    ) -> Self:
        """
        Construct a subscriber adapter for ``type(definition_model_value)``.

        ``definition_model_value`` is the template instance Drake uses to
        allocate the abstract output port (Drake requires a model value
        for type identity on abstract ports). Build it from
        ``manor.common.definitions.utils.defaults``, or pass a custom
        instance if a specific buffer shape is required downstream.
        """
        return cls(definition_model_value=definition_model_value, channel=channel, lcm=lcm)


class AegisLCMPublisherAdapter(Diagram):
    """
    Diagram that takes an aegis definition on a single abstract input
    port, encodes it to its LCM counterpart, and publishes it on an aegis
    LCM channel at the given period.

    Construct via ``AegisLCMPublisherAdapter.from_lcm_type(...)``. The
    input port is named by ``AegisLCMPublisherAdapterPorts.INPUT_DEFINITION``.
    """

    def __init__(
        self,
        definition_model_value: DefinitionBase,
        channel: AegisChannel,
        lcm: DrakeLcm,
        publish_period: float,
    ) -> None:
        super().__init__()
        definition_cls = type(definition_model_value)
        builder = DiagramBuilder()

        translator = builder.AddSystem(_DefinitionToLcmTranslator(definition_model_value=definition_model_value))
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
            translator.GetOutputPort(_DefinitionToLcmTranslatorPorts.OUTPUT_LCM),
            publisher.get_input_port(),
        )
        builder.ExportInput(
            translator.GetInputPort(_DefinitionToLcmTranslatorPorts.INPUT_DEFINITION),
            AegisLCMPublisherAdapterPorts.INPUT_DEFINITION,
        )

        builder.BuildInto(self)
        self.set_name(f"{type(self).__name__}_{channel.name}")

    @classmethod
    def from_lcm_type(
        cls,
        definition_model_value: DefinitionBase,
        channel: AegisChannel,
        lcm: DrakeLcm,
        publish_period: float,
    ) -> Self:
        """
        Construct a publisher adapter for ``type(definition_model_value)``.

        ``publish_period`` is forwarded to the underlying
        ``LcmPublisherSystem``. ``definition_model_value`` is the input
        port allocator template; build from
        ``manor.common.definitions.utils.defaults`` for the standard
        zero-valued instance.
        """
        return cls(
            definition_model_value=definition_model_value,
            channel=channel,
            lcm=lcm,
            publish_period=publish_period,
        )
