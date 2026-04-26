"""
Helios LeafSystem: periodic sensor publisher.

Holds a SensorBackend (sim or hardware) and produces RGB + depth image
messages. Each stream has its own publish frequency: setting either
frequency to ``0.0`` disables that stream (no output port, no periodic
event). Both at zero yields a dummy Helios with no outputs at all,
useful when downstream subsystems don't need camera input but the
same diagram shape must be preserved.

The backend is a plain Python protocol -- not a Drake system -- so the
Drake graph shape is identical in sim and on hardware.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Protocol, Self, runtime_checkable

import attr
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.helios.hardware_backend import HardwareSensorBackendConfig
from manor.common.aegis.helios.sim_backend import SimSensorBackendConfig
from manor.common.aegis.yaml_utils import assert_keys_match_attrs, require_dict, require_number
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData


class HeliosPorts(StrEnum):
    """
    Named output ports exposed by Helios. Helios has no input ports.
    """

    OUTPUT_RGB_IMAGE = "rgb_image"
    OUTPUT_DEPTH_IMAGE = "depth_image"


@runtime_checkable
class SensorBackend(Protocol):
    """
    Protocol for a source of raw sensor frames.

    Implementations return freshly stamped messages; Helios is responsible
    only for the publish cadence and for exposing them as output ports.
    """

    def read_rgb(self) -> RGBImageData: ...

    def read_depth(self) -> DepthImageData: ...


@attr.frozen
class HeliosConfig:
    """
    Helios sub-system configuration.

    ``publish_rgb_frequency_hz`` and ``publish_depth_frequency_hz``
    control the per-stream publish cadence; setting either to ``0.0``
    disables that stream. Both at zero yields a dummy Helios with no
    output ports.

    ``sim_backend_config`` and ``hardware_backend_config`` parametrise
    the per-mode backends; only the matching one is used in any given
    aegis build.

    ``SYSTEM_NAME`` is the name applied to the Helios LeafSystem in the
    diagram; pinning it as a class attribute keeps the name and the rest
    of the sub-system's parametrisation in one place without making it
    settable per instance.
    """

    SYSTEM_NAME: ClassVar[str] = "helios"

    publish_rgb_frequency_hz: float = 30.0
    publish_depth_frequency_hz: float = 30.0
    sim_backend_config: SimSensorBackendConfig = attr.field(factory=SimSensorBackendConfig)
    hardware_backend_config: HardwareSensorBackendConfig = attr.field(factory=HardwareSensorBackendConfig)

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the ``helios_config:`` block of an aegis YAML.
        """
        assert_keys_match_attrs(cls, d, "helios_config")
        sim_backend_config = (
            SimSensorBackendConfig.from_yaml_dict(
                require_dict(d["sim_backend_config"], "helios_config.sim_backend_config")
            )
            if "sim_backend_config" in d
            else SimSensorBackendConfig()
        )
        hardware_backend_config = (
            HardwareSensorBackendConfig.from_yaml_dict(
                require_dict(d["hardware_backend_config"], "helios_config.hardware_backend_config")
            )
            if "hardware_backend_config" in d
            else HardwareSensorBackendConfig()
        )
        return cls(
            publish_rgb_frequency_hz=require_number(
                d.get("publish_rgb_frequency_hz", 30.0),
                "helios_config.publish_rgb_frequency_hz",
            ),
            publish_depth_frequency_hz=require_number(
                d.get("publish_depth_frequency_hz", 30.0),
                "helios_config.publish_depth_frequency_hz",
            ),
            sim_backend_config=sim_backend_config,
            hardware_backend_config=hardware_backend_config,
        )


class Helios(LeafSystem):
    """
    Publishes sensor messages, one independent periodic event per
    enabled stream. ``publish_rgb_frequency_hz`` / ``publish_depth_frequency_hz``
    of ``0.0`` mean "skip this stream entirely" (no output port).
    """

    def __init__(
        self,
        backend: SensorBackend,
        publish_rgb_frequency_hz: float = 30.0,
        publish_depth_frequency_hz: float = 30.0,
    ) -> None:
        super().__init__()
        if publish_rgb_frequency_hz < 0.0:
            raise ValueError(f"publish_rgb_frequency_hz must be non-negative, got {publish_rgb_frequency_hz}")
        if publish_depth_frequency_hz < 0.0:
            raise ValueError(f"publish_depth_frequency_hz must be non-negative, got {publish_depth_frequency_hz}")

        self.backend = backend
        self.publish_rgb_frequency_hz = publish_rgb_frequency_hz
        self.publish_depth_frequency_hz = publish_depth_frequency_hz

        self._rgb_state_index = None
        self._depth_state_index = None

        if publish_rgb_frequency_hz > 0.0:
            self._rgb_state_index = self.DeclareAbstractState(AbstractValue.Make(RGBImageData.construct_default()))
            self.DeclareAbstractOutputPort(
                HeliosPorts.OUTPUT_RGB_IMAGE,
                alloc=lambda: AbstractValue.Make(RGBImageData.construct_default()),
                calc=self._calc_rgb_output,
                prerequisites_of_calc={self.abstract_state_ticket(self._rgb_state_index)},
            )
            self.DeclarePeriodicUnrestrictedUpdateEvent(
                period_sec=1.0 / publish_rgb_frequency_hz,
                offset_sec=0.0,
                update=self._periodic_rgb_update,
            )

        if publish_depth_frequency_hz > 0.0:
            self._depth_state_index = self.DeclareAbstractState(AbstractValue.Make(DepthImageData.construct_default()))
            self.DeclareAbstractOutputPort(
                HeliosPorts.OUTPUT_DEPTH_IMAGE,
                alloc=lambda: AbstractValue.Make(DepthImageData.construct_default()),
                calc=self._calc_depth_output,
                prerequisites_of_calc={self.abstract_state_ticket(self._depth_state_index)},
            )
            self.DeclarePeriodicUnrestrictedUpdateEvent(
                period_sec=1.0 / publish_depth_frequency_hz,
                offset_sec=0.0,
                update=self._periodic_depth_update,
            )

    def _calc_rgb_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._rgb_state_index).get_value())

    def _calc_depth_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._depth_state_index).get_value())

    def _periodic_rgb_update(self, context: Context, state: State) -> EventStatus:
        del context
        state.get_mutable_abstract_state(self._rgb_state_index).set_value(self.backend.read_rgb())
        return EventStatus.Succeeded()

    def _periodic_depth_update(self, context: Context, state: State) -> EventStatus:
        del context
        state.get_mutable_abstract_state(self._depth_state_index).set_value(self.backend.read_depth())
        return EventStatus.Succeeded()
