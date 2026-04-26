"""
Helios LeafSystem: periodic sensor publisher.

Holds a SensorBackend (sim or hardware) and produces RGB + depth image
messages at ``publish_frequency``. The backend is a plain Python
protocol -- not a Drake system -- so the Drake graph shape is identical
in sim and on hardware.

``publish_rgb`` and ``publish_depth`` toggle whether the corresponding
output ports are declared. Both being false yields a "dummy" Helios
with no outputs, useful when downstream subsystems don't need camera
input but the same diagram shape must be preserved.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Protocol, runtime_checkable

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

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


class Helios(LeafSystem):
    """
    Publishes sensor messages at a fixed frequency. RGB / depth output
    ports are conditionally declared based on the publish flags.
    """

    def __init__(
        self,
        backend: SensorBackend,
        publish_frequency: float,
        publish_rgb: bool = True,
        publish_depth: bool = True,
    ) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self.backend = backend
        self.publish_frequency = publish_frequency
        self.publish_rgb = publish_rgb
        self.publish_depth = publish_depth

        self._rgb_state_index = None
        self._depth_state_index = None

        if publish_rgb:
            self._rgb_state_index = self.DeclareAbstractState(AbstractValue.Make(RGBImageData.construct_default()))
            self.DeclareAbstractOutputPort(
                HeliosPorts.OUTPUT_RGB_IMAGE,
                alloc=lambda: AbstractValue.Make(RGBImageData.construct_default()),
                calc=self._calc_rgb_output,
                prerequisites_of_calc={self.abstract_state_ticket(self._rgb_state_index)},
            )

        if publish_depth:
            self._depth_state_index = self.DeclareAbstractState(AbstractValue.Make(DepthImageData.construct_default()))
            self.DeclareAbstractOutputPort(
                HeliosPorts.OUTPUT_DEPTH_IMAGE,
                alloc=lambda: AbstractValue.Make(DepthImageData.construct_default()),
                calc=self._calc_depth_output,
                prerequisites_of_calc={self.abstract_state_ticket(self._depth_state_index)},
            )

        if publish_rgb or publish_depth:
            self.DeclarePeriodicUnrestrictedUpdateEvent(
                period_sec=1.0 / publish_frequency,
                offset_sec=0.0,
                update=self._periodic_update,
            )

    def _calc_rgb_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._rgb_state_index).get_value())

    def _calc_depth_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._depth_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        if self._rgb_state_index is not None:
            state.get_mutable_abstract_state(self._rgb_state_index).set_value(self.backend.read_rgb())
        if self._depth_state_index is not None:
            state.get_mutable_abstract_state(self._depth_state_index).set_value(self.backend.read_depth())
        return EventStatus.Succeeded()
