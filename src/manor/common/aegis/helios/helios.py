"""
Helios LeafSystem: periodic sensor publisher.

Holds a SensorBackend (sim or hardware) and produces RGB + depth image
messages at ``publish_frequency``. The backend is a plain Python protocol
-- not a Drake system -- so the Drake graph shape is identical in sim and
on hardware.
"""

from __future__ import annotations

from enum import StrEnum
from typing import Protocol, runtime_checkable

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.utils.defaults import construct_depth_image, construct_rgb_image


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
    Publishes sensor messages at a fixed frequency.
    """

    def __init__(self, backend: SensorBackend, publish_frequency: float) -> None:
        super().__init__()
        if publish_frequency <= 0.0:
            raise ValueError(f"publish_frequency must be positive, got {publish_frequency}")

        self._backend = backend
        self._publish_frequency = publish_frequency

        self._rgb_state_index = self.DeclareAbstractState(AbstractValue.Make(construct_rgb_image()))
        self._depth_state_index = self.DeclareAbstractState(AbstractValue.Make(construct_depth_image()))

        self.DeclareAbstractOutputPort(
            HeliosPorts.OUTPUT_RGB_IMAGE,
            alloc=lambda: AbstractValue.Make(construct_rgb_image()),
            calc=self._calc_rgb_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._rgb_state_index)},
        )
        self.DeclareAbstractOutputPort(
            HeliosPorts.OUTPUT_DEPTH_IMAGE,
            alloc=lambda: AbstractValue.Make(construct_depth_image()),
            calc=self._calc_depth_output,
            prerequisites_of_calc={self.abstract_state_ticket(self._depth_state_index)},
        )

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / publish_frequency,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @property
    def publish_frequency(self) -> float:
        return self._publish_frequency

    @property
    def backend(self) -> SensorBackend:
        return self._backend

    def _calc_rgb_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._rgb_state_index).get_value())

    def _calc_depth_output(self, context: Context, output: AbstractValue) -> None:
        output.set_value(context.get_abstract_state(self._depth_state_index).get_value())

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        state.get_mutable_abstract_state(self._rgb_state_index).set_value(self._backend.read_rgb())
        state.get_mutable_abstract_state(self._depth_state_index).set_value(self._backend.read_depth())
        return EventStatus.Succeeded()
