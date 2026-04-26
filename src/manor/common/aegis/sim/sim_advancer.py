"""
``SimAdvancer``: sim-mode-only LeafSystem that drives a ``Sim`` forward.

The aegis backends call into a shared ``Sim`` instance to read sensor
frames and joint state, but the Sim's clock is otherwise idle -- nothing
in those calls advances physics. ``SimAdvancer`` plugs that gap: at a
fixed period it calls ``sim.advance_to(context.get_time())``, keeping
the Sim's simulated clock in step with the outer aegis diagram clock.

It has no input or output ports; it's pure side-effect. The default
period (~500 Hz) is fast relative to camera + policy rates so sensor
reads land on a freshly stepped Sim state.
"""

from __future__ import annotations

from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.sim.sim import Sim

_DEFAULT_SIM_ADVANCE_FREQUENCY_HZ = 500.0


class SimAdvancer(LeafSystem):
    """
    LeafSystem that advances a ``Sim`` on a fixed-period event.
    """

    def __init__(self, sim: Sim, advance_frequency_hz: float = _DEFAULT_SIM_ADVANCE_FREQUENCY_HZ) -> None:
        super().__init__()
        if advance_frequency_hz <= 0.0:
            raise ValueError(f"advance_frequency_hz must be positive, got {advance_frequency_hz}")

        self._sim = sim
        self._advance_frequency_hz = advance_frequency_hz

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / advance_frequency_hz,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    @property
    def sim(self) -> Sim:
        return self._sim

    @property
    def advance_frequency_hz(self) -> float:
        return self._advance_frequency_hz

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        del state
        self._sim.advance_to(context.get_time())
        return EventStatus.Succeeded()
