"""
StaleCommandWatchdog: hardware-only LeafSystem that observes the Action
stream and pokes HardwareManipulatorBackend.pet_watchdog on
every periodic tick.

Architectural rationale:

* Talos's contract is "JointEECommand in, Proprioception out". It must
  not know about Action -- that's a Metis concept, and routing it
  through Talos would cross the kyber/talos layer boundary.
* The staleness detection itself lives on the backend (so the trip
  decision can be made inside HardwareManipulatorBackend.send_joint_ee_command,
  which is where the parked-flag short-circuit and driver.unprime call
  fire). All this LeafSystem does is keep the backend's "latest action
  stamp" up to date.
* The watchdog is opt-in: gylos (sim mode) does not instantiate it,
  because there's no physical robot to park. Only run_kylos wires it
  into the diagram.

The watchdog holds a direct reference to the backend the same way Talos
does -- the backend lives in the kylos process and is shared across
LeafSystems by reference, not by Drake port.
"""

from __future__ import annotations

from enum import StrEnum

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.definitions.action import Action


class StaleCommandWatchdogPorts(StrEnum):
    """
    Named input ports exposed by StaleCommandWatchdog.
    """

    INPUT_ACTION = "action"


class StaleCommandWatchdog(LeafSystem):
    """
    Pumps the latest Action header into a backend's action-staleness
    watchdog at a fixed periodic rate.

    The backend is responsible for the actual trip decision; the
    watchdog only forwards the header. tick_frequency_hz should be at
    least the Metis publish frequency so the backend always observes
    the freshest header before the threshold elapses.
    """

    def __init__(self, backend, tick_frequency_hz: float) -> None:
        super().__init__()
        if tick_frequency_hz <= 0.0:
            raise ValueError(f"tick_frequency_hz must be positive, got {tick_frequency_hz}")

        self.backend = backend
        self.tick_frequency_hz = tick_frequency_hz

        self._action_input = self.DeclareAbstractInputPort(
            StaleCommandWatchdogPorts.INPUT_ACTION,
            AbstractValue.Make(Action.construct_default()),
        )
        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / tick_frequency_hz,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        action: Action = self._action_input.Eval(context)
        self.backend.pet_watchdog(action.header)
        return EventStatus.Succeeded()
