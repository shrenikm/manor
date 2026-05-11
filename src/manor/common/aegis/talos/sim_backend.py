"""
Simulation ManipulatorBackend.

Closes over a Gaia instance: forwards Talos's outgoing JointEECommand into the simulation, and reads
the simulation's joint + EE state back. Gaia is *not* advanced from here -- that's the GaiaAdvancer
LeafSystem's job.

Lifecycle ownership splits along two axes, mirroring the hardware backend:

* Kyber lifecycle (start / stop) drives "prime / unprime" by snapping the plant context to PRIME on
  start and to REST on stop, so sim and hardware share the same starting/ending state.
* Metis lifecycle (action stream presence) drives halt / resume. If the action stream goes stale the
  backend halts the simulated arm by clearing Gaia's command latches (the inner controller falls back
  to "hold measured pose with zero desired velocity") and rejects further send_joint_ee_command calls
  until a fresh action header arrives via pet_watchdog, which auto-resumes.
"""

from __future__ import annotations

import time
from typing import Self

import attr
import numpy as np

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.logging_utils import ManorLogger


@attr.frozen
class SimManipulatorBackendConfig:
    """
    Configuration for the simulation manipulator backend.

    minimum_watchdog_frequency_hz sets the lower bound on the rate at which the backend expects
    pet_watchdog calls (proxying for the Metis Action stream). The staleness threshold the backend
    trips on is 1 / minimum_watchdog_frequency_hz seconds. Required (no default) -- every sim run
    must declare it in the YAML so the operator has consciously chosen a value matched to the
    policy's publish rate. Hardware mirrors this field on its own backend config; the two are
    declared on the active backend's block so the operator only fills in the one for the mode in
    use.
    """

    minimum_watchdog_frequency_hz: float = attr.field(validator=attr.validators.gt(0.0))

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "sim_backend_config"))


@attr.define
class SimManipulatorBackend:
    """
    ManipulatorBackend that drives a shared Gaia.
    """

    gaia: Gaia
    config: SimManipulatorBackendConfig
    # Newest action header seen via pet_watchdog, in monotonic ns. Zero means "no action has ever
    # arrived" -- the watchdog stays disarmed during the startup grace window before the first real
    # Metis publish.
    _latest_action_monotonic_ns: int = attr.field(init=False, default=0)
    # Halted flag. Set when the watchdog trips on a stale action stream; cleared automatically when
    # a strictly-newer action header arrives. While halted the gaia command latches have been
    # cleared and send_joint_ee_command is a no-op so commands routed from stale Kyber state
    # aren't pushed to the plant.
    _stopped: bool = attr.field(init=False, default=False)
    _logger: ManorLogger = attr.field(init=False)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        return ManorLogger(self.__class__.__name__)

    def start(self) -> None:
        # Kyber-lifecycle hook: mirror HardwareManipulatorBackend.start. In sim there's no streaming
        # controller to ramp toward the target, so we snap the plant context directly to the
        # manipulator's PRIME plant positions vector; the diagram begins ticking from that pose. Reset
        # watchdog state so a relaunched gylos always starts from a known-clean baseline.
        self._latest_action_monotonic_ns = 0
        self._stopped = False
        self.gaia.set_joint_positions(self.gaia.manipulator_model.get_prime_plant_positions())

    def stop(self) -> None:
        # Symmetric counterpart to start: snap the plant to REST so the sim's final pose matches what
        # unprime leaves the real arm at. The diagram is tearing down anyway, so this is purely state
        # hygiene -- but the symmetry keeps the lifecycle obvious.
        self.gaia.set_joint_positions(self.gaia.manipulator_model.get_rest_plant_positions())

    def _is_action_stale(self) -> bool:
        # Startup grace: if we've never seen a real action header, don't park. The first Metis publish
        # will set _latest_action_monotonic_ns; only after that point can the watchdog trip.
        if self._latest_action_monotonic_ns == 0:
            return False
        threshold_ns = int(1e9 / self.config.minimum_watchdog_frequency_hz)
        return (time.monotonic_ns() - self._latest_action_monotonic_ns) > threshold_ns

    def halt(self) -> None:
        # Stop the simulated arm by clearing Gaia's command latches. Without this, the most recently
        # stashed velocity (or position) would keep driving the inner controller indefinitely;
        # clearing falls _DesiredStateSource back to its "hold measured pose with zero desired
        # velocity" default. Pose / EE state are otherwise preserved.
        self.gaia.clear_command_latches()

    def pet_watchdog(self, header: TimestampHeader) -> None:
        """
        Reset the staleness timer with the latest upstream-action header. Called by
        StaleCommandWatchdog on every periodic tick. Two side effects:

        * Strictly-newer header advances _latest_action_monotonic_ns so _is_action_stale stays False
          until at least one threshold-window passes without a fresh tick.
        * If the watchdog had previously halted the arm and we now see a strictly-newer header,
          auto-resume: clear the halted flag so subsequent send_joint_ee_command calls flow through
          again. This makes a Metis restart "just work" without operator intervention.
        """
        # Only advance the latest stamp on a strictly-newer header. The LCM subscriber holds the last
        # received message, so the watchdog hands us the same header tick after tick when Metis is
        # paused; if we treated each call as "fresh" the timer could never trip.
        if header.monotonic_ns <= self._latest_action_monotonic_ns:
            return
        self._latest_action_monotonic_ns = int(header.monotonic_ns)
        if self._stopped:
            self._logger.info("fresh action stream detected after halt -- resuming sim arm motion")
            self._stopped = False

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None:
        if self._stopped:
            return
        # Startup gate: if no real action has ever made it through, the JointEECommand on the wire is
        # the default-constructed one. Forwarding that to gaia stashes a zero-shape command --
        # harmless in practice, but we drop the send anyway so behaviour matches hardware (which would
        # crash the xarm SDK on the empty payload).
        if self._latest_action_monotonic_ns == 0:
            return
        if self._is_action_stale():
            # Trip the watchdog: log, halt the arm (clear gaia latches -- pose preserved), mark
            # _stopped so subsequent commands are dropped until pet_watchdog sees a fresh header and
            # auto-resumes.
            stale_age_s = (time.monotonic_ns() - self._latest_action_monotonic_ns) * 1e-9
            threshold_s = 1.0 / self.config.minimum_watchdog_frequency_hz
            self._logger.warning(
                f"action stream stale ({stale_age_s:.3f}s since last fresh action; threshold "
                f"{threshold_s:.3f}s = 1 / {self.config.minimum_watchdog_frequency_hz:.3f}Hz) -- "
                f"halting sim arm motion. Restart Metis to resume."
            )
            self._stopped = True
            self.halt()
            return
        joint_command = joint_ee_command.joint_command
        if joint_command.joint_positions is not None:
            self.gaia.apply_joint_position_command(joint_command.joint_positions)
        elif joint_command.joint_velocities is not None:
            self.gaia.apply_joint_velocity_command(joint_command.joint_velocities)
        ee_command = joint_ee_command.ee_command
        if ee_command is not None:
            if ee_command.ee_positions is not None:
                self.gaia.apply_ee_position_command(ee_command.ee_positions)
            elif ee_command.ee_velocities is not None:
                self.gaia.apply_ee_velocity_command(ee_command.ee_velocities)

    def read_joint_state(self) -> JointState:
        return self.gaia.read_joint_state()

    def read_ee_state(self) -> EEState:
        manipulator_model = self.gaia.manipulator_model
        num_arm_dof = manipulator_model.get_num_dof()
        num_ee_dofs = manipulator_model.get_num_ee_dofs()
        header = TimestampHeader.from_system_time()

        joint_state = self.gaia.read_joint_state()
        plant_ee_q = joint_state.joint_positions.positions[num_arm_dof:]
        ee_position_values = manipulator_model.plant_positions_to_ee_positions(plant_ee_q)

        return EEState(
            header=header,
            ee_positions=EEPositions(header=header, positions=ee_position_values),
            ee_velocities=EEVelocities(
                header=header,
                # EE velocity is not modelled separately yet.
                velocities=np.zeros(num_ee_dofs, dtype=np.float64),
            ),
        )
