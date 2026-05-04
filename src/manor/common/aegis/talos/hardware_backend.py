"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an IManipulatorDriver. The driver is
the source of truth for joint DOF / EE DOF counts; this backend just
routes ManipulatorBackend calls (send_joint_ee_command / read_joint_state /
read_ee_state / start / stop) to the corresponding driver methods.

The backend also runs a stale-command watchdog: every action header
seen via pet_watchdog is stamped against time.monotonic_ns;
if no fresh action arrives within stale_command_threshold_s the
backend calls driver.unprime() to drop the arm to ZERO + STOP and
flips into a "parked" state where subsequent
send_joint_ee_command calls are dropped. The trip is sticky -- a fresh
action does NOT auto-rearm the backend, because the user spec calls
out that we must not let a rogue policy reattach after an outage. Only
a fresh start() (i.e. an aegis restart) re-primes the driver and
re-arms the watchdog.
"""

from __future__ import annotations

import time
from typing import Self

import attr
import numpy as np

from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.ee_positions import EEPositions
from manor.common.definitions.ee_state import EEState
from manor.common.definitions.ee_velocities import EEVelocities
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.logging_utils import ManorLogger
from manor.manipulators.manipulator_driver import IManipulatorDriver

# Default staleness threshold for the action watchdog. Picked at ~3 ticks of a 10 Hz Metis publisher; if
# Metis publishes faster the threshold is comfortably loose, and if Metis publishes slower the operator
# can override via stale_command_threshold_s in talos_config / hardware_backend_config.
_DEFAULT_STALE_COMMAND_THRESHOLD_S = 0.3


@attr.frozen
class HardwareManipulatorBackendConfig:
    """
    Hardware-specific knobs for the manipulator backend. joint DOF / EE
    DOF counts intentionally live on the driver, not here.

    stale_command_threshold_s controls the action-staleness watchdog:
    if no fresh Action arrives via pet_watchdog within this
    window, the backend parks the arm (driver.unprime) and drops
    further commands until the next backend.start().
    """

    stale_command_threshold_s: float = _DEFAULT_STALE_COMMAND_THRESHOLD_S

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "hardware_backend_config"))


@attr.define
class HardwareManipulatorBackend:
    """
    ManipulatorBackend that delegates to an IManipulatorDriver.
    """

    driver: IManipulatorDriver
    config: HardwareManipulatorBackendConfig = attr.field(factory=HardwareManipulatorBackendConfig)
    # Newest action header seen via pet_watchdog, in monotonic ns. Zero means "no action has
    # ever arrived" -- the watchdog stays disarmed during the startup grace window before the first
    # real Metis publish.
    _latest_action_monotonic_ns: int = attr.field(init=False, default=0)
    # Sticky parked flag. Set when the watchdog trips; cleared only by start(). While parked, the
    # driver has been unprimed and send_joint_ee_command is a no-op so a rogue policy resuming after
    # an outage cannot reattach without an explicit aegis restart.
    _parked: bool = attr.field(init=False, default=False)
    _logger: ManorLogger = attr.field(init=False)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        return ManorLogger(self.__class__.__name__)

    def start(self) -> None:
        # start() always re-primes and re-arms the watchdog so a manual REPL relaunch (after a
        # watchdog trip + Metis restart) restores normal operation.
        self._latest_action_monotonic_ns = 0
        self._parked = False
        self.driver.prime()

    def stop(self) -> None:
        # Aegis-wide shutdown path. Unprime is idempotent enough that calling it again after a
        # watchdog-driven park is safe: the helpers' switch_mode and move-to-ZERO don't fault when
        # the arm is already at ZERO.
        self.driver.unprime()

    def pet_watchdog(self, header: TimestampHeader) -> None:
        """
        Reset the staleness timer with the latest upstream-action header. Called by
        StaleCommandWatchdog on every periodic tick; the watchdog only "bites" (parks the arm) once
        the gap between time.monotonic_ns and the latest petted header crosses the configured
        threshold.
        """
        # Only advance the latest stamp on a strictly-newer header. The LCM subscriber holds the last
        # received message, so the watchdog hands us the same header tick after tick when Metis is
        # paused; if we treated each call as "fresh" the timer could never trip.
        if header.monotonic_ns > self._latest_action_monotonic_ns:
            self._latest_action_monotonic_ns = int(header.monotonic_ns)

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None:
        if self._parked:
            return
        if self._is_action_stale():
            # Trip the watchdog: log, unprime the arm (move to ZERO + STOP, no disconnect), park.
            stale_age_s = (time.monotonic_ns() - self._latest_action_monotonic_ns) * 1e-9
            self._logger.warning(
                f"action stream stale ({stale_age_s:.3f}s since last fresh action; threshold "
                f"{self.config.stale_command_threshold_s:.3f}s) -- parking the arm. Restart aegis to "
                f"re-arm."
            )
            self._parked = True
            self.driver.unprime()
            return
        joint_command = joint_ee_command.joint_command
        if joint_command.joint_positions is not None:
            self.driver.write_joint_positions(joint_command.joint_positions)
        elif joint_command.joint_velocities is not None:
            self.driver.write_joint_velocities(joint_command.joint_velocities)
        ee_command = joint_ee_command.ee_command
        if ee_command is not None:
            if ee_command.ee_positions is not None:
                self.driver.write_ee_positions(ee_command.ee_positions)
            elif ee_command.ee_velocities is not None:
                self.driver.write_ee_velocities(ee_command.ee_velocities)

    def read_joint_state(self) -> JointState:
        positions = self.driver.read_joint_positions()
        velocities = self.driver.read_joint_velocities()
        return JointState(
            header=TimestampHeader.from_system_time(),
            joint_positions=positions,
            joint_velocities=velocities,
        )

    def read_ee_state(self) -> EEState:
        positions = self.driver.read_ee_positions()
        velocities = self.driver.read_ee_velocities()
        header = TimestampHeader.from_system_time()
        if positions is None:
            positions = EEPositions(
                header=header,
                positions=np.zeros(self.driver.get_num_ee_dofs(), dtype=np.float64),
            )
        if velocities is None:
            velocities = EEVelocities(
                header=header,
                velocities=np.zeros(self.driver.get_num_ee_dofs(), dtype=np.float64),
            )
        return EEState(
            header=header,
            ee_positions=positions,
            ee_velocities=velocities,
        )

    def _is_action_stale(self) -> bool:
        # Startup grace: if we've never seen a real action header, don't park. The first Metis publish
        # will set _latest_action_monotonic_ns; only after that point can the watchdog trip.
        if self._latest_action_monotonic_ns == 0:
            return False
        threshold_ns = int(self.config.stale_command_threshold_s * 1e9)
        return (time.monotonic_ns() - self._latest_action_monotonic_ns) > threshold_ns
