"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an IManipulatorDriver. The driver is
the source of truth for joint DOF / EE DOF counts; this backend just
routes ManipulatorBackend calls (send_joint_ee_command / read_joint_state /
read_ee_state / start / stop) to the corresponding driver methods.

The backend also runs a stale-command watchdog: every action header
seen via pet_watchdog is stamped against time.monotonic_ns; if no
fresh action arrives within 1 / minimum_watchdog_frequency_hz seconds
the backend calls driver.unprime() to drop the arm to ZERO + STOP and
flips into a "parked" state where subsequent send_joint_ee_command
calls are dropped. The trip is sticky -- a fresh action does NOT
auto-rearm the backend, because the user spec calls out that we must
not let a rogue policy reattach after an outage. Only a fresh start()
(i.e. an aegis restart) re-primes the driver and re-arms the watchdog.
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


@attr.frozen
class HardwareManipulatorBackendConfig:
    """
    Hardware-specific knobs for the manipulator backend. joint DOF / EE
    DOF counts intentionally live on the driver, not here.

    minimum_watchdog_frequency_hz sets the lower bound on the rate at
    which the backend expects pet_watchdog calls (proxying for the
    Metis Action stream). The staleness threshold the backend trips on
    is 1 / minimum_watchdog_frequency_hz seconds. Required (no default)
    -- every hardware run must declare it in the YAML so the operator
    has consciously chosen a value matched to the policy's publish rate.
    """

    minimum_watchdog_frequency_hz: float = attr.field(validator=attr.validators.gt(0.0))

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "hardware_backend_config"))


@attr.define
class HardwareManipulatorBackend:
    """
    ManipulatorBackend that delegates to an IManipulatorDriver.
    """

    driver: IManipulatorDriver
    config: HardwareManipulatorBackendConfig
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
        # Startup gate: if no real action has ever made it through, the JointEECommand on the wire
        # is the default-constructed one (Kyber stamps it but the inner JointCommand still carries
        # an empty joint_positions array, since JointCommand.construct_default sets joint_positions
        # to size num_joints=0). Forwarding that to the driver crashes the xarm SDK when it iterates
        # angs[i]. Drop the send until the watchdog has been pet at least once.
        if self._latest_action_monotonic_ns == 0:
            return
        if self._is_action_stale():
            # Trip the watchdog: log, unprime the arm (move to ZERO + STOP, no disconnect), park.
            stale_age_s = (time.monotonic_ns() - self._latest_action_monotonic_ns) * 1e-9
            threshold_s = 1.0 / self.config.minimum_watchdog_frequency_hz
            self._logger.warning(
                f"action stream stale ({stale_age_s:.3f}s since last fresh action; threshold "
                f"{threshold_s:.3f}s = 1 / {self.config.minimum_watchdog_frequency_hz:.3f}Hz) -- "
                f"parking the arm. Restart aegis to re-arm."
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
        threshold_ns = int(1e9 / self.config.minimum_watchdog_frequency_hz)
        return (time.monotonic_ns() - self._latest_action_monotonic_ns) > threshold_ns
