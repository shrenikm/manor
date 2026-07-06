"""
Hardware ManipulatorBackend.

Wraps the robot's control SDK via an IManipulatorDriver. The driver is
the source of truth for joint DOF / EE DOF counts; this backend just
routes ManipulatorBackend calls (send_joint_ee_command / read_joint_state /
read_ee_state / start / stop) to the corresponding driver methods.

Lifecycle ownership splits along two axes:

* Kyber lifecycle (start / stop) drives prime / unprime. start() runs
  the full bring-up sequence and moves to PRIME; stop() reverses it
  and parks at REST. This binds prime/unprime to the kylos process
  itself -- spinning kylos up means the arm is ready, spinning it
  down means the arm goes home and powers down.
* Metis lifecycle (action stream presence) drives halt / resume. If
  the action stream goes stale, the backend calls driver.halt() to
  freeze motion at the current pose without unpriming -- mode and
  energization are preserved, the arm just refuses commands. When a
  fresh action arrives again the backend calls driver.resume() and
  command flow restarts from wherever the arm currently is. No move
  to REST, no mode reset; restarting Metis just resumes where you
  left off.
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
from manor.manipulators.lite6.driver import Lite6DriverConfig
from manor.manipulators.manipulator_driver import IManipulatorDriver
from manor.manipulators.rebot_b601_dm.driver import RebotB601DmDriverConfig


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

    The per-driver config blocks carry manipulator-specific tuning knobs
    (joint speed limits, gripper torque ceilings, etc.). They live here
    -- rather than at a peer level -- because aegis.py constructs the
    driver inside the same code path that builds this backend, so
    colocating their configs keeps the YAML compact. Each block is
    optional at the parse layer; aegis.py requires the block matching
    the configured manipulator type when it builds the driver, so a
    hardware YAML only declares the block for the arm it actually runs.
    """

    minimum_watchdog_frequency_hz: float = attr.field(validator=attr.validators.gt(0.0))
    lite6_driver_config: Lite6DriverConfig | None = None
    rebot_b601_dm_driver_config: RebotB601DmDriverConfig | None = None

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
    # Newest action header seen via pet_watchdog, in monotonic ns. Zero means "no action has ever
    # arrived" -- the watchdog stays disarmed during the startup grace window before the first real
    # Metis publish.
    _latest_action_monotonic_ns: int = attr.field(init=False, default=0)
    # Halted flag. Set when the watchdog trips on a stale action stream; cleared automatically when
    # a strictly-newer action header arrives. While halted the driver has been driver.halt()'d (arm
    # holds its current pose, motors energized, mode preserved) and send_joint_ee_command is a
    # no-op so commands routed from stale Kyber state aren't pushed to the driver.
    _stopped: bool = attr.field(init=False, default=False)
    _logger: ManorLogger = attr.field(init=False)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        return ManorLogger(self.__class__.__name__)

    def start(self) -> None:
        # Kyber-lifecycle hook: bring up the arm. Reset watchdog state so a relaunched kylos always
        # starts from a known-clean baseline.
        self._latest_action_monotonic_ns = 0
        self._stopped = False
        self.driver.prime()

    def stop(self) -> None:
        # Kyber-lifecycle hook: tear down the arm (move to REST, set_state STOP, no disconnect). If
        # the watchdog had already halted the arm, unprime is still safe -- the cli helpers'
        # switch_mode + move-to-REST no-op cleanly when the arm is already there.
        self.driver.unprime()

    def _is_action_stale(self) -> bool:
        # Startup grace: if we've never seen a real action header, don't park. The first Metis publish
        # will set _latest_action_monotonic_ns; only after that point can the watchdog trip.
        if self._latest_action_monotonic_ns == 0:
            return False
        threshold_ns = int(1e9 / self.config.minimum_watchdog_frequency_hz)
        return (time.monotonic_ns() - self._latest_action_monotonic_ns) > threshold_ns

    def pet_watchdog(self, header: TimestampHeader) -> None:
        """
        Reset the staleness timer with the latest upstream-action header. Called by
        StaleCommandWatchdog on every periodic tick. Two side effects:

        * Strictly-newer header advances _latest_action_monotonic_ns so _is_action_stale stays False
          until at least one threshold-window passes without a fresh tick.
        * If the watchdog had previously halted the arm and we now see a strictly-newer header,
          auto-resume: call driver.resume() and clear the halted flag so subsequent
          send_joint_ee_command calls flow through again. This makes a Metis restart "just work"
          without operator intervention.
        """
        # Only advance the latest stamp on a strictly-newer header. The LCM subscriber holds the last
        # received message, so the watchdog hands us the same header tick after tick when Metis is
        # paused; if we treated each call as "fresh" the timer could never trip.
        if header.monotonic_ns <= self._latest_action_monotonic_ns:
            return
        self._latest_action_monotonic_ns = int(header.monotonic_ns)
        if self._stopped:
            self._logger.info("fresh action stream detected after halt -- resuming arm motion")
            self.driver.resume()
            self._stopped = False

    def send_joint_ee_command(self, joint_ee_command: JointEECommand) -> None:
        if self._stopped:
            return
        # Startup gate: if no real action has ever made it through, the JointEECommand on the wire is
        # the default-constructed one (Kyber stamps it but the inner JointCommand still carries an
        # empty joint_positions array, since JointCommand.construct_default sets joint_positions to
        # size num_joints=0). Forwarding that to the driver crashes the xarm SDK when it iterates
        # angs[i]. Drop the send until the watchdog has been pet at least once.
        if self._latest_action_monotonic_ns == 0:
            return
        if self._is_action_stale():
            # Trip the watchdog: log, halt the arm (set_state STOP -- pose / mode / energization all
            # preserved), mark _stopped so subsequent commands are dropped until pet_watchdog sees a
            # fresh header and auto-resumes.
            stale_age_s = (time.monotonic_ns() - self._latest_action_monotonic_ns) * 1e-9
            threshold_s = 1.0 / self.config.minimum_watchdog_frequency_hz
            self._logger.warning(
                f"action stream stale ({stale_age_s:.3f}s since last fresh action; threshold "
                f"{threshold_s:.3f}s = 1 / {self.config.minimum_watchdog_frequency_hz:.3f}Hz) -- "
                f"halting arm motion. Restart Metis to resume."
            )
            self._stopped = True
            self.driver.halt()
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
