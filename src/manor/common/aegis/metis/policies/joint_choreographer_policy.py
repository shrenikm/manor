"""
JointChoreographerPolicy: drives a manipulator through a YAML-configured sequence of per-joint
"sections", each running a parameterised control signal (step or sine) on a single joint while
the others hold zero velocity. Recreated as a Metis policy so it runs through the standard
metis -> kyber -> talos pipeline -- the same aegis runner drives both sim (gylos) and hardware
(kylos), which is the point: the policy exists to compare commanded vs observed joint response
on identical waypoints across the two transports.

Section state machine, per (joint, section) pair:

* START_DELAY -- emit zero velocities for start_time_delay seconds.
* PRE_ACTIVE  -- run a P-controller on the configured joint to drive the arm toward
  start_joint_positions (the pose the active phase should begin from). Advances to ACTIVE once
  measured positions are within an absolute tolerance of the start pose.
* ACTIVE      -- emit the section's control_signal on the configured joint for active_time
  seconds; other joints hold zero velocity. Per-tick target / observed velocities are recorded
  for the active joint.
* END_DELAY   -- emit zero velocities for end_time_delay seconds, then move to the next section
  (or next joint, or done).

When the last section's END_DELAY completes the policy flips is_done(), saves one figure per
joint to disk (target vs observed velocity per section), and from there on emits zero
velocities so the diagram can keep ticking.
"""

from __future__ import annotations

import os
from datetime import datetime
from enum import Enum, auto
from typing import ClassVar, Self

import attr
import numpy as np
import yaml
from matplotlib.figure import Figure

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.control.signals import ControlSignal, SineControlSignal, StepControlSignal
from manor.common.custom_types import DirPath, FilePath, JointPositionsVector, JointVelocitiesVector
from manor.common.definitions.action import Action
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError
from manor.common.logging_utils import ManorLogger
from manor.common.path_utils import create_directory_if_not_exists, get_project_root, resolve_under_project_root

# Tolerance for the P-controller pre-active "are we at start_joint_positions yet?" check, in
# radians per joint. Matches the deprecated choreographer's atol=0.01.
_PRE_ACTIVE_POSITION_TOLERANCE_RAD = 0.01

# P gain used during the PRE_ACTIVE phase to drive the arm toward start_joint_positions. Matches
# the deprecated choreographer's hard-coded 0.5.
_PRE_ACTIVE_KP = 0.5

# Subdirectory under the project root where per-run plot output directories are created. The
# project's .gitignore already excludes results/ so these intermediate artifacts don't leak into
# the working tree.
_PLOT_RESULTS_SUBDIR = "results/choreographer"

# Maximum number of sections per joint that the plot grid can lay out. Beyond 9 the layout would
# either become unreadable or need pagination -- we don't have a use case for that yet, so cap
# loud rather than silently truncating.
_MAX_PLOTTED_SECTIONS_PER_JOINT = 9


def _parse_control_signal(raw: object, context: str) -> ControlSignal:
    if not isinstance(raw, dict):
        raise AegisConfigError(f"{context} must be a mapping; got {type(raw).__name__}")
    body = dict(raw)
    type_value = body.pop("type", None)
    if not isinstance(type_value, str) or not type_value:
        raise AegisConfigError(f"{context}.type is required and must be a non-empty string")
    if type_value == "step":
        try:
            return StepControlSignal(**body)
        except TypeError as e:
            raise AegisConfigError(f"{context}: invalid fields for step signal: {e}") from e
    if type_value == "sine":
        try:
            return SineControlSignal.standard_positive_signal(**body)
        except TypeError as e:
            raise AegisConfigError(f"{context}: invalid fields for sine signal: {e}") from e
    raise AegisConfigError(f"{context}.type must be 'step' or 'sine'; got {type_value!r}")


@attr.frozen
class ChoreographedSection:
    """
    One section of the per-joint choreography. start_time_delay / end_time_delay bookend the
    active phase; active_time bounds the control_signal emission. start_joint_positions is the
    pose the PRE_ACTIVE phase drives toward before kicking off the signal.
    """

    start_time_delay: float
    end_time_delay: float
    active_time: float
    start_joint_positions: JointPositionsVector = attr.field(eq=attr.cmp_using(eq=np.array_equal))
    control_signal: ControlSignal


@attr.frozen
class JointChoreographedSections:
    """
    All sections that act on a single joint (joint_index). The choreographer iterates these in
    declared order before moving to the next joint.
    """

    joint_index: int
    choreographed_sections: tuple[ChoreographedSection, ...]


def _parse_choreography_yaml_dict(raw: dict, num_arm_dof: int | None) -> tuple[JointChoreographedSections, ...]:
    """
    Parse the joint-block-of-section-blocks structure used by the choreographer YAML. num_arm_dof is
    enforced when supplied so a section can't reference an out-of-range joint or carry a
    start_joint_positions vector of the wrong length.
    """
    parsed: list[JointChoreographedSections] = []
    for joint_block_name, joint_block in raw.items():
        context = f"choreographer.{joint_block_name}"
        if not isinstance(joint_block, dict):
            raise AegisConfigError(f"{context} must be a mapping; got {type(joint_block).__name__}")
        if "joint_index" not in joint_block:
            raise AegisConfigError(f"{context}.joint_index is required")
        joint_index = joint_block["joint_index"]
        if not isinstance(joint_index, int):
            raise AegisConfigError(f"{context}.joint_index must be an int; got {type(joint_index).__name__}")
        if num_arm_dof is not None and not 0 <= joint_index < num_arm_dof:
            raise AegisConfigError(f"{context}.joint_index {joint_index} out of range for num_arm_dof={num_arm_dof}")
        sections: list[ChoreographedSection] = []
        # Iterate sections in dict-insertion order to preserve the YAML's declared sequence.
        for section_name, section_block in joint_block.items():
            if section_name == "joint_index":
                continue
            section_context = f"{context}.{section_name}"
            if not isinstance(section_block, dict):
                raise AegisConfigError(f"{section_context} must be a mapping; got {type(section_block).__name__}")
            try:
                start_positions = np.array(section_block["start_joint_positions"], dtype=np.float64)
            except (KeyError, TypeError, ValueError) as e:
                raise AegisConfigError(f"{section_context}.start_joint_positions invalid: {e}") from e
            if num_arm_dof is not None and start_positions.shape != (num_arm_dof,):
                raise AegisConfigError(
                    f"{section_context}.start_joint_positions must have shape ({num_arm_dof},); "
                    f"got {start_positions.shape}"
                )
            try:
                section = ChoreographedSection(
                    start_time_delay=float(section_block["start_time_delay"]),
                    end_time_delay=float(section_block["end_time_delay"]),
                    active_time=float(section_block["active_time"]),
                    start_joint_positions=start_positions,
                    control_signal=_parse_control_signal(section_block["control_signal"], section_context),
                )
            except KeyError as e:
                raise AegisConfigError(f"{section_context}: missing required field {e}") from e
            sections.append(section)
        parsed.append(JointChoreographedSections(joint_index=joint_index, choreographed_sections=tuple(sections)))
    return tuple(parsed)


def _parse_inlined_sections(value: object, context: str) -> tuple[JointChoreographedSections, ...]:
    if not isinstance(value, dict):
        raise AegisConfigError(f"{context} must be a mapping; got {type(value).__name__}")
    return _parse_choreography_yaml_dict(value, num_arm_dof=None)


@attr.frozen
class JointChoreographerPolicyConfig(MetisPolicyConfigBase):
    """
    Config for JointChoreographerPolicy. The YAML schema mirrors the deprecated Lite6 choreographer
    config (jointN_choreography blocks, each with sectionN sub-blocks).

    yaml_filepath, when set, is used by from_yaml_dict to load the choreography from an external file
    -- this keeps the aegis YAML compact (the choreography itself can run hundreds of lines of section
    definitions). When unset, joint_choreographed_sections must be inlined directly in the dict that
    from_yaml_dict consumes.
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.JOINT_CHOREOGRAPHER

    num_arm_dof: int
    joint_choreographed_sections: tuple[JointChoreographedSections, ...]

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        if "yaml_filepath" in d and "joint_choreographed_sections" in d:
            raise AegisConfigError(
                "JointChoreographerPolicyConfig: pass exactly one of yaml_filepath or "
                "joint_choreographed_sections, not both"
            )
        if "yaml_filepath" in d:
            yaml_filepath = d["yaml_filepath"]
            num_arm_dof_value = d.get("num_arm_dof")
            if not isinstance(yaml_filepath, str):
                raise AegisConfigError("JointChoreographerPolicyConfig.yaml_filepath must be a string")
            if not isinstance(num_arm_dof_value, int):
                raise AegisConfigError("JointChoreographerPolicyConfig.num_arm_dof must be an int")
            return cls.from_yaml_filepath(yaml_filepath=yaml_filepath, num_arm_dof=num_arm_dof_value)
        return cls(
            **parse_attrs_yaml(
                cls,
                d,
                "JointChoreographerPolicyConfig",
                custom_parsers={
                    "joint_choreographed_sections": _parse_inlined_sections,
                },
            )
        )

    @classmethod
    def from_yaml_filepath(cls, yaml_filepath: FilePath, num_arm_dof: int) -> Self:
        """
        Load a JointChoreographerPolicyConfig from a standalone YAML file. Relative paths resolve
        against the project root so the bundled config can write
        yaml_filepath: configs/choreographer/lite6.yaml and have it work regardless of cwd. Format
        matches the deprecated Lite6 choreographer config: top-level keys are joint blocks containing
        a joint_index plus N sectionN sub-blocks; each section has start/end time delays, an active
        time, a start_joint_positions vector, and a control_signal block.
        """
        resolved = resolve_under_project_root(yaml_filepath)
        if not os.path.exists(resolved):
            raise AegisConfigError(f"choreographer YAML not found: {resolved!r}")
        with open(resolved, "r") as fp:
            raw = yaml.safe_load(fp)
        if not isinstance(raw, dict):
            raise AegisConfigError(
                f"choreographer YAML {resolved!r} must be a mapping at the top level; got {type(raw).__name__}"
            )
        return cls(
            num_arm_dof=num_arm_dof,
            joint_choreographed_sections=_parse_choreography_yaml_dict(raw, num_arm_dof=num_arm_dof),
        )


def _default_plot_output_dir() -> DirPath:
    """
    Compute a fresh timestamped directory under results/choreographer for one policy run. The factory
    runs at policy construction time so each aegis run gets its own folder; back-to-back runs at the
    same system-time second collide, which is fine -- we'd rather overwrite than silently nest.
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(get_project_root(), _PLOT_RESULTS_SUBDIR, timestamp)


def _subplots_grid_for_num_sections(num_sections: int) -> tuple[int, int]:
    """
    Pick a (rows, cols) layout for num_sections subplots that keeps each panel readable. Tuned by
    eye for the choreographer plot; raises if asked to handle more than the supported max.
    """
    if not 1 <= num_sections <= _MAX_PLOTTED_SECTIONS_PER_JOINT:
        raise ValueError(f"unsupported number of sections for plotting: {num_sections}")
    return {
        1: (1, 1),
        2: (1, 2),
        3: (1, 3),
        4: (2, 2),
        5: (2, 3),
        6: (2, 3),
        7: (3, 3),
        8: (3, 3),
        9: (3, 3),
    }[num_sections]


class _SectionStatus(Enum):
    START_DELAY = auto()
    PRE_ACTIVE = auto()
    ACTIVE = auto()
    END_DELAY = auto()

    def is_start_delay(self) -> bool:
        return self is _SectionStatus.START_DELAY

    def is_pre_active(self) -> bool:
        return self is _SectionStatus.PRE_ACTIVE

    def is_active(self) -> bool:
        return self is _SectionStatus.ACTIVE

    def is_end_delay(self) -> bool:
        return self is _SectionStatus.END_DELAY


@attr.define
class JointChoreographerPolicy:
    """
    Stateful Metis policy that walks the configured (joint, section) pairs through the
    START_DELAY / PRE_ACTIVE / ACTIVE / END_DELAY state machine. is_done() goes True after the
    last section's END_DELAY completes; subsequent step() calls return zero velocities.
    """

    joint_choreographed_sections: tuple[JointChoreographedSections, ...]
    num_arm_dof: int
    _logger: ManorLogger = attr.field(init=False)
    _plot_output_dir: DirPath = attr.field(init=False, factory=_default_plot_output_dir)

    _current_joint_idx: int = attr.field(init=False, default=0)
    _current_section_idx: int = attr.field(init=False, default=0)
    _status: _SectionStatus = attr.field(init=False, default=_SectionStatus.START_DELAY)
    _section_start_wait_time_s: float | None = attr.field(init=False, default=None)
    _section_active_start_time_s: float | None = attr.field(init=False, default=None)
    _section_end_wait_time_s: float | None = attr.field(init=False, default=None)
    _done: bool = attr.field(init=False, default=False)

    # Per-section recordings of section-relative time, target velocity, and observed velocity for
    # the active joint. Keyed by (choreography index in joint_choreographed_sections, section
    # index). Populated only during the ACTIVE phase; consumed by _save_plots when the policy
    # finishes. Lists rather than ndarrays because we don't know the section length up front.
    _section_times_map: dict[tuple[int, int], list[float]] = attr.field(init=False, factory=dict)
    _section_target_velocities_map: dict[tuple[int, int], list[float]] = attr.field(init=False, factory=dict)
    _section_observed_velocities_map: dict[tuple[int, int], list[float]] = attr.field(init=False, factory=dict)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        return ManorLogger(self.__class__.__name__)

    @classmethod
    def from_config(cls, config: JointChoreographerPolicyConfig) -> Self:
        return cls(
            joint_choreographed_sections=config.joint_choreographed_sections,
            num_arm_dof=config.num_arm_dof,
        )

    def _handle_start_delay(
        self,
        now_s: float,
        section: ChoreographedSection,
        joint_index: int,
        velocities: JointVelocitiesVector,
    ) -> JointVelocitiesVector:
        if self._section_start_wait_time_s is None:
            self._section_start_wait_time_s = now_s
        elif now_s - self._section_start_wait_time_s > section.start_time_delay:
            self._section_start_wait_time_s = None
            self._status = _SectionStatus.PRE_ACTIVE
            self._logger.info(f"[joint {joint_index + 1}][section {self._current_section_idx + 1}] start delay done")
        return velocities

    def _handle_pre_active(
        self,
        observation: Observation,
        now_s: float,
        section: ChoreographedSection,
        joint_index: int,
        velocities: JointVelocitiesVector,
    ) -> JointVelocitiesVector:
        measured = self._extract_arm_positions(observation=observation)
        if measured is None:
            return velocities
        target = section.start_joint_positions
        if np.allclose(measured, target, atol=_PRE_ACTIVE_POSITION_TOLERANCE_RAD):
            self._status = _SectionStatus.ACTIVE
            self._section_active_start_time_s = now_s
            self._logger.info(f"[joint {joint_index + 1}][section {self._current_section_idx + 1}] pre-active done")
            return velocities
        # P-controller in joint space to drive measured -> target. Same shape as the deprecated
        # implementation: gain * (target - measured), no integral term, applied across all joints
        # (not just the one section.control_signal targets).
        return _PRE_ACTIVE_KP * (target - measured)

    def _handle_active(
        self,
        now_s: float,
        section: ChoreographedSection,
        joint_index: int,
        velocities: JointVelocitiesVector,
    ) -> JointVelocitiesVector:
        if self._section_active_start_time_s is None:
            # Defensive: pre-active should have set this. If we got here without it, latch now.
            self._section_active_start_time_s = now_s
        elapsed = now_s - self._section_active_start_time_s
        if elapsed > section.active_time:
            self._section_active_start_time_s = None
            self._status = _SectionStatus.END_DELAY
            self._logger.info(f"[joint {joint_index + 1}][section {self._current_section_idx + 1}] active done")
            return velocities
        velocities[joint_index] = section.control_signal.compute_signal(time_step=elapsed)
        return velocities

    def _handle_end_delay(
        self,
        now_s: float,
        section: ChoreographedSection,
        joint_index: int,
        velocities: JointVelocitiesVector,
    ) -> JointVelocitiesVector:
        if self._section_end_wait_time_s is None:
            self._section_end_wait_time_s = now_s
            return velocities
        if now_s - self._section_end_wait_time_s <= section.end_time_delay:
            return velocities
        self._section_end_wait_time_s = None
        self._status = _SectionStatus.START_DELAY
        self._logger.info(f"[joint {joint_index + 1}][section {self._current_section_idx + 1}] end delay done")
        self._advance(joint_index=joint_index)
        return velocities

    def _advance(self, joint_index: int) -> None:
        jcs = self.joint_choreographed_sections[self._current_joint_idx]
        if self._current_section_idx < len(jcs.choreographed_sections) - 1:
            self._current_section_idx += 1
            self._logger.info(f"[joint {joint_index + 1}] section {self._current_section_idx + 1} starting")
            return
        if self._current_joint_idx < len(self.joint_choreographed_sections) - 1:
            self._current_joint_idx += 1
            self._current_section_idx = 0
            self._logger.info(f"choreographer: moving to joint {joint_index + 2}")
            return
        self._done = True
        self._logger.info("choreographer: all sections complete")
        self._save_plots()

    def _extract_arm_positions(self, observation: Observation) -> JointPositionsVector | None:
        if observation.proprioception is None:
            return None
        positions = observation.proprioception.joint_state.joint_positions.positions
        if positions.size == 0:
            return None
        # Proprioception carries the plant's full position vector (arm + EE plant DOFs); the
        # choreographer only operates on the arm block, so slice off any trailing EE positions
        # before comparing against start_joint_positions.
        return np.asarray(positions[: self.num_arm_dof], dtype=np.float64).copy()

    def _save_plots(self) -> None:
        """
        Render one figure per choreographed joint with target vs observed velocity per section,
        and write each as a PNG under _plot_output_dir. Uses matplotlib's Figure interface
        directly (not pyplot) so this is safe to call from the metis publish thread without
        touching pyplot's global state or trying to start a GUI backend.
        """
        if not self._section_times_map:
            self._logger.info("choreographer: no ACTIVE-phase samples recorded, skipping plot output")
            return
        create_directory_if_not_exists(self._plot_output_dir)
        for joint_idx, jcs in enumerate(self.joint_choreographed_sections):
            section_keys = sorted(k for k in self._section_times_map if k[0] == joint_idx)
            if not section_keys:
                continue
            fig = self._build_joint_figure(joint_index=jcs.joint_index, section_keys=section_keys)
            output_path = os.path.join(self._plot_output_dir, f"joint_{jcs.joint_index + 1}.png")
            fig.savefig(output_path)
            self._logger.info(f"choreographer: wrote {output_path}")

    def _build_joint_figure(self, joint_index: int, section_keys: list[tuple[int, int]]) -> Figure:
        nrows, ncols = _subplots_grid_for_num_sections(num_sections=len(section_keys))
        fig = Figure()
        fig.suptitle(f"Choreographer Analysis Plots - Joint {joint_index + 1}")
        axes = fig.subplots(nrows=nrows, ncols=ncols)
        # subplots returns a single Axes when (1, 1), a 1D array when one of nrows/ncols is 1, and
        # a 2D array otherwise. Flatten to a uniform list so the indexing below stays simple.
        if nrows == 1 and ncols == 1:
            axes_list = [axes]
        elif nrows == 1 or ncols == 1:
            axes_list = list(axes)
        else:
            axes_list = [ax for axes_row in axes for ax in axes_row]
        for plot_idx, key in enumerate(section_keys):
            ax = axes_list[plot_idx]
            t = np.asarray(self._section_times_map[key], dtype=np.float64)
            tv = np.asarray(self._section_target_velocities_map[key], dtype=np.float64)
            ov = np.asarray(self._section_observed_velocities_map[key], dtype=np.float64)
            ax.set_title(f"Section {key[1] + 1}/{len(section_keys)}")
            ax.plot(t, tv, color="blue", label="Target velocity")
            ax.plot(t, ov, color="orange", label="Observed velocity")
            ax.set_xlabel("t (sec)")
            ax.set_ylabel("qdot (rad/s)")
            ax.legend(loc="best")
        fig.tight_layout()
        return fig

    def _record_if_active(
        self,
        observation: Observation,
        now_s: float,
        velocities: JointVelocitiesVector,
    ) -> None:
        """
        Stash a (section-relative time, target velocity, observed velocity) sample for the active
        joint when the policy is in the ACTIVE phase. Other phases are bookkeeping; their data isn't
        useful for the per-section comparison plot. Section-relative time uses
        _section_active_start_time_s so the per-section x-axes start at zero on the plot.
        """
        if not self._status.is_active() or self._section_active_start_time_s is None:
            return
        if observation.proprioception is None:
            return
        observed = observation.proprioception.joint_state.joint_velocities.velocities
        if observed.size == 0:
            return
        jcs = self.joint_choreographed_sections[self._current_joint_idx]
        joint_index = jcs.joint_index
        key = (self._current_joint_idx, self._current_section_idx)
        section_t_s = now_s - self._section_active_start_time_s
        self._section_times_map.setdefault(key, []).append(section_t_s)
        self._section_target_velocities_map.setdefault(key, []).append(float(velocities[joint_index]))
        self._section_observed_velocities_map.setdefault(key, []).append(float(observed[joint_index]))

    def _compute_velocities(self, observation: Observation, now_s: float) -> JointVelocitiesVector:
        if self._done:
            return np.zeros(self.num_arm_dof, dtype=np.float64)
        jcs = self.joint_choreographed_sections[self._current_joint_idx]
        section = jcs.choreographed_sections[self._current_section_idx]
        joint_index = jcs.joint_index
        velocities = np.zeros(self.num_arm_dof, dtype=np.float64)
        if self._status is _SectionStatus.START_DELAY:
            return self._handle_start_delay(
                now_s=now_s, section=section, joint_index=joint_index, velocities=velocities
            )
        if self._status is _SectionStatus.PRE_ACTIVE:
            return self._handle_pre_active(
                observation=observation,
                now_s=now_s,
                section=section,
                joint_index=joint_index,
                velocities=velocities,
            )
        if self._status is _SectionStatus.ACTIVE:
            return self._handle_active(now_s=now_s, section=section, joint_index=joint_index, velocities=velocities)
        return self._handle_end_delay(now_s=now_s, section=section, joint_index=joint_index, velocities=velocities)

    def is_done(self) -> bool:
        return self._done

    def step(self, observation: Observation) -> Action:
        header = TimestampHeader.from_system_time()
        now_s = header.system_ns * 1e-9
        velocities = self._compute_velocities(observation=observation, now_s=now_s)
        self._record_if_active(observation=observation, now_s=now_s, velocities=velocities)
        return Action(
            header=header,
            joint_command=JointCommand(
                header=header,
                joint_velocities=JointVelocities(header=header, velocities=velocities),
            ),
        )
