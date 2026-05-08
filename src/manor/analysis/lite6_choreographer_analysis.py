"""
Lite6 choreographer analysis.

Drives a JointChoreographerPolicy against a Gaia-backed simulation, records the per-joint
target velocity (what the policy emitted) and observed velocity (what the plant integrated)
each tick, and on completion produces per-joint section-by-section comparison plots.

This script does NOT use the full aegis pipeline (no metis / kyber / talos / LCM). The
choreographer's purpose is to characterise the in-plant tracking quality of joint-velocity
commands; routing those commands through aegis would add a transport hop that's not relevant
to the analysis. The runner instead steps the policy directly, applies its joint-velocity
output to gaia.apply_joint_velocity_command, and advances the gaia simulator on the same dt.

Plots: one figure per choreographed joint, with one subplot per section showing target vs
observed joint velocity for that section's ACTIVE phase. Mirrors the deprecated pliant
choreographer plot layout.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence

import matplotlib.pyplot as plt
import numpy as np

from manor.common.aegis.gaia.gaia import Gaia, GaiaConfig
from manor.common.aegis.metis.policies.joint_choreographer_policy import (
    DEFAULT_LITE6_CHOREOGRAPHER_YAML_FILEPATH,
    JointChoreographerPolicy,
    JointChoreographerPolicyConfig,
)
from manor.common.custom_types import FilePath
from manor.common.definitions.observation import Observation
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.logging_utils import ManorLogger
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant

_LOGGER = ManorLogger("lite6_choreographer_analysis")

# Tick rate the analysis loop runs at (Hz). Matches the deprecated pliant choreographer's
# inner-controller cadence; high enough to see the control_signal shape on the plot, low enough
# that the recordings stay tractable in memory.
_ANALYSIS_TICK_HZ = 100.0


def _build_observation_from_gaia(gaia: Gaia, num_arm_dof: int) -> Observation:
    """
    Wrap gaia.read_joint_state into an Observation the policy can consume. The arm-only slice of
    the plant's q vector becomes the joint_positions; everything else uses construct_default.
    """
    joint_state = gaia.read_joint_state()
    # Slice plant q / v down to the arm DOF the policy reasons about. The policy's
    # _extract_arm_positions trusts whatever it gets, so we must only feed arm-sized data.
    arm_positions = joint_state.joint_positions.positions[:num_arm_dof].copy()
    arm_velocities = joint_state.joint_velocities.velocities[:num_arm_dof].copy()
    proprioception = Proprioception.construct_default(num_joints=num_arm_dof)
    return Observation(
        header=TimestampHeader.from_system_time(),
        proprioception=Proprioception(
            header=proprioception.header,
            joint_state=type(joint_state)(
                header=joint_state.header,
                joint_positions=type(joint_state.joint_positions)(
                    header=joint_state.joint_positions.header,
                    positions=arm_positions,
                ),
                joint_velocities=type(joint_state.joint_velocities)(
                    header=joint_state.joint_velocities.header,
                    velocities=arm_velocities,
                ),
            ),
        ),
    )


def run_choreographer_analysis(
    yaml_filepath: FilePath = DEFAULT_LITE6_CHOREOGRAPHER_YAML_FILEPATH,
    variant: Lite6Variant = Lite6Variant.PARALLEL_GRIPPER_NORMAL,
) -> None:
    """
    Drive the choreographer policy against an in-process Gaia, recording target / observed
    joint velocities per section, and produce comparison plots when the policy reports done.
    """
    config = JointChoreographerPolicyConfig.from_yaml_filepath(yaml_filepath=yaml_filepath, num_arm_dof=LITE6_ARM_DOF)
    policy = JointChoreographerPolicy.from_config(config)

    manipulator_model = Lite6Model(variant=variant)
    gaia = Gaia(manipulator_model=manipulator_model, config=GaiaConfig(enable_meshcat=False))
    gaia.finalize()

    dt_s = 1.0 / _ANALYSIS_TICK_HZ
    num_arm_dof = config.num_arm_dof

    # times_map / target_velocities_map / observed_velocities_map are nested:
    #   outer key = (joint_idx_in_choreography, section_idx)
    #   value = list of floats (one per ACTIVE-phase tick)
    # Records only ACTIVE-phase samples -- the START_DELAY / PRE_ACTIVE / END_DELAY phases are
    # bookkeeping; their data isn't useful for the per-section comparison plot.
    times_map: dict[tuple[int, int], list[float]] = {}
    target_velocities_map: dict[tuple[int, int], list[float]] = {}
    observed_velocities_map: dict[tuple[int, int], list[float]] = {}

    sim_time_s = 0.0
    while not policy.is_done():
        observation = _build_observation_from_gaia(gaia, num_arm_dof=num_arm_dof)
        action = policy.step(observation)
        target_v = action.joint_command.joint_velocities.velocities
        joint_state = observation.proprioception.joint_state
        observed_v = joint_state.joint_velocities.velocities

        # Record only when the policy is in the ACTIVE phase. Inspecting policy state via private
        # attributes is intentional: this script is the policy's first-party analysis tool and
        # the alternative -- a separate "is_active" public method -- would clutter the policy's
        # interface for every other consumer.
        if policy._status.name == "ACTIVE":
            key = (policy._current_joint_idx, policy._current_section_idx)
            jcs = config.joint_choreographed_sections[policy._current_joint_idx]
            joint_index = jcs.joint_index
            times_map.setdefault(key, []).append(sim_time_s)
            target_velocities_map.setdefault(key, []).append(float(target_v[joint_index]))
            observed_velocities_map.setdefault(key, []).append(float(observed_v[joint_index]))

        gaia.apply_joint_velocity_command(action.joint_command.joint_velocities)
        sim_time_s += dt_s
        gaia.advance_to(sim_time_s)

    _LOGGER.info("choreography complete; generating plots")
    _plot_recordings(
        choreographed_sections=config.joint_choreographed_sections,
        times_map=times_map,
        target_velocities_map=target_velocities_map,
        observed_velocities_map=observed_velocities_map,
    )
    gaia.shutdown()


def _plot_recordings(
    choreographed_sections: Sequence,
    times_map: dict[tuple[int, int], list[float]],
    target_velocities_map: dict[tuple[int, int], list[float]],
    observed_velocities_map: dict[tuple[int, int], list[float]],
) -> None:
    """
    One figure per joint. Each figure carries one subplot per recorded section, showing target vs
    observed velocity over the ACTIVE phase. Layout mirrors the deprecated pliant choreographer
    so longtime users find it familiar.
    """
    for joint_idx, jcs in enumerate(choreographed_sections):
        joint_index = jcs.joint_index
        section_keys = sorted(k for k in times_map if k[0] == joint_idx)
        if not section_keys:
            continue
        nrows, ncols = _subplots_grid_for_num_sections(num_sections=len(section_keys))
        fig, axes = plt.subplots(nrows=nrows, ncols=ncols)
        if nrows == 1 and ncols == 1:
            axes = [axes]
        elif nrows > 1:
            axes = [ax for axes_row in axes for ax in axes_row]
        fig.suptitle(f"Choreographer Analysis Plots - Joint {joint_index + 1}")
        for plot_idx, key in enumerate(section_keys):
            ax = axes[plot_idx]
            t = np.asarray(times_map[key], dtype=np.float64)
            tv = np.asarray(target_velocities_map[key], dtype=np.float64)
            ov = np.asarray(observed_velocities_map[key], dtype=np.float64)
            zero_start_t = t - t[0]
            ax.set_title(f"Section {key[1] + 1}/{len(section_keys)}")
            ax.plot(zero_start_t, tv, color="blue", label="Target velocity")
            ax.plot(zero_start_t, ov, color="orange", label="Observed velocity")
            ax.set_xlabel("t (sec)")
            ax.set_ylabel("qdot (rad/s)")
            ax.legend(loc="best")
        plt.tight_layout()
    plt.show()


def _subplots_grid_for_num_sections(num_sections: int) -> tuple[int, int]:
    if not 1 <= num_sections <= 9:
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


def _main() -> None:
    parser = argparse.ArgumentParser(description="Run the Lite6 choreographer analysis and plot recordings.")
    parser.add_argument(
        "--yaml",
        default=DEFAULT_LITE6_CHOREOGRAPHER_YAML_FILEPATH,
        help="Path to the choreographer YAML config.",
    )
    parser.add_argument(
        "--variant",
        default=Lite6Variant.PARALLEL_GRIPPER_NORMAL.value,
        choices=[v.value for v in Lite6Variant],
        help="Lite6 variant to instantiate.",
    )
    args = parser.parse_args()
    run_choreographer_analysis(
        yaml_filepath=args.yaml,
        variant=Lite6Variant(args.variant),
    )


if __name__ == "__main__":
    _main()
