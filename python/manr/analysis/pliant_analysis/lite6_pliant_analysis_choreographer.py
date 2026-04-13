from __future__ import annotations

import os
from enum import Enum, auto
from typing import Sequence, Tuple

import attr
import matplotlib.pyplot as plt
import numpy as np
import yaml
from pydrake.systems.framework import BasicVector, Context, LeafSystem

from manr.common.control.signals import ControlSignal, SineControlSignal, StepControlSignal
from manr.common.custom_types import FilePath, JointPositionsVector
from manr.common.exceptions import Lite6PliantChoreographerError, Lite6PliantError
from manr.common.logging_utils import MRLogger
from manr.lite6.pliant.lite6_pliant_utils import Lite6PliantConfig
from manr.lite6.utils.lite6_model_utils import LITE6_DOF, Lite6ControlType

CC_PE_INPUT_PORT = "cc_pe_input_port"
CC_VE_INPUT_PORT = "cc_ve_input_port"
CC_OUTPUT_PORT = "cc_output_port"
CHOREOGRAPHER_CONFIG_YAML_FILENAME = "choreographer_config.yaml"


def get_choreographer_config_yaml_filepath() -> FilePath:
    return os.path.join(
        os.path.dirname(__file__),
        CHOREOGRAPHER_CONFIG_YAML_FILENAME,
    )


@attr.frozen
class ChoreographedSection:
    start_time_delay: float
    end_time_delay: float
    active_time: float
    start_joint_positions: JointPositionsVector
    control_signal: ControlSignal


@attr.frozen
class JointChoreographedSections:
    joint_index: int
    choreographed_sections: Sequence[ChoreographedSection]

    def __len__(self) -> int:
        return len(self.choreographed_sections)

    def __getitem__(self, index: int) -> ChoreographedSection:
        return self.choreographed_sections[index]


@attr.frozen
class Lite6PliantChoreographer:
    joint_choreographed_sections: Sequence[JointChoreographedSections]

    def __len__(self) -> int:
        return len(self.joint_choreographed_sections)

    def __getitem__(self, index: int) -> JointChoreographedSections:
        return self.joint_choreographed_sections[index]

    @classmethod
    def from_yaml(
        cls,
        yaml_filepath: FilePath,
    ) -> Lite6PliantChoreographer:
        with open(file=yaml_filepath, mode="r") as stream:
            parsed_yaml = yaml.safe_load(stream=stream)
        jcs = []
        # TODO: Validate joint index duplicates?
        for jcs_dict in parsed_yaml.values():
            joint_index = jcs_dict.pop("joint_index")
            sections = []
            for sections_dict in jcs_dict.values():
                control_signal_dict = sections_dict.pop("control_signal")
                control_signal_type = control_signal_dict.pop("type")
                if control_signal_type == "step":
                    control_signal = StepControlSignal(**control_signal_dict)
                elif control_signal_type == "sine":
                    control_signal = SineControlSignal.standard_positive_signal(
                        **control_signal_dict,
                    )
                else:
                    raise Lite6PliantChoreographerError("Invalid control signal type")
                cs = ChoreographedSection(
                    start_time_delay=sections_dict.pop("start_time_delay"),
                    end_time_delay=sections_dict.pop("end_time_delay"),
                    active_time=sections_dict.pop("active_time"),
                    start_joint_positions=np.array(sections_dict.pop("start_joint_positions")),
                    control_signal=control_signal,
                )
                sections.append(cs)
            jcs.append(
                JointChoreographedSections(
                    joint_index=joint_index,
                    choreographed_sections=sections,
                )
            )
        return cls(
            joint_choreographed_sections=jcs,
        )


class ChoreographedSectionStatus(Enum):
    START_DELAY = auto()
    PRE_ACTIVE = auto()
    ACTIVE = auto()
    END_DELAY = auto()


class Lite6PliantChoreographerController(LeafSystem):
    def __init__(
        self,
        config: Lite6PliantConfig,
        choreographer: Lite6PliantChoreographer,
    ):
        # Currently only designed for velocity control
        if config.lite6_control_type != Lite6ControlType.VELOCITY:
            raise Lite6PliantError("Only velocity control is supported as of now.")

        super().__init__()

        self.config = config
        self.choreographer = choreographer
        self.kp = 0.5
        self._logger = MRLogger(self.__class__.__name__)

        # Variables
        self.num_choreographed_joints = len(self.choreographer)

        # States.
        self._current_joint_ind = 0
        self._current_section_ind = 0
        self._status = ChoreographedSectionStatus.START_DELAY
        self._section_start_wait_time = None
        self._section_active_time = None
        self._section_end_wait_time = None
        self._done = False

        # Storing the poses for plotting.
        # Note that we can use Drake's VectorLogSink for this, but in this case
        # We want to correctly partition the data for plotting, so we do it manually.
        self._current_recorded_times = np.empty(0, dtype=np.float64)
        self._current_recorded_target_velocities = np.empty(0, dtype=np.float64)
        self._current_recorded_estimated_velocities = np.empty(0, dtype=np.float64)
        self._times_map = {joint_ind: {} for joint_ind in range(self.num_choreographed_joints)}
        self._target_velocities_map = {joint_ind: {} for joint_ind in range(self.num_choreographed_joints)}
        self._estimated_velocities_map = {joint_ind: {} for joint_ind in range(self.num_choreographed_joints)}

        self.cc_pe_input_port = self.DeclareVectorInputPort(
            name=CC_PE_INPUT_PORT,
            size=LITE6_DOF,
        )
        self.cc_ve_input_port = self.DeclareVectorInputPort(
            name=CC_VE_INPUT_PORT,
            size=LITE6_DOF,
        )

        self.cc_output_port = self.DeclareVectorOutputPort(
            name=CC_OUTPUT_PORT,
            size=LITE6_DOF,
            calc=self._compute_control_velocities_output,
        )

    @staticmethod
    def _get_subplots_size_for_num_sections(num_sections: int) -> Tuple[int, int]:
        assert 1 <= num_sections <= 9
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

    def plot_recordings(self) -> None:
        self._logger.info("Generating plots!")
        for joint_ind in range(self.num_choreographed_joints):
            assert (
                len(self._times_map[joint_ind])
                == len(self._target_velocities_map[joint_ind])
                == len(self._estimated_velocities_map[joint_ind])
            )
            joint_index = self.choreographer[joint_ind].joint_index
            num_sections = len(self._times_map[joint_ind])
            nrows, ncols = self._get_subplots_size_for_num_sections(num_sections=num_sections)
            fig, axes = plt.subplots(
                nrows=nrows,
                ncols=ncols,
            )
            # Flattening axes if it returns a nested 2d list.
            if nrows == 1 and ncols == 1:
                axes = [axes]
            if nrows > 1:
                axes = [ax for axes_row in axes for ax in axes_row]
            fig.suptitle(f"Choreographer Analysis Plots - Joint {joint_index + 1}")
            for section_ind in range(num_sections):
                ax = axes[section_ind]
                t = self._times_map[joint_ind][section_ind]
                tv = self._target_velocities_map[joint_ind][section_ind]
                ev = self._estimated_velocities_map[joint_ind][section_ind]

                # Make sure that the times are offset and start from zero for each
                # of the section plots.
                zero_start_t = np.copy(t)
                zero_start_t -= zero_start_t[0]

                ax.set_title(f"Section {section_ind + 1}/{num_sections}")
                ax.plot(zero_start_t, tv, color="blue", label="Target velocities")
                ax.plot(zero_start_t, ev, color="orange", label="Estimated velocities")
                ax.set_xlabel("t (sec)")
                ax.set_ylabel("qdot (rad/s)")

            plt.tight_layout()
            plt.show()

    def is_done(self) -> bool:
        return self._done

    def _compute_control_velocities_output(
        self,
        context: Context,
        output_vector: BasicVector,
    ) -> None:
        # TODO: Split into cleaner smaller functions.
        if self._done:
            output_vector.SetFromVector(value=np.zeros(LITE6_DOF, dtype=np.float64))
            return

        current_time = context.get_time()
        pe_vector = self.cc_pe_input_port.Eval(context)
        ve_vector = self.cc_ve_input_port.Eval(context)

        # self._current_joint_ind refers to the index of the Joint Choreographed Section
        # that is currently active. It does not point to the actual joint being moved.
        jcs = self.choreographer[self._current_joint_ind]
        cs = jcs[self._current_section_ind]

        # joint_index is the actual joint index in the state that is currently being actuated.
        joint_index = jcs.joint_index

        velocities_output_vector = np.zeros(LITE6_DOF, dtype=np.float64)

        if self._status == ChoreographedSectionStatus.START_DELAY:
            # We wait by sending zero velocities.
            if self._section_start_wait_time is None:
                self._section_start_wait_time = current_time
            elif current_time - self._section_start_wait_time > cs.start_time_delay:
                # Move to active.
                self._section_start_wait_time = None
                self._status = ChoreographedSectionStatus.PRE_ACTIVE
                self._logger.info(
                    f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] Start delay done."
                )
        elif self._status == ChoreographedSectionStatus.PRE_ACTIVE:
            if np.allclose(pe_vector, cs.start_joint_positions, atol=0.01):
                self._status = ChoreographedSectionStatus.ACTIVE
                self._section_active_time = current_time
                self._logger.info(
                    f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] Pre active done."
                )
            else:
                # P controller to get the joints to the start positions.
                velocities_output_vector = self.kp * (cs.start_joint_positions - pe_vector)
        elif self._status == ChoreographedSectionStatus.ACTIVE:
            assert self._section_active_time is not None
            if current_time - self._section_active_time > cs.active_time:
                self._section_active_time = None
                self._status = ChoreographedSectionStatus.END_DELAY

                # Active section is done. Add the current recorded data to the maps.
                self._times_map[self._current_joint_ind][self._current_section_ind] = np.copy(
                    self._current_recorded_times
                )
                self._target_velocities_map[self._current_joint_ind][self._current_section_ind] = np.copy(
                    self._current_recorded_target_velocities
                )
                self._estimated_velocities_map[self._current_joint_ind][self._current_section_ind] = np.copy(
                    self._current_recorded_estimated_velocities
                )

                # Clear out the existing current data vectors.
                self._current_recorded_times = np.empty(0, dtype=np.float64)
                self._current_recorded_target_velocities = np.empty(0, dtype=np.float64)
                self._current_recorded_estimated_velocities = np.empty(0, dtype=np.float64)

                self._logger.info(f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] Active done.")
            else:
                signal = cs.control_signal.compute_signal(time_step=current_time - self._section_active_time)
                velocities_output_vector[joint_index] = signal
                # Record data.
                self._current_recorded_times = np.hstack((self._current_recorded_times, current_time))
                self._current_recorded_target_velocities = np.hstack((self._current_recorded_target_velocities, signal))
                self._current_recorded_estimated_velocities = np.hstack(
                    (
                        self._current_recorded_estimated_velocities,
                        ve_vector[joint_index],
                    )
                )

        elif self._status == ChoreographedSectionStatus.END_DELAY:
            # We wait by sending zero velocities.
            if self._section_end_wait_time is None:
                self._section_end_wait_time = current_time
            elif current_time - self._section_end_wait_time > cs.end_time_delay:
                # Move to active.
                self._section_end_wait_time = None
                self._status = ChoreographedSectionStatus.START_DELAY
                self._logger.info(f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] End delay done.")

                # If this is the last section, we go to the next joint and reset the section index to 0
                if self._current_section_ind == len(jcs) - 1:
                    if self._current_joint_ind == len(self.choreographer) - 1:
                        # If this is also the last joint, we are done.
                        self._done = True
                        self._logger.info("Choreography done!")
                    else:
                        self._logger.info(
                            f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] Current joint completed. Moving to the next one."
                        )
                        self._current_joint_ind += 1
                        self._current_section_ind = 0
                else:
                    # Go to the next section.
                    self._logger.info(
                        f"[Joint {joint_index + 1}][Section {self._current_section_ind + 1}] Current section completed. Moving to the next one."
                    )
                    self._current_section_ind += 1

        output_vector.SetFromVector(
            value=velocities_output_vector,
        )
