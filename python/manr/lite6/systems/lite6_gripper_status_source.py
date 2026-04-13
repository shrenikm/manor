from typing import List

import numpy as np
from pydrake.common.value import AbstractValue, Value
from pydrake.systems.framework import Context, LeafSystem

from manr.lite6.utils.lite6_model_utils import Lite6GripperStatus

LITE6_GSS_OP_NAME = "gss_output"


# TODO: Tests.
class Lite6GripperStatusSource(LeafSystem):
    def __init__(
        self,
        times: List[float],
        statuses: List[Lite6GripperStatus],
    ):
        super().__init__()

        # Lists must be of the same size.
        assert len(times) == len(statuses)
        # First time must be 0. This means that the gripper status at the
        # start time must be provided.
        assert np.isclose(times[0], 0.0)

        self._n = len(times)
        self._index = 0

        self._times = times
        self._statuses = statuses

        self.gss_output_port = self.DeclareAbstractOutputPort(
            name=LITE6_GSS_OP_NAME,
            alloc=lambda: Value(Lite6GripperStatus.NEUTRAL),
            calc=self._compute_gripper_status_source_output,
            prerequisites_of_calc={
                self.time_ticket(),
            },
        )

    def _compute_gripper_status_source_output(
        self,
        context: Context,
        output_value: AbstractValue,
    ) -> None:
        # If we're at the last index, the rest of the simulation will have this system
        # output the final state at its output port.
        if self._index == self._n - 1:
            output_value.set_value(self._statuses[self._index])
            return

        current_time = context.get_time()
        while self._index < self._n - 1 and current_time >= self._times[self._index + 1]:
            self._index += 1

        output_value.set_value(self._statuses[self._index])
