"""
Tests for Soma: port shape and proprioception assembly.
"""

from __future__ import annotations

import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.soma.soma import Soma, SomaPorts
from manor.common.definitions.eef_pose import EEFPose
from manor.common.definitions.eef_twist import EEFTwist
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.utils.defaults import construct_default_eef_state, construct_default_joint_state
from manor.common.testing_utils import run_manor_tests


class TestSomaConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            Soma(robot_model_path=None, publish_frequency=0.0)

    def test_declares_expected_ports(self) -> None:
        soma = Soma(robot_model_path=None, publish_frequency=100.0)
        assert soma.num_input_ports() == 2
        assert soma.num_output_ports() == 1
        assert soma.GetInputPort(SomaPorts.INPUT_JOINT_STATE) is not None
        assert soma.GetInputPort(SomaPorts.INPUT_EEF_STATE) is not None
        assert soma.GetOutputPort(SomaPorts.OUTPUT_PROPRIOCEPTION) is not None


class TestSomaProprioception:
    def test_assembles_proprioception_from_inputs(self) -> None:
        soma = Soma(robot_model_path=None, publish_frequency=100.0)
        context = soma.CreateDefaultContext()
        soma.GetInputPort(SomaPorts.INPUT_JOINT_STATE).FixValue(
            context, AbstractValue.Make(construct_default_joint_state(num_joints=6))
        )
        soma.GetInputPort(SomaPorts.INPUT_EEF_STATE).FixValue(
            context, AbstractValue.Make(construct_default_eef_state(num_eef_dofs=1))
        )

        simulator = Simulator(soma, context)
        simulator.AdvanceTo(0.05)

        proprioception = soma.GetOutputPort(SomaPorts.OUTPUT_PROPRIOCEPTION).Eval(simulator.get_context())
        assert isinstance(proprioception, Proprioception)
        assert proprioception.joint_state is not None
        assert proprioception.eef_state is not None
        assert isinstance(proprioception.eef_pose, EEFPose)
        assert isinstance(proprioception.eef_twist, EEFTwist)


if __name__ == "__main__":
    run_manor_tests()
