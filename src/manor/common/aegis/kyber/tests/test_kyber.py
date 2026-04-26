"""
Tests for Kyber: port shape, controller dispatch, and YAML
round-tripping for ``KyberConfig``.
"""

from __future__ import annotations

import numpy as np
import pytest
from pydrake.common.value import AbstractValue
from pydrake.systems.analysis import Simulator

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberController,
    KyberControllerManager,
    KyberControllerType,
)
from manor.common.aegis.kyber.controllers.passthrough_controller import (
    PassthroughController,
    PassthroughControllerConfig,
)
from manor.common.aegis.kyber.controllers.zero_velocity_controller import (
    ZeroVelocityController,
    ZeroVelocityControllerConfig,
)
from manor.common.aegis.kyber.kyber import Kyber, KyberConfig, KyberPorts
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import LITE6_ARM_DOF, Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_lite6() -> Lite6Model:
    return Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL)


def _make_kyber(**overrides) -> Kyber:
    defaults = dict(
        controller=ZeroVelocityController(num_dof=LITE6_ARM_DOF),
        publish_frequency=100.0,
    )
    defaults.update(overrides)
    return Kyber(**defaults)


def _make_action(positions: np.ndarray) -> Action:
    return Action(
        header=TimestampHeader(monotonic_ns=1, system_ns=2),
        joint_positions=JointPositions(
            header=TimestampHeader(monotonic_ns=3, system_ns=4),
            positions=positions,
        ),
    )


def _make_proprioception(n_joints: int) -> Proprioception:
    return Proprioception(
        header=TimestampHeader(monotonic_ns=5, system_ns=6),
        joint_state=JointState(
            header=TimestampHeader(monotonic_ns=7, system_ns=8),
            joint_positions=JointPositions(
                header=TimestampHeader(monotonic_ns=9, system_ns=10),
                positions=np.zeros(n_joints, dtype=np.float64),
            ),
            joint_velocities=JointVelocities(
                header=TimestampHeader(monotonic_ns=11, system_ns=12),
                velocities=np.zeros(n_joints, dtype=np.float64),
            ),
        ),
    )


def _fix_inputs(kyber: Kyber, context, action: Action, proprioception: Proprioception) -> None:
    kyber.GetInputPort(KyberPorts.INPUT_ACTION).FixValue(context, AbstractValue.Make(action))
    kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION).FixValue(context, AbstractValue.Make(proprioception))


def _read_command(kyber: Kyber, context) -> Command:
    return kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND).Eval(context)


class _RecordingController:
    def __init__(self, canned_command: Command) -> None:
        self.canned = canned_command
        self.calls: list[tuple[Action, Proprioception]] = []

    def step(self, action: Action, proprioception: Proprioception) -> Command:
        self.calls.append((action, proprioception))
        return self.canned


class TestKyberConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            _make_kyber(publish_frequency=0.0)
        with pytest.raises(ValueError):
            _make_kyber(publish_frequency=-10.0)

    def test_declares_expected_ports(self) -> None:
        kyber = _make_kyber()
        assert kyber.num_input_ports() == 2
        assert kyber.num_output_ports() == 1
        assert kyber.GetInputPort(KyberPorts.INPUT_ACTION) is not None
        assert kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION) is not None
        assert kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND) is not None

    def test_owns_no_plant(self) -> None:
        kyber = _make_kyber()
        assert not hasattr(kyber, "plant")

    def test_stores_publish_frequency(self) -> None:
        kyber = _make_kyber(publish_frequency=250.0)
        assert kyber.publish_frequency == 250.0


class TestKyberControllerDispatch:
    def test_periodic_update_calls_controller_with_inputs(self) -> None:
        canned = Command(
            header=TimestampHeader.from_system_time(),
            joint_velocities=JointVelocities(
                header=TimestampHeader.from_system_time(),
                velocities=np.zeros(LITE6_ARM_DOF),
            ),
        )
        controller = _RecordingController(canned)
        kyber = _make_kyber(controller=controller)
        context = kyber.CreateDefaultContext()
        _fix_inputs(
            kyber,
            context,
            _make_action(np.zeros(LITE6_ARM_DOF)),
            _make_proprioception(n_joints=LITE6_ARM_DOF),
        )

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)

        assert len(controller.calls) >= 1
        last_action, last_proprioception = controller.calls[-1]
        assert isinstance(last_action, Action)
        assert isinstance(last_proprioception, Proprioception)

    def test_zero_velocity_controller_emits_zero_velocity_command(self) -> None:
        kyber = _make_kyber()
        context = kyber.CreateDefaultContext()
        _fix_inputs(
            kyber,
            context,
            _make_action(np.array([0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8], dtype=np.float64)),
            _make_proprioception(n_joints=LITE6_ARM_DOF),
        )

        simulator = Simulator(kyber, context)
        simulator.AdvanceTo(0.05)

        command = _read_command(kyber, simulator.get_context())
        assert command.joint_velocities is not None
        np.testing.assert_array_equal(command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))


class TestZeroVelocityController:
    def test_is_a_controller(self) -> None:
        assert isinstance(ZeroVelocityController(num_dof=LITE6_ARM_DOF), KyberController)

    def test_emits_zero_velocity_regardless_of_inputs(self) -> None:
        controller = ZeroVelocityController(num_dof=LITE6_ARM_DOF)
        action = _make_action(np.full(LITE6_ARM_DOF, 0.5))
        proprioception = _make_proprioception(n_joints=LITE6_ARM_DOF)
        command = controller.step(action, proprioception)
        assert command.joint_velocities is not None
        np.testing.assert_array_equal(command.joint_velocities.velocities, np.zeros(LITE6_ARM_DOF))


class TestPassthroughController:
    def test_is_a_controller(self) -> None:
        assert isinstance(PassthroughController(num_dof=LITE6_ARM_DOF), KyberController)

    def test_passes_joint_positions_through(self) -> None:
        controller = PassthroughController(num_dof=LITE6_ARM_DOF)
        positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)
        action = _make_action(positions)
        command = controller.step(action, _make_proprioception(n_joints=LITE6_ARM_DOF))
        assert command.joint_positions is not None
        np.testing.assert_array_equal(command.joint_positions.positions, positions)


class TestKyberControllerConfigs:
    def test_zero_velocity_config_pins_enum(self) -> None:
        assert ZeroVelocityControllerConfig.CONTROLLER_TYPE is KyberControllerType.ZERO_VELOCITY

    def test_passthrough_config_pins_enum(self) -> None:
        assert PassthroughControllerConfig.CONTROLLER_TYPE is KyberControllerType.PASSTHROUGH


class TestKyberControllerManager:
    def test_from_config_zero_velocity(self) -> None:
        controller = KyberControllerManager.from_config(
            ZeroVelocityControllerConfig(num_dof=LITE6_ARM_DOF),
            manipulator_model=_make_lite6(),
        )
        assert isinstance(controller, ZeroVelocityController)
        assert controller.num_dof == LITE6_ARM_DOF

    def test_from_config_passthrough(self) -> None:
        controller = KyberControllerManager.from_config(
            PassthroughControllerConfig(num_dof=LITE6_ARM_DOF),
            manipulator_model=_make_lite6(),
        )
        assert isinstance(controller, PassthroughController)

    def test_config_from_yaml_dict_zero_velocity(self) -> None:
        config = KyberControllerManager.config_from_yaml_dict({"type": "zero_velocity", "num_dof": 6})
        assert isinstance(config, ZeroVelocityControllerConfig)
        assert config.num_dof == 6

    def test_config_from_yaml_dict_passthrough(self) -> None:
        config = KyberControllerManager.config_from_yaml_dict({"type": "passthrough", "num_dof": 8})
        assert isinstance(config, PassthroughControllerConfig)
        assert config.num_dof == 8

    def test_config_from_yaml_dict_rejects_missing_type(self) -> None:
        with pytest.raises(AegisConfigError):
            KyberControllerManager.config_from_yaml_dict({"num_dof": 6})

    def test_config_from_yaml_dict_rejects_unknown_type(self) -> None:
        with pytest.raises(AegisConfigError):
            KyberControllerManager.config_from_yaml_dict({"type": "made_up", "num_dof": 6})

    def test_config_from_yaml_dict_rejects_unknown_keys(self) -> None:
        with pytest.raises(AegisConfigError):
            KyberControllerManager.config_from_yaml_dict({"type": "zero_velocity", "garbage": 7})


class TestKyberConfigYaml:
    def test_round_trips_minimal_block(self) -> None:
        config = KyberConfig.from_yaml_dict({"controller_config": {"type": "zero_velocity", "num_dof": 6}})
        assert isinstance(config.controller_config, ZeroVelocityControllerConfig)
        assert config.publish_frequency_hz == 500.0

    def test_round_trips_full_block(self) -> None:
        config = KyberConfig.from_yaml_dict(
            {
                "publish_frequency_hz": 250.0,
                "controller_config": {"type": "passthrough", "num_dof": 7},
            }
        )
        assert config.publish_frequency_hz == 250.0
        assert isinstance(config.controller_config, PassthroughControllerConfig)
        assert config.controller_config.num_dof == 7

    def test_rejects_missing_controller_config(self) -> None:
        with pytest.raises(AegisConfigError):
            KyberConfig.from_yaml_dict({"publish_frequency_hz": 500.0})


if __name__ == "__main__":
    run_manor_tests()
