"""
IKPassthroughController: bridges Cartesian arm commands down to joint
commands while passing joint and EE commands through unchanged.

Behaviour by Action shape:

* JointCommand: forwarded as-is on the joint side of the JointEECommand.
* CartesianCommand:
    - cartesian_pose: Drake InverseKinematics solves for an arm-side q
      that places the FK / IK frame at the requested pose. The plant's
      EE block (whatever non-arm joints the URDF encodes for the
      end-effector -- parallel-gripper fingers, dexterous-hand finger
      joints, etc.) is pinned to the proprioception's measured values
      via lock constraints so IK only varies the arm DOFs. Solution is
      wrapped as a JointPositions command sized to num_arm_dof.
    - cartesian_twist: Drake DoDifferentialInverseKinematics maps the
      requested spatial velocity to a v vector. EE-block velocity
      limits are pinned to (0, 0) so diff IK only allocates motion to
      the arm DOFs. Solution is wrapped as a JointVelocities command
      sized to num_arm_dof.
* JointTrajectoryCommand / CartesianTrajectoryCommand: not handled in a
  passthrough controller, fall back to a zero-velocity arm command.
* EECommand: forwarded as-is on the EE side of the JointEECommand.
* EETrajectoryCommand: not handled, dropped.

The controller owns its own Drake MultibodyPlant built from the
manipulator description; this is an IK-only plant, independent of the
sim plant in Gaia and the FK plant in Talos. The arm / EE split is
determined by IManipulatorModel.get_num_dof() so the controller works
uniformly across robots and end-effector types -- no gripper-specific
assumptions live here.

Frame convention: every Cartesian target -- pose translation /
orientation, twist linear / angular -- is interpreted in the
**manipulator's own base frame**, not the sim world frame. The IK
plant pulls the base frame name from
IManipulatorModel.get_base_frame_name() and uses it directly as the
reference frame for AddPositionConstraint / AddOrientationConstraint
and as the (frame_A, frame_E) pair for CalcJacobianSpatialVelocity.
That means a YAML translation of [0.25, 0.0, 0.15] is "0.25 m
forward of the base, 0.15 m above the base," regardless of where
the manipulator is mounted in the sim world.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np
from pydrake.math import RotationMatrix
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsParameters,
    DifferentialInverseKinematicsStatus,
    DoDifferentialInverseKinematics,
    InverseKinematics,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import Frame, JacobianWrtVariable
from pydrake.solvers import Solve

from manor.common.aegis.kyber.controllers.controller_manager import (
    KyberControllerConfigBase,
    KyberControllerType,
)
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.joint_command import JointCommand
from manor.common.definitions.joint_ee_command import JointEECommand
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.proprioception import Proprioception
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.logging_utils import ManorLogger
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel

# Default time step for the differential IK QP. Diff IK is
# rate-controlled, so this is the dt used to convert v_next into the
# step's joint position increment for the joint-limit projection
# (q_current + N v_next dt). The aegis Kyber tick rate (typically
# 200 Hz) is the right ballpark; 5 ms is a safe default and is
# overridable via YAML.
_DEFAULT_DIFF_IK_TIME_STEP_S: float = 5e-3

# Joint position slack used when locking the EE block during the
# pose IK solve. For each non-arm DOF we constrain q to
# [measured - tol, measured + tol]; the slack only needs to absorb
# numerical noise, since IK is not allowed to move the EE here. Diff
# IK takes a stricter line and pins v[ee_block] to (0, 0) (see
# _build_diff_ik_params) so spatial-velocity tracking is allocated
# entirely to the arm DOFs.
_EE_BLOCK_POSITION_LOCK_TOL: float = 1e-4
_DEFAULT_ORIENTATION_THETA_BOUND_RAD: float = 1e-3

# Throttle keys for ManorLogger.warning_throttled. Each key buckets one class of warning so a
# sustained problem fires at most once per second per controller instance instead of flooding the
# terminal at the 500 Hz Kyber tick rate. Without these the IK-failure / unsupported-action paths
# look identical to "policy is doing nothing" and the operator can't tell the arm is wedged.
_IK_FAILURE_THROTTLE_KEY: str = "ik_failure"
_TRAJECTORY_FALLBACK_THROTTLE_KEY: str = "trajectory_fallback"


@attr.frozen
class IKPassthroughControllerConfig(KyberControllerConfigBase):
    """
    Config for IKPassthroughController. The controller derives plant
    geometry and frame names from manipulator_model; the YAML body only
    needs to override numerical knobs (diff IK time step,
    orientation tolerance for the IK pose solve) when the defaults
    aren't right for the task.
    """

    CONTROLLER_TYPE: ClassVar[KyberControllerType] = KyberControllerType.IK_PASSTHROUGH

    diff_ik_time_step_s: float = _DEFAULT_DIFF_IK_TIME_STEP_S
    orientation_theta_bound_rad: float = _DEFAULT_ORIENTATION_THETA_BOUND_RAD

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "IKPassthroughControllerConfig"))


def _build_ik_plant(manipulator_model: IManipulatorModel) -> MultibodyPlant:
    plant = MultibodyPlant(time_step=0.0)
    parser = Parser(plant)
    add_robot_models_to_package_map(parser.package_map())
    model_index = parser.AddModels(manipulator_model.get_description_filepath())[0]
    base_frame = plant.GetFrameByName(manipulator_model.get_base_frame_name(), model_index)
    plant.WeldFrames(plant.world_frame(), base_frame)
    plant.Finalize()
    return plant


@attr.define
class IKPassthroughController:
    """
    Cartesian-to-joint IK controller. Joint-shaped Action commands fall
    through unchanged; Cartesian-shaped Action commands go through Drake's
    InverseKinematics (pose) or DoDifferentialInverseKinematics (twist).
    """

    manipulator_model: IManipulatorModel
    config: IKPassthroughControllerConfig
    plant: MultibodyPlant
    _plant_context: object = attr.field(default=None)
    _diff_ik_params: DifferentialInverseKinematicsParameters = attr.field(default=None)
    _base_frame: Frame | None = attr.field(default=None)
    _tip_frame: Frame | None = attr.field(default=None)
    _logger: ManorLogger = attr.field(init=False)

    @_logger.default
    def _initialize_logger(self) -> ManorLogger:
        # Logger name is the class name so subclasses (if any are
        # ever introduced) get their own logger automatically without
        # having to override anything.
        return ManorLogger(self.__class__.__name__)

    @classmethod
    def build(
        cls,
        config: IKPassthroughControllerConfig,
        manipulator_model: IManipulatorModel,
    ) -> Self:
        plant = _build_ik_plant(manipulator_model)
        controller = cls(manipulator_model=manipulator_model, config=config, plant=plant)
        controller._plant_context = plant.CreateDefaultContext()
        controller._diff_ik_params = controller._build_diff_ik_params()
        # Cache the base + tip frames so every solve doesn't re-look
        # them up by name. The base frame is the explicit reference
        # for every Cartesian target the controller accepts (pose
        # translation, pose orientation, twist linear / angular).
        controller._base_frame = plant.GetFrameByName(manipulator_model.get_base_frame_name())
        controller._tip_frame = plant.GetFrameByName(manipulator_model.get_fk_ik_frame_name())
        return controller

    def _build_diff_ik_params(self) -> DifferentialInverseKinematicsParameters:
        # Initialise with plant-derived position / velocity counts;
        # joint position / velocity limits come from the URDF via the
        # plant. Pin the EE-block velocity limits to (0, 0) so diff IK
        # never allocates spatial-velocity tracking to non-arm joints.
        params = DifferentialInverseKinematicsParameters(
            num_positions=self.plant.num_positions(),
            num_velocities=self.plant.num_velocities(),
        )
        params.set_time_step(self.config.diff_ik_time_step_s)

        v_lower = np.asarray(self.plant.GetVelocityLowerLimits(), dtype=np.float64).copy()
        v_upper = np.asarray(self.plant.GetVelocityUpperLimits(), dtype=np.float64).copy()
        # Drake reports +/- inf on unlimited joints. Diff IK requires
        # finite limits to stay well-posed; clip to a wide-but-finite
        # band on the arm DOFs and zero on the EE block.
        num_arm_dof = self.manipulator_model.get_num_dof()
        wide_arm_limit = 10.0  # rad/s -- much higher than realistic, just to keep the QP bounded.
        for i in range(self.plant.num_velocities()):
            if i < num_arm_dof:
                if not np.isfinite(v_lower[i]):
                    v_lower[i] = -wide_arm_limit
                if not np.isfinite(v_upper[i]):
                    v_upper[i] = wide_arm_limit
            else:
                v_lower[i] = 0.0
                v_upper[i] = 0.0
        params.set_joint_velocity_limits((v_lower, v_upper))

        q_lower = np.asarray(self.plant.GetPositionLowerLimits(), dtype=np.float64).copy()
        q_upper = np.asarray(self.plant.GetPositionUpperLimits(), dtype=np.float64).copy()
        # Same finite-band fix-up for joint position limits where the
        # URDF leaves them unbounded.
        wide_q_limit = 10.0
        for i in range(self.plant.num_positions()):
            if not np.isfinite(q_lower[i]):
                q_lower[i] = -wide_q_limit
            if not np.isfinite(q_upper[i]):
                q_upper[i] = wide_q_limit
        params.set_joint_position_limits((q_lower, q_upper))
        return params

    def step(self, action: Action, proprioception: Proprioception) -> JointEECommand:
        header = TimestampHeader.from_system_time()
        num_arm_dof = self.manipulator_model.get_num_dof()

        if action.joint_command is not None:
            joint_command = action.joint_command
        elif action.cartesian_command is not None:
            joint_command = self._cartesian_to_joint(
                cartesian_command=action.cartesian_command,
                proprioception=proprioception,
                header=header,
            )
        else:
            # JointTrajectoryCommand / CartesianTrajectoryCommand / EETrajectoryCommand: not handled
            # in a passthrough controller -- fall back to a zero-velocity arm command rather than
            # crashing the kyber tick. Trajectory-aware controllers should subclass / replace this.
            self._log_trajectory_fallback(action)
            joint_command = JointCommand(
                header=header,
                joint_velocities=JointVelocities(header=header, velocities=np.zeros(num_arm_dof, dtype=np.float64)),
            )

        return JointEECommand(header=header, joint_command=joint_command, ee_command=action.ee_command)

    def _cartesian_to_joint(
        self,
        cartesian_command: CartesianCommand,
        proprioception: Proprioception,
        header: TimestampHeader,
    ) -> JointCommand:
        if cartesian_command.cartesian_pose is not None:
            joint_positions_arm = self._solve_pose_ik(
                target_translation=cartesian_command.cartesian_pose.translation,
                target_orientation_wxyz=cartesian_command.cartesian_pose.orientation,
                proprioception=proprioception,
            )
            return JointCommand(
                header=header,
                joint_positions=JointPositions(header=header, positions=joint_positions_arm),
            )
        # cartesian_twist
        joint_velocities_arm = self._solve_twist_diff_ik(
            target_linear=cartesian_command.cartesian_twist.linear,
            target_angular=cartesian_command.cartesian_twist.angular,
            proprioception=proprioception,
        )
        return JointCommand(
            header=header,
            joint_velocities=JointVelocities(header=header, velocities=joint_velocities_arm),
        )

    def _log_ik_failure(self, message: str) -> None:
        """
        Rate-limited warning when an IK / diff-IK solve fails. Both solver fall-back paths look
        identical to "policy is doing nothing" without a log -- the arm just freezes silently. The
        ManorLogger throttle keeps this to one line per second per logger so a sustained failure
        doesn't drown the log at the 500 Hz Kyber tick rate.
        """
        self._logger.warning_throttled(message, key=_IK_FAILURE_THROTTLE_KEY)

    def _log_trajectory_fallback(self, action: Action) -> None:
        """
        Rate-limited warning when an unsupported (trajectory-shaped) Action arrives and step() falls
        back to a zero-velocity arm command. Same motivation as _log_ik_failure -- silent fallback
        looks like a dead policy. Names the offending shape so the operator knows which side of the
        policy/controller contract is wrong.
        """
        if action.joint_trajectory_command is not None:
            shape = "joint_trajectory_command"
        elif action.cartesian_trajectory_command is not None:
            shape = "cartesian_trajectory_command"
        elif action.ee_trajectory_command is not None:
            shape = "ee_trajectory_command"
        else:
            shape = "unknown (no command field set)"
        self._logger.warning_throttled(
            f"IKPassthroughController received unsupported action shape ({shape}); "
            f"falling back to zero-velocity arm command. Use a trajectory-aware controller to handle this.",
            key=_TRAJECTORY_FALLBACK_THROTTLE_KEY,
        )

    def _populate_plant_context(self, proprioception: Proprioception) -> None:
        positions = proprioception.joint_state.joint_positions.positions
        velocities = proprioception.joint_state.joint_velocities.velocities
        # Proprioception's joint_state is sized to plant num_positions /
        # num_velocities at steady state; tolerate the early-bring-up
        # case where it's still sized to 0 by leaving the context at its
        # default zero-q.
        if positions.shape[0] == self.plant.num_positions():
            self.plant.SetPositions(self._plant_context, positions)
        if velocities.shape[0] == self.plant.num_velocities():
            self.plant.SetVelocities(self._plant_context, velocities)

    def _solve_pose_ik(
        self,
        target_translation: np.ndarray,
        target_orientation_wxyz: np.ndarray,
        proprioception: Proprioception,
    ) -> np.ndarray:
        self._populate_plant_context(proprioception)
        num_arm_dof = self.manipulator_model.get_num_dof()
        q_init = np.asarray(self.plant.GetPositions(self._plant_context), dtype=np.float64)

        ik = InverseKinematics(self.plant, with_joint_limits=True)

        # Cartesian targets are explicitly in the manipulator base
        # frame (see module docstring). Using the named base frame
        # rather than world_frame makes the contract obvious in the
        # constraint set and is robust against ever changing how the
        # IK plant welds the manipulator into its world.
        target = np.asarray(target_translation, dtype=np.float64)
        ik.AddPositionConstraint(
            frameB=self._tip_frame,
            p_BQ=np.zeros(3),
            frameA=self._base_frame,
            p_AQ_lower=target,
            p_AQ_upper=target,
        )
        # Drake's RotationMatrix takes a Quaternion(w, x, y, z) -- our
        # CartesianPose stores orientation in the same wxyz order.
        from pydrake.common.eigen_geometry import Quaternion

        target_quat = Quaternion(
            float(target_orientation_wxyz[0]),
            float(target_orientation_wxyz[1]),
            float(target_orientation_wxyz[2]),
            float(target_orientation_wxyz[3]),
        )
        target_rotation = RotationMatrix(target_quat)
        ik.AddOrientationConstraint(
            frameAbar=self._base_frame,
            R_AbarA=target_rotation,
            frameBbar=self._tip_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=self.config.orientation_theta_bound_rad,
        )

        # Pin the EE block of q to its measured value via tight box
        # bounds so IK only varies the arm DOFs. Works generically: any
        # plant DOF past num_arm_dof is treated as part of the EE,
        # whatever the URDF encodes there.
        if self.plant.num_positions() > num_arm_dof:
            q_var = ik.q()
            for i in range(num_arm_dof, self.plant.num_positions()):
                ik.get_mutable_prog().AddBoundingBoxConstraint(
                    q_init[i] - _EE_BLOCK_POSITION_LOCK_TOL,
                    q_init[i] + _EE_BLOCK_POSITION_LOCK_TOL,
                    q_var[i],
                )

        ik.get_mutable_prog().SetInitialGuess(ik.q(), q_init)
        result = Solve(ik.prog())
        if not result.is_success():
            # Degrade gracefully: hold current arm q. The diagram keeps
            # ticking; the operator sees the arm freeze rather than
            # crash. Note the target is in the manipulator base frame
            # (the IK plant welds the base to the world origin) -- the
            # most common cause of failure here is an out-of-reach
            # request expressed in the wrong frame.
            translation_str = np.array2string(np.asarray(target_translation, dtype=np.float64), precision=4)
            orientation_str = np.array2string(np.asarray(target_orientation_wxyz, dtype=np.float64), precision=4)
            self._log_ik_failure(
                f"pose IK infeasible: target translation={translation_str}, "
                f"orientation_wxyz={orientation_str} (base frame); holding current arm q"
            )
            return q_init[:num_arm_dof].copy()
        q_solution = np.asarray(result.GetSolution(ik.q()), dtype=np.float64)
        return q_solution[:num_arm_dof].copy()

    def _solve_twist_diff_ik(
        self,
        target_linear: np.ndarray,
        target_angular: np.ndarray,
        proprioception: Proprioception,
    ) -> np.ndarray:
        self._populate_plant_context(proprioception)
        num_arm_dof = self.manipulator_model.get_num_dof()

        q_current = np.asarray(self.plant.GetPositions(self._plant_context), dtype=np.float64)
        v_current = np.asarray(self.plant.GetVelocities(self._plant_context), dtype=np.float64)

        # Diff IK takes a 6-vector V = [angular; linear] paired with a
        # spatial Jacobian; the V and the Jacobian must agree on which
        # frame they're expressed in. The controller's contract is
        # base-frame Cartesian targets (see module docstring), so the
        # Jacobian uses the manipulator base as both the
        # "with-respect-to" and the "expressed-in" frame. For a base
        # whose orientation matches the world (e.g. lite6 with default
        # base_rpy = [0, 0, 0]) this is numerically identical to the
        # world-frame version, but the contract is now explicit and
        # robust to a non-trivial mounting orientation later.
        V = np.concatenate([np.asarray(target_angular, dtype=np.float64), np.asarray(target_linear, dtype=np.float64)])
        jacobian = self.plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kV,
            self._tip_frame,
            np.zeros(3),
            self._base_frame,
            self._base_frame,
        )

        result = DoDifferentialInverseKinematics(q_current, v_current, V, jacobian, self._diff_ik_params)
        if result.status != DifferentialInverseKinematicsStatus.kSolutionFound:
            linear_str = np.array2string(np.asarray(target_linear, dtype=np.float64), precision=4)
            angular_str = np.array2string(np.asarray(target_angular, dtype=np.float64), precision=4)
            self._log_ik_failure(
                f"diff IK infeasible: target linear={linear_str}, angular={angular_str} "
                f"(base frame), status={result.status}; falling back to zero arm velocity"
            )
            return np.zeros(num_arm_dof, dtype=np.float64)
        v_solution = np.asarray(result.joint_velocities, dtype=np.float64)
        return v_solution[:num_arm_dof].copy()
