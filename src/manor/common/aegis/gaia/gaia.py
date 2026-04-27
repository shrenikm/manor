"""
Gaia: a Python object that wraps a self-contained Drake simulation.

The aegis simulation backends close over an instance of this class.
Internally Gaia owns a ``MultibodyPlant``, a ``SceneGraph``, zero or
more ``RgbdSensor`` instances, an optional Meshcat visualiser, and an
internal ``Simulator`` that advances the world up to the time
requested by the outer aegis diagram.

Gaia is intentionally NOT a Drake LeafSystem. The aegis diagram graph
shape is identical in sim and on hardware -- the only mode-specific
moving parts are the backends (which call into Gaia) and a sim-only
``GaiaAdvancer`` LeafSystem (declared elsewhere) that keeps Gaia's
clock in step with the aegis diagram's clock.

Lifecycle: a Gaia instance is constructed cheaply (just the public
config). All Drake plumbing -- parsing the description, welding the
manipulator, building the scene-graph diagram, allocating contexts --
runs inside ``finalize()`` and must be called explicitly before any
read / advance / render call. Calling read or advance methods before
``finalize()`` raises ``GaiaError``.
"""

from __future__ import annotations

from typing import Any, Self

import attr
import numpy as np
from pydrake.geometry import Meshcat, MeshcatParams, MeshcatVisualizer, SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import BasicVector, Context, Diagram, DiagramBuilder, LeafSystem

from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.custom_types import JointPositionsVector
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import GaiaError
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel

# Plant time step. ``0.0`` selects continuous-time integration; any
# positive value selects Drake's discrete-time solver. The bundled
# YAML uses ``0.001`` (1 ms) because discrete is roughly an order of
# magnitude cheaper on a 6-DOF arm under gravity, which is what lets
# the outer aegis simulator actually hold real-time pace. The default
# left here matches the legacy continuous-time mode for back-compat
# with tests / dev scripts that construct ``GaiaConfig()`` directly.
_DEFAULT_PLANT_TIME_STEP_S = 0.0

# Default placeholder camera resolution; real cameras will override
# these via configuration when the sensor wiring lands.
_DEFAULT_RGB_HEIGHT = 480
_DEFAULT_RGB_WIDTH = 640
_DEFAULT_DEPTH_HEIGHT = 480
_DEFAULT_DEPTH_WIDTH = 640

# Pin Meshcat to port 7000 so the URL doesn't drift to 7001/2/3 when
# a previous gylos's listening socket hasn't been fully released yet.
# If 7000 really is in use (e.g. a leftover gylos process), Drake's
# Meshcat will raise loudly -- preferable to silently drifting ports.
_MESHCAT_PORT = 7000

# Continuous-time integrator tuning for Gaia's inner Simulator. The
# default RK3 adaptive integrator shrinks its step aggressively when
# the unactuated plant accelerates under gravity, which is what makes
# the outer aegis simulator fail to hold real-time pace at the
# configured publish frequencies. Capping the step size and loosening
# the target accuracy is fine for a visual-only sim (no contact
# physics, no controller in the loop yet) and is cheap to revert
# once a tracking controller lands.
_INTEGRATOR_MAX_STEP_SIZE_S = 0.01
_INTEGRATOR_TARGET_ACCURACY = 1e-2


@attr.frozen
class GaiaConfig:
    """
    Knobs for constructing a ``Gaia``.

    ``time_step`` is the discrete time step of the underlying
    ``MultibodyPlant`` -- 0.0 selects continuous-time integration.

    ``enable_meshcat`` toggles the Meshcat visualiser. Tests leave
    this off; interactive runs flip it on so the browser session
    streams the plant geometry.

    ``target_realtime_rate`` scales how fast the **outer** aegis
    Simulator (the one that drives every periodic publisher) advances
    relative to wall-clock. ``0.0`` means "as fast as possible"
    (tests, headless batch runs); ``1.0`` means "real time"
    (interactive runs against Meshcat). Gaia's internal Simulator
    is always run as-fast-as-possible; rate-pacing happens once at
    the outer aegis simulator level so we don't end up with two
    simulators both sleeping against wall-clock and stepping on each
    other.
    """

    time_step: float = _DEFAULT_PLANT_TIME_STEP_S
    enable_meshcat: bool = False
    target_realtime_rate: float = 0.0
    rgb_height: int = _DEFAULT_RGB_HEIGHT
    rgb_width: int = _DEFAULT_RGB_WIDTH
    depth_height: int = _DEFAULT_DEPTH_HEIGHT
    depth_width: int = _DEFAULT_DEPTH_WIDTH

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the ``gaia_config:`` block of an aegis YAML.
        """
        return cls(**parse_attrs_yaml(cls, d, "gaia_config"))


class _DesiredStateSource(LeafSystem):
    """
    Build the desired-state vector for Gaia's
    ``InverseDynamicsController`` from the latest stashed command on
    the Gaia instance plus the plant's current measured state.

    The mapping mirrors the deprecated ``Lite6PliantMultiplexer`` so
    sim behavior matches the real robot's:

    * Velocity command:
      ``desired_q = measured_q`` (position error always zero) and
      ``desired_v[arm] = command_v``. A zero-velocity command therefore
      drives the controller to fight only motion, holding the arm in
      place. Matches "real robot stays still given zero velocities".
    * Position command:
      ``desired_q[arm] = command_q`` and ``desired_v = 0``. The PID
      term in the controller pulls the arm toward the target.
    * No command yet:
      ``desired_q = measured_q`` and ``desired_v = 0``. Holds the URDF
      default pose at startup until the first command arrives.

    Gripper joints (anything past the arm DOFs) always stay at
    ``desired_q[gripper] = measured_q[gripper]`` and
    ``desired_v[gripper] = 0`` -- gripper is commanded via the EEF
    channel, not joint commands, and that path isn't routed through
    Gaia yet. So the gripper is held at whatever position the URDF
    initialised it to.

    Reads ``gaia.latest_position_command`` / ``latest_velocity_command``
    directly. Drake calls ``_compute`` from the inner simulator's
    advance, which is single-threaded against the outer aegis loop
    where ``apply_*_command`` is invoked, so the Python-level reads
    are safe.
    """

    def __init__(self, gaia: Gaia, num_positions: int) -> None:
        super().__init__()
        self._gaia = gaia
        self._num_positions = num_positions
        self._estimated_state_input = self.DeclareVectorInputPort(
            "estimated_state",
            2 * num_positions,
        )
        self.DeclareVectorOutputPort(
            "desired_state",
            2 * num_positions,
            self._compute,
        )

    def _compute(self, context: Context, output: BasicVector) -> None:
        estimated = self._estimated_state_input.Eval(context)
        measured_q = np.asarray(estimated[: self._num_positions], dtype=np.float64)

        # Default: hold measured pose with zero desired velocity. This
        # is what runs at startup and any time both stashed commands
        # are None.
        desired_q = measured_q.copy()
        desired_v = np.zeros(self._num_positions, dtype=np.float64)

        velocity_cmd = self._gaia.latest_velocity_command
        position_cmd = self._gaia.latest_position_command
        if velocity_cmd is not None:
            arm_dof = velocity_cmd.velocities.shape[0]
            desired_v[:arm_dof] = velocity_cmd.velocities
        elif position_cmd is not None:
            arm_dof = position_cmd.positions.shape[0]
            desired_q[:arm_dof] = position_cmd.positions

        output.SetFromVector(np.concatenate([desired_q, desired_v]))


@attr.define
class Gaia:
    """
    Drake-backed simulator owning its own plant, scene graph, sensors,
    and ``Simulator``. Constructed with just the config; the plant /
    scene-graph diagram + ``Simulator`` are built by ``finalize()`` and
    must exist before any read or advance call.
    """

    manipulator_model: IManipulatorModel
    environment_config: EnvironmentConfig = attr.field(factory=EnvironmentConfig.default)
    config: GaiaConfig = attr.field(factory=GaiaConfig)

    plant: MultibodyPlant | None = attr.field(default=None, init=False)
    scene_graph: SceneGraph | None = attr.field(default=None, init=False)
    diagram: Diagram | None = attr.field(default=None, init=False)
    simulator: Simulator | None = attr.field(default=None, init=False)
    meshcat: Meshcat | None = attr.field(default=None, init=False)
    latest_position_command: JointPositions | None = attr.field(default=None, init=False)
    latest_velocity_command: JointVelocities | None = attr.field(default=None, init=False)
    _plant_context: Any = attr.field(default=None, init=False)
    _rgb_template: RGBImageData | None = attr.field(default=None, init=False)
    _depth_template: DepthImageData | None = attr.field(default=None, init=False)
    _finalized: bool = attr.field(default=False, init=False)

    def finalize(self) -> None:
        """
        Build the internal Drake diagram, allocate contexts, and ready
        ``Gaia`` for ``read_*`` / ``advance_to`` calls. Must be called
        exactly once before any read / advance / render call.
        """
        if self._finalized:
            raise GaiaError("Gaia.finalize() has already been called for this instance")

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=self.config.time_step)

        parser = Parser(plant)
        add_robot_models_to_package_map(parser.package_map())
        manipulator_model_index = parser.AddModels(self.manipulator_model.get_description_filepath())[0]

        # Weld the manipulator's base frame to the world at the configured pose.
        base_frame = plant.GetFrameByName(
            self.manipulator_model.get_base_frame_name(),
            manipulator_model_index,
        )
        plant.WeldFrames(
            plant.world_frame(),
            base_frame,
            RigidTransform(
                RollPitchYaw(self.environment_config.manipulator_base_rpy),
                self.environment_config.manipulator_base_xyz,
            ),
        )

        # Add and (optionally) weld every static model in the env.
        # ``weld_to_world=False`` is the right choice for URDFs that
        # already pin themselves to the world via an internal fixed
        # joint; attempting a second weld would conflict with the one
        # declared in the URDF.
        for static_model in self.environment_config.extra_models:
            static_index = parser.AddModels(static_model.description_filepath)[0]
            if not static_model.weld_to_world:
                continue
            body_indices = plant.GetBodyIndices(static_index)
            if not body_indices:
                raise GaiaError(f"Static model {static_model.name!r} has no bodies after parsing")
            base_body = plant.get_body(body_indices[0])
            plant.WeldFrames(
                plant.world_frame(),
                base_body.body_frame(),
                RigidTransform(
                    RollPitchYaw(static_model.base_rpy),
                    static_model.base_xyz,
                ),
            )

        plant.Finalize()

        # Wire an InverseDynamicsController on the manipulator's
        # actuation port so the plant can hold its pose under gravity.
        # Without this every actuated joint (arm + gripper fingers)
        # runs unactuated, the arm sags and the lightweight gripper
        # finger links accelerate without bound (visibly "fly off"
        # in meshcat). Controller plant is a separate manipulator-only
        # MultibodyPlant -- the InverseDynamicsController needs a
        # plant whose actuated DOFs match the controller's input.
        controller_plant = MultibodyPlant(time_step=self.config.time_step)
        controller_parser = Parser(controller_plant)
        add_robot_models_to_package_map(controller_parser.package_map())
        controller_parser.AddModels(self.manipulator_model.get_description_filepath())
        controller_plant.WeldFrames(
            controller_plant.world_frame(),
            controller_plant.GetFrameByName(self.manipulator_model.get_base_frame_name()),
            RigidTransform(),
        )
        controller_plant.Finalize()

        gains = self.manipulator_model.get_default_sim_pid_gains()
        id_controller = builder.AddSystem(
            InverseDynamicsController(
                controller_plant,
                kp=gains.kp,
                ki=gains.ki,
                kd=gains.kd,
                has_reference_acceleration=False,
            )
        )

        # Desired-state source builds (q_des, v_des) from the latest
        # stashed command on this Gaia instance plus the plant's
        # measured state. See ``_DesiredStateSource`` for the mapping
        # rules; the short version is "velocity-control mode by
        # default, falls through to position-control if a position
        # command is stashed, holds measured pose otherwise".
        num_positions = controller_plant.num_positions()
        desired_source = builder.AddSystem(_DesiredStateSource(gaia=self, num_positions=num_positions))
        manipulator_state_port = plant.get_state_output_port(manipulator_model_index)
        builder.Connect(
            manipulator_state_port,
            id_controller.get_input_port_estimated_state(),
        )
        builder.Connect(
            manipulator_state_port,
            desired_source.GetInputPort("estimated_state"),
        )
        builder.Connect(
            desired_source.GetOutputPort("desired_state"),
            id_controller.get_input_port_desired_state(),
        )
        builder.Connect(
            id_controller.get_output_port_control(),
            plant.get_actuation_input_port(manipulator_model_index),
        )

        meshcat: Meshcat | None = None
        if self.config.enable_meshcat:
            # Spawn the Meshcat http/websocket server (printed URL is the
            # one the user opens) and wire its visualizer to the scene
            # graph so the live geometry streams to the browser.
            meshcat = Meshcat(MeshcatParams(port=_MESHCAT_PORT))
            MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

        diagram = builder.Build()
        simulator = Simulator(diagram)
        # Gaia's inner simulator is driven by GaiaAdvancer from the
        # outer aegis simulator's clock. Real-time pacing is the outer
        # simulator's job; if we throttle here too, both simulators
        # sleep against wall-clock and the periodic publishers
        # downstream see wide rate variance.
        simulator.set_target_realtime_rate(0.0)
        # Continuous-time integrator: cap the max step and loosen
        # target accuracy so the outer simulator can hold real-time
        # pace. See module-level constants for context.
        if self.config.time_step == 0.0:
            integrator = simulator.get_mutable_integrator()
            integrator.set_maximum_step_size(_INTEGRATOR_MAX_STEP_SIZE_S)
            integrator.set_target_accuracy(_INTEGRATOR_TARGET_ACCURACY)
        simulator.Initialize()

        plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())

        self.plant = plant
        self.scene_graph = scene_graph
        self.diagram = diagram
        self.simulator = simulator
        self.meshcat = meshcat
        self._plant_context = plant_context
        self._rgb_template = RGBImageData.construct_default(
            height=self.config.rgb_height,
            width=self.config.rgb_width,
        )
        self._depth_template = DepthImageData.construct_default(
            height=self.config.depth_height,
            width=self.config.depth_width,
        )
        self._finalized = True

    def advance_to(self, time_s: float) -> None:
        """
        Advance the internal Simulator to ``time_s`` (in simulated seconds).
        """
        self._require_finalized()
        if time_s < self.simulator.get_context().get_time():
            return
        self.simulator.AdvanceTo(time_s)

    def apply_joint_position_command(self, joint_positions: JointPositions) -> None:
        """
        Stash the latest joint-position command. Wiring this into the
        plant's actuation input is deferred to a follow-up that hooks
        in a per-DOF tracking controller.
        """
        self._require_finalized()
        self.latest_position_command = joint_positions

    def apply_joint_velocity_command(self, joint_velocities: JointVelocities) -> None:
        """
        Stash the latest joint-velocity command. Same caveat as
        ``apply_joint_position_command``.
        """
        self._require_finalized()
        self.latest_velocity_command = joint_velocities

    def read_joint_state(self) -> JointState:
        """
        Read positions + velocities directly from the live plant context.
        """
        self._require_finalized()
        positions = np.asarray(self.plant.GetPositions(self._plant_context), dtype=np.float64).copy()
        velocities = np.asarray(self.plant.GetVelocities(self._plant_context), dtype=np.float64).copy()
        header = TimestampHeader.from_system_time()
        return JointState(
            header=header,
            joint_positions=JointPositions(header=header, positions=positions),
            joint_velocities=JointVelocities(header=header, velocities=velocities),
        )

    def set_joint_positions(self, positions: JointPositionsVector) -> None:
        """
        Snap the plant's joint positions to ``positions``. Used by the
        future "both" mirror mode where the real robot's measured
        positions drive the sim visualisation.
        """
        self._require_finalized()
        positions_arr = np.asarray(positions, dtype=np.float64)
        if positions_arr.shape != (self.plant.num_positions(),):
            raise GaiaError(
                f"set_joint_positions expected shape ({self.plant.num_positions()},); got {positions_arr.shape}"
            )
        self.plant.SetPositions(self._plant_context, positions_arr)

    def render_rgb(self, camera_id: str = "default") -> RGBImageData:
        """
        Render the RGB view from the named camera. Currently a structural
        stub returning an empty frame with a fresh timestamp.
        """
        self._require_finalized()
        del camera_id
        return attr.evolve(self._rgb_template, header=TimestampHeader.from_system_time())

    def render_depth(self, camera_id: str = "default") -> DepthImageData:
        """
        Render the depth view from the named camera. Currently a
        structural stub returning an empty frame with a fresh timestamp.
        """
        self._require_finalized()
        del camera_id
        return attr.evolve(self._depth_template, header=TimestampHeader.from_system_time())

    def is_finalized(self) -> bool:
        return self._finalized

    def shutdown(self) -> None:
        """
        Drop references to all heavy Drake resources (meshcat server,
        inner simulator, diagram) so their C++ destructors run
        synchronously rather than at interpreter shutdown. The meshcat
        destructor closes the listening socket; running it now means
        the next gylos launch can re-bind port 7000 immediately
        instead of drifting to 7001 / 7002 / ...

        Safe to call zero or multiple times. After ``shutdown`` the
        Gaia instance must not be used (read / advance / render).
        """
        self.meshcat = None
        self.simulator = None
        self.diagram = None
        self.scene_graph = None
        self.plant = None
        self._plant_context = None
        self._finalized = False

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise GaiaError("Gaia.finalize() must be called before Gaia can be used")
