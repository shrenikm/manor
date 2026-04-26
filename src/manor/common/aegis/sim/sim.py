"""
Sim: a Python object that wraps a self-contained Drake simulation.

The aegis simulation backends close over an instance of this class.
Inside Sim lives a ``MultibodyPlant`` (physics), a ``SceneGraph``
(geometry), zero or more ``RgbdSensor`` instances, an optional Meshcat
visualiser, and an internal ``Simulator`` that advances the world up to
the time requested by the outer aegis diagram.

Sim is intentionally NOT a Drake LeafSystem. The aegis diagram graph
shape is the same in sim and on hardware -- the only mode-specific
moving parts are the backends (which call into Sim) and a sim-only
``_SimAdvancer`` LeafSystem (declared elsewhere) that keeps Sim's clock
in step with the aegis diagram's clock.

The public surface is:

  * ``apply_joint_position_command`` / ``apply_joint_velocity_command``
    -- accept the latest controller command. The current implementation
    stores the command and exposes it back on reads; a future change
    will route it into the plant's actuation port.
  * ``read_joint_state`` -- returns the current plant state as a
    ``JointState``.
  * ``render_rgb`` / ``render_depth`` -- placeholder image renders. They
    return empty buffers until cameras are wired in.
  * ``advance_to(t)`` -- ticks the internal Simulator forward.
  * ``set_joint_positions`` -- forcibly snap the plant's joint positions.
    Used by the future "both" mirror mode (real robot driving the sim
    visualisation).
"""

from __future__ import annotations

import attr
import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram, DiagramBuilder

from manor.common.aegis.sim.env_config import EnvironmentConfig
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import SimError
from manor.common.model_utils import add_robot_models_to_package_map
from manor.manipulators.manipulator_model import IManipulatorModel

# Plant time step. 0.0 selects continuous-time integration, which is
# more forgiving for the current zero-actuation stub; the discrete
# solver tends to diverge on an unactuated plant under gravity. A
# follow-up that wires in a position-tracking controller can switch
# this back to a small discrete step (~1 ms) safely.
_DEFAULT_PLANT_TIME_STEP_S = 0.0

# Default placeholder camera resolution; real cameras will override
# these via configuration when the sensor wiring lands.
_DEFAULT_RGB_HEIGHT = 480
_DEFAULT_RGB_WIDTH = 640
_DEFAULT_DEPTH_HEIGHT = 480
_DEFAULT_DEPTH_WIDTH = 640


@attr.frozen
class SimConfig:
    """
    Knobs for constructing a ``Sim``.

    ``time_step`` is the discrete time step of the underlying
    ``MultibodyPlant`` -- 0.0 selects continuous-time integration.
    ``enable_meshcat`` toggles the Meshcat visualiser (off by default to
    keep tests headless).
    """

    time_step: float = _DEFAULT_PLANT_TIME_STEP_S
    enable_meshcat: bool = False
    rgb_height: int = _DEFAULT_RGB_HEIGHT
    rgb_width: int = _DEFAULT_RGB_WIDTH
    depth_height: int = _DEFAULT_DEPTH_HEIGHT
    depth_width: int = _DEFAULT_DEPTH_WIDTH


class Sim:
    """
    Drake-backed simulator owning its own plant, scene graph, sensors,
    and ``Simulator``. Instantiated once per aegis run and shared by the
    sim sensor + manipulator backends.
    """

    def __init__(
        self,
        manipulator_model: IManipulatorModel,
        environment_config: EnvironmentConfig | None = None,
        config: SimConfig | None = None,
    ) -> None:
        self._manipulator_model = manipulator_model
        self._environment_config = environment_config if environment_config is not None else EnvironmentConfig.default()
        self._config = config if config is not None else SimConfig()

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=self._config.time_step)
        self._plant: MultibodyPlant = plant
        self._scene_graph: SceneGraph = scene_graph

        parser = Parser(plant)
        add_robot_models_to_package_map(parser.package_map())
        manipulator_model_index = parser.AddModels(manipulator_model.get_description_filepath())[0]

        # Weld the manipulator's base frame to the world at the configured pose.
        base_frame = plant.GetFrameByName(manipulator_model.get_base_frame_name(), manipulator_model_index)
        plant.WeldFrames(
            plant.world_frame(),
            base_frame,
            RigidTransform(
                RollPitchYaw(self._environment_config.manipulator_base_rpy),
                self._environment_config.manipulator_base_xyz,
            ),
        )

        # Add and (optionally) weld every static model in the env.
        # ``weld_to_world=False`` is the right choice for URDFs that
        # already pin themselves to the world via an internal fixed
        # joint -- attempting a second weld would conflict with the
        # one declared in the URDF.
        for static_model in self._environment_config.extra_models:
            model_index = parser.AddModels(static_model.description_filepath)[0]
            if not static_model.weld_to_world:
                continue
            body_indices = plant.GetBodyIndices(model_index)
            if not body_indices:
                raise SimError(f"Static model {static_model.name!r} has no bodies after parsing")
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

        self._diagram: Diagram = builder.Build()
        self._simulator: Simulator = Simulator(self._diagram)
        self._simulator.set_target_realtime_rate(0.0)
        self._simulator.Initialize()

        self._diagram_context = self._simulator.get_mutable_context()
        self._plant_context = self._diagram.GetMutableSubsystemContext(self._plant, self._diagram_context)

        # The plant input port for actuated DOFs. We hold zero actuation
        # by default; concrete command application lands in a follow-up.
        self._plant.get_actuation_input_port().FixValue(self._plant_context, np.zeros(self._plant.num_actuators()))

        self._latest_position_command: np.ndarray | None = None
        self._latest_velocity_command: np.ndarray | None = None

        # Cached stub frames for render_rgb / render_depth until the
        # RgbdSensor wiring lands.
        self._rgb_template = RGBImageData.construct_default(
            height=self._config.rgb_height, width=self._config.rgb_width
        )
        self._depth_template = DepthImageData.construct_default(
            height=self._config.depth_height, width=self._config.depth_width
        )

    @property
    def manipulator_model(self) -> IManipulatorModel:
        return self._manipulator_model

    @property
    def environment_config(self) -> EnvironmentConfig:
        return self._environment_config

    @property
    def plant(self) -> MultibodyPlant:
        return self._plant

    @property
    def scene_graph(self) -> SceneGraph:
        return self._scene_graph

    @property
    def diagram(self) -> Diagram:
        return self._diagram

    @property
    def simulator(self) -> Simulator:
        return self._simulator

    def advance_to(self, time_s: float) -> None:
        """
        Advance the internal Simulator to ``time_s`` (in simulated seconds).
        """
        if time_s < self._simulator.get_context().get_time():
            return
        self._simulator.AdvanceTo(time_s)

    def apply_joint_position_command(self, positions: np.ndarray) -> None:
        """
        Stash the latest joint-position command. Wiring this into the
        plant's actuation input is deferred to a follow-up that hooks
        in a per-DOF tracking controller.
        """
        self._latest_position_command = np.asarray(positions, dtype=np.float64).copy()

    def apply_joint_velocity_command(self, velocities: np.ndarray) -> None:
        """
        Stash the latest joint-velocity command. Same caveat as
        ``apply_joint_position_command``.
        """
        self._latest_velocity_command = np.asarray(velocities, dtype=np.float64).copy()

    def read_joint_state(self) -> JointState:
        """
        Read positions + velocities directly from the live plant context.
        """
        positions = np.asarray(self._plant.GetPositions(self._plant_context), dtype=np.float64).copy()
        velocities = np.asarray(self._plant.GetVelocities(self._plant_context), dtype=np.float64).copy()
        header = TimestampHeader.from_system_time()
        return JointState(
            header=header,
            joint_positions=JointPositions(header=header, positions=positions),
            joint_velocities=JointVelocities(header=header, velocities=velocities),
        )

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """
        Snap the plant's joint positions to ``positions``. Used by the
        future "both" mirror mode where the real robot's measured
        positions drive the sim visualisation.
        """
        positions_arr = np.asarray(positions, dtype=np.float64)
        if positions_arr.shape != (self._plant.num_positions(),):
            raise SimError(
                f"set_joint_positions expected shape ({self._plant.num_positions()},); got {positions_arr.shape}"
            )
        self._plant.SetPositions(self._plant_context, positions_arr)

    def render_rgb(self, camera_id: str = "default") -> RGBImageData:
        """
        Render the RGB view from the named camera. Currently a structural
        stub returning an empty frame with a fresh timestamp.
        """
        del camera_id
        return attr.evolve(self._rgb_template, header=TimestampHeader.from_system_time())

    def render_depth(self, camera_id: str = "default") -> DepthImageData:
        """
        Render the depth view from the named camera. Currently a
        structural stub returning an empty frame with a fresh timestamp.
        """
        del camera_id
        return attr.evolve(self._depth_template, header=TimestampHeader.from_system_time())

    def get_latest_position_command(self) -> np.ndarray | None:
        return self._latest_position_command

    def get_latest_velocity_command(self) -> np.ndarray | None:
        return self._latest_velocity_command
