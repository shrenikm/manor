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

from enum import StrEnum
from typing import Any, Self

import attr
import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram, DiagramBuilder

from manor.common.aegis.gaia.env_config import EnvironmentConfig
from manor.common.custom_types import JointPositionsVector
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.joint_positions import JointPositions
from manor.common.definitions.joint_state import JointState
from manor.common.definitions.joint_velocities import JointVelocities
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.definitions.timestamp_header import TimestampHeader
from manor.common.exceptions import AegisConfigError, GaiaError
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


class GaiaYamlKey(StrEnum):
    """
    YAML field names for the ``gaia:`` block of an aegis config.
    """

    TIME_STEP = "time_step"
    ENABLE_MESHCAT = "enable_meshcat"
    RGB_HEIGHT = "rgb_height"
    RGB_WIDTH = "rgb_width"
    DEPTH_HEIGHT = "depth_height"
    DEPTH_WIDTH = "depth_width"


def _gaia_require_int(value: object, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise AegisConfigError(f"'{field_name}' must be an int; got {type(value).__name__}")
    return value


def _gaia_require_number(value: object, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AegisConfigError(f"'{field_name}' must be a number; got {type(value).__name__}")
    return float(value)


def _gaia_require_bool(value: object, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise AegisConfigError(f"'{field_name}' must be a bool; got {type(value).__name__}")
    return value


@attr.frozen
class GaiaConfig:
    """
    Knobs for constructing a ``Gaia``.

    ``time_step`` is the discrete time step of the underlying
    ``MultibodyPlant`` -- 0.0 selects continuous-time integration.
    ``enable_meshcat`` toggles the Meshcat visualiser (off by default
    to keep tests headless).
    """

    time_step: float = _DEFAULT_PLANT_TIME_STEP_S
    enable_meshcat: bool = False
    rgb_height: int = _DEFAULT_RGB_HEIGHT
    rgb_width: int = _DEFAULT_RGB_WIDTH
    depth_height: int = _DEFAULT_DEPTH_HEIGHT
    depth_width: int = _DEFAULT_DEPTH_WIDTH

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the ``gaia:`` block of an aegis YAML.
        """
        allowed = {key.value for key in GaiaYamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(f"gaia: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}")
        return cls(
            time_step=_gaia_require_number(
                d.get(GaiaYamlKey.TIME_STEP, _DEFAULT_PLANT_TIME_STEP_S),
                f"gaia.{GaiaYamlKey.TIME_STEP}",
            ),
            enable_meshcat=_gaia_require_bool(
                d.get(GaiaYamlKey.ENABLE_MESHCAT, False),
                f"gaia.{GaiaYamlKey.ENABLE_MESHCAT}",
            ),
            rgb_height=_gaia_require_int(
                d.get(GaiaYamlKey.RGB_HEIGHT, _DEFAULT_RGB_HEIGHT),
                f"gaia.{GaiaYamlKey.RGB_HEIGHT}",
            ),
            rgb_width=_gaia_require_int(
                d.get(GaiaYamlKey.RGB_WIDTH, _DEFAULT_RGB_WIDTH),
                f"gaia.{GaiaYamlKey.RGB_WIDTH}",
            ),
            depth_height=_gaia_require_int(
                d.get(GaiaYamlKey.DEPTH_HEIGHT, _DEFAULT_DEPTH_HEIGHT),
                f"gaia.{GaiaYamlKey.DEPTH_HEIGHT}",
            ),
            depth_width=_gaia_require_int(
                d.get(GaiaYamlKey.DEPTH_WIDTH, _DEFAULT_DEPTH_WIDTH),
                f"gaia.{GaiaYamlKey.DEPTH_WIDTH}",
            ),
        )


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

        diagram = builder.Build()
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(0.0)
        simulator.Initialize()

        plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())

        # Hold zero actuation by default; concrete command application
        # will land in a follow-up that wires in a tracking controller.
        plant.get_actuation_input_port().FixValue(plant_context, np.zeros(plant.num_actuators()))

        self.plant = plant
        self.scene_graph = scene_graph
        self.diagram = diagram
        self.simulator = simulator
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

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise GaiaError("Gaia.finalize() must be called before Gaia can be used")
