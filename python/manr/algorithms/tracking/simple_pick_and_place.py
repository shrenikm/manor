"""
Simple pick and place of a 1 inch block.
"""

from typing import Tuple

import numpy as np
from pydrake.all import DiagramBuilder
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import Context, Diagram, EventStatus
from pydrake.systems.primitives import TrajectorySource
from pydrake.trajectories import PiecewisePose

from manr.common.model_utils import ObjectModelConfig, ObjectModelType
from manr.lite6.pliant.lite6_pliant import create_lite6_pliant
from manr.lite6.pliant.lite6_pliant_utils import (
    LITE6_PLIANT_GSD_IP_NAME,
    LITE6_PLIANT_PE_OP_NAME,
    LITE6_PLIANT_VD_IP_NAME,
    LITE6_PLIANT_VE_OP_NAME,
    Lite6PliantConfig,
    Lite6PliantType,
    create_simulator_for_lite6_pliant,
    get_tuned_pid_gains_for_pliant_id_controller,
)
from manr.lite6.systems.lite6_diff_ik_controller import Lite6DiffIKController
from manr.lite6.systems.lite6_gripper_status_source import Lite6GripperStatusSource
from manr.lite6.utils.lite6_model_utils import (
    Lite6ControlType,
    Lite6GripperStatus,
    Lite6ModelType,
    create_lite6_plant_for_system,
    get_default_height_for_object_model_type,
    get_lite6_urdf_eef_tip_frame_name,
    get_unactuated_parallel_gripper_counterpart,
)

OBJECT_TO_GRIPPER_Z = 0.0
G_TO_F_Z = -0.03
G_WAIT_TIME = 1.0
EEF_LINEAR_VELOCITY = 0.05


def _construct_trajectory_sources(
    X_WG: RigidTransform,
    X_WOPick: RigidTransform,
    X_WOPlace: RigidTransform,
    X_WOEnd: RigidTransform,
) -> Tuple[float, TrajectorySource, Lite6GripperStatusSource]:
    X_OG = RigidTransform(
        R=RotationMatrix.MakeXRotation(theta=np.pi),
        p=np.array([0.0, 0.0, OBJECT_TO_GRIPPER_Z], dtype=np.float64),
    )
    X_GF = RigidTransform(
        p=np.array([0.0, 0.0, G_TO_F_Z], dtype=np.float64),
    )
    X_WGPick = X_WOPick @ X_OG
    X_WGPlace = X_WOPlace @ X_OG
    X_WEnd = X_WOEnd @ X_OG

    X_WFPick = X_WGPick @ X_GF
    X_WFPlace = X_WGPlace @ X_GF

    # Constructing the trajectory. We have some duplicate poses
    # to wait for gripper open/close.
    poses = [
        X_WG,
        X_WFPick,
        X_WGPick,
        X_WGPick,
        X_WFPlace,
        X_WGPlace,
        X_WGPlace,
        X_WFPlace,
        X_WG,
    ]
    # To compute the times estimate, we use an estimated required gripper velocity and the
    # Euclidean distance between poses.
    GFPick_distance = np.linalg.norm(X_WG.translation() - X_WFPick.translation())
    GFPick_time = GFPick_distance / EEF_LINEAR_VELOCITY

    GPickFPlace_distance = np.linalg.norm(X_WGPick.translation() - X_WFPlace.translation())
    GPickFPlace_time = GPickFPlace_distance / EEF_LINEAR_VELOCITY

    FPlaceG_distance = np.linalg.norm(X_WFPlace.translation() - X_WG.translation())
    FPlaceG_time = FPlaceG_distance / EEF_LINEAR_VELOCITY

    g_f_time = np.abs(G_TO_F_Z / EEF_LINEAR_VELOCITY)

    pick_time = GFPick_time + g_f_time + G_WAIT_TIME
    place_time = pick_time + GPickFPlace_time + g_f_time

    times = [
        0.0,
        GFPick_time,
        GFPick_time + g_f_time,
        pick_time,
        pick_time + GPickFPlace_time,
        place_time,
        place_time + G_WAIT_TIME,
        place_time + G_WAIT_TIME + g_f_time,
        place_time + G_WAIT_TIME + g_f_time + FPlaceG_time,
    ]

    X_WG_trajectory = PiecewisePose.MakeLinear(
        times=times,
        poses=poses,
    )
    V_WG_trajectory = X_WG_trajectory.MakeDerivative(derivative_order=1)
    end_time = V_WG_trajectory.end_time()

    gripper_V_trajectory_source = TrajectorySource(
        trajectory=V_WG_trajectory,
    )

    gripper_times = [
        0.0,
        GFPick_time + g_f_time,
        place_time,
        place_time + G_WAIT_TIME + g_f_time,
    ]
    gripper_statuses = [
        Lite6GripperStatus.OPEN,
        Lite6GripperStatus.CLOSED,
        Lite6GripperStatus.OPEN,
        Lite6GripperStatus.CLOSED,
    ]

    gripper_status_source = Lite6GripperStatusSource(
        times=gripper_times,
        statuses=gripper_statuses,
    )

    return end_time, gripper_V_trajectory_source, gripper_status_source


def execute_simple_pick_and_place(
    config: Lite6PliantConfig,
    X_WOPick: RigidTransform,
    X_WOPlace: RigidTransform,
    X_WOEnd: RigidTransform,
) -> None:
    unactuated_lite6_model_type = get_unactuated_parallel_gripper_counterpart(
        lite6_model_type=config.lite6_model_type,
    )

    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )
    # TODO: Handle plant stuff for hardware pliant
    main_plant = lite6_pliant_container.plant
    assert main_plant is not None

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.pliant_diagram,
    )

    main_plant_context = main_plant.CreateDefaultContext()
    X_WG = main_plant.EvalBodyPoseInWorld(
        context=main_plant_context,
        body=main_plant.GetBodyByName(
            name=get_lite6_urdf_eef_tip_frame_name(
                lite6_model_type=config.lite6_model_type,
            ),
        ),
    )

    (
        end_time,
        gripper_V_trajectory_source,
        gripper_status_source,
    ) = _construct_trajectory_sources(
        X_WG=X_WG,
        X_WOPick=X_WOPick,
        X_WOPlace=X_WOPlace,
        X_WOEnd=X_WOEnd,
    )

    gripper_V_trajectory_source = builder.AddSystem(
        gripper_V_trajectory_source,
    )

    gripper_status_source = builder.AddSystem(
        gripper_status_source,
    )

    ik_plant = create_lite6_plant_for_system(
        time_step=config.plant_config.time_step,
        lite6_model_type=unactuated_lite6_model_type,
    )
    lite6_diff_ik_controller = builder.AddSystem(
        Lite6DiffIKController(
            plant=ik_plant,
            lite6_model_type=unactuated_lite6_model_type,
        ),
    )

    # Connect the IK controller.
    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_PE_OP_NAME),
        lite6_diff_ik_controller.ik_pe_input_port,
    )
    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_VE_OP_NAME),
        lite6_diff_ik_controller.ik_ve_input_port,
    )
    builder.Connect(
        gripper_V_trajectory_source.get_output_port(),
        lite6_diff_ik_controller.ik_gvd_input_port,
    )

    # Connect the Pliant.
    builder.Connect(
        lite6_diff_ik_controller.ik_vd_output_port,
        lite6_pliant.GetInputPort(LITE6_PLIANT_VD_IP_NAME),
    )

    builder.Connect(
        gripper_status_source.get_output_port(),
        lite6_pliant.GetInputPort(LITE6_PLIANT_GSD_IP_NAME),
    )

    diagram = builder.Build()
    simulator = create_simulator_for_lite6_pliant(
        config=config,
        diagram=diagram,
    )
    simulator_context = simulator.get_mutable_context()

    diagram.ForcedPublish(simulator_context)

    # Set up the monitor to end the simulation.
    def simulation_end_monitor(context: Context):
        if context.get_time() > end_time:
            return EventStatus.ReachedTermination(
                diagram,
                "Trajectory end time reached. Stopping simulation!",
            )

    # Monitor to stop the simulation after choreography is done.
    simulator.set_monitor(
        monitor=simulation_end_monitor,
    )

    with lite6_pliant_container.auto_meshcat_visualization(record=True):
        simulator.AdvanceTo(
            boundary_time=np.inf,
            interruptible=True,
        )


if __name__ == "__main__":
    pick_xy = np.array([0.20, 0.0])
    place_xy = np.array([0.17, -0.12])
    end_xy = np.array([0.12, 0.0])
    place_height_padding = 0.005

    lite6_model_type = Lite6ModelType.ROBOT_WITH_ARP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.SIMULATION
    id_controller_pid_gains = get_tuned_pid_gains_for_pliant_id_controller(
        lite6_control_type=lite6_control_type,
    )
    time_step = 0.001
    hardware_control_loop_time_step = 0.02
    plant_config = MultibodyPlantConfig(
        time_step=time_step,
        # TODO: Better way to get this string?
        contact_model="hydroelastic_with_fallback",
    )

    cube_base_height = get_default_height_for_object_model_type(
        ObjectModelType.CUBE_1_INCH_BLUE,
    )
    # Blue cube is the cube to pick.
    blue_cube = ObjectModelConfig(
        object_model_type=ObjectModelType.CUBE_1_INCH_BLUE,
        position=np.hstack((pick_xy, cube_base_height)),
    )

    # Stack of the other 3 cubes.
    yellow_cube = ObjectModelConfig(
        object_model_type=ObjectModelType.CUBE_1_INCH_YELLOW,
        position=np.hstack((place_xy, cube_base_height)),
    )
    red_cube = ObjectModelConfig(
        object_model_type=ObjectModelType.CUBE_1_INCH_RED,
        position=np.hstack((place_xy, cube_base_height + 0.01 * 2.54)),
    )
    green_cube = ObjectModelConfig(
        object_model_type=ObjectModelType.CUBE_1_INCH_GREEN,
        position=np.hstack((place_xy, cube_base_height + 2 * 0.01 * 2.54)),
    )

    object_model_configs = [
        blue_cube,
        yellow_cube,
        red_cube,
        green_cube,
    ]

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
        inverse_dynamics_pid_gains=id_controller_pid_gains,
        plant_config=plant_config,
        hardware_control_loop_time_step=hardware_control_loop_time_step,
        object_model_configs=object_model_configs,
    )

    # Blue cube is the one to pick
    pick_position = np.copy(blue_cube.position)

    # The blue cube needs to be placed over the green cube (top of the stack of cubes)
    place_position = np.copy(green_cube.position)
    place_position[2] += 0.01 * 2.54 + place_height_padding

    end_position = np.hstack((end_xy, cube_base_height))

    X_WOPick = RigidTransform(p=pick_position)
    X_WOPlace = RigidTransform(p=place_position)
    X_WOEnd = RigidTransform(p=end_position)

    execute_simple_pick_and_place(
        config=lite6_pliant_config,
        X_WOPick=X_WOPick,
        X_WOPlace=X_WOPlace,
        X_WOEnd=X_WOEnd,
    )
