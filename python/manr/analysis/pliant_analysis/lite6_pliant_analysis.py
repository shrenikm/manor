import numpy as np
from pydrake.all import DiagramBuilder
from pydrake.common.value import Value
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import Diagram, EventStatus

from manr.analysis.pliant_analysis.lite6_pliant_analysis_choreographer import (
    Lite6PliantChoreographer,
    Lite6PliantChoreographerController,
    get_choreographer_config_yaml_filepath,
)
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
from manr.lite6.utils.lite6_model_utils import Lite6ControlType, Lite6GripperStatus, Lite6ModelType


def analyze_lite6_pliant(
    config: Lite6PliantConfig,
    choreographer: Lite6PliantChoreographer,
) -> None:
    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )

    lite6_pliant: Diagram = builder.AddNamedSystem(
        name="lite6_pliant",
        system=lite6_pliant_container.pliant_diagram,
    )

    choreographer_controller = builder.AddSystem(
        Lite6PliantChoreographerController(
            config=config,
            choreographer=choreographer,
        ),
    )

    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_PE_OP_NAME),
        choreographer_controller.cc_pe_input_port,
    )
    builder.Connect(
        lite6_pliant.GetOutputPort(LITE6_PLIANT_VE_OP_NAME),
        choreographer_controller.cc_ve_input_port,
    )
    builder.Connect(
        choreographer_controller.cc_output_port,
        lite6_pliant.GetInputPort(LITE6_PLIANT_VD_IP_NAME),
    )

    diagram = builder.Build()
    simulator = create_simulator_for_lite6_pliant(
        config=config,
        diagram=diagram,
    )
    simulator_context = simulator.get_mutable_context()
    lite6_pliant_context = lite6_pliant.GetMyContextFromRoot(simulator_context)

    lite6_pliant.GetInputPort(
        port_name=LITE6_PLIANT_GSD_IP_NAME,
    ).FixValue(lite6_pliant_context, Value(Lite6GripperStatus.NEUTRAL))

    diagram.ForcedPublish(simulator_context)

    def simulation_end_monitor(*_):
        if choreographer_controller.is_done():
            return EventStatus.ReachedTermination(diagram, "Choreography and recording done!")

    # Monitor to stop the simulation after choreography is done.
    simulator.set_monitor(
        monitor=simulation_end_monitor,
    )

    with lite6_pliant_container.auto_meshcat_visualization(record=False):
        simulator.AdvanceTo(
            boundary_time=np.inf,
            interruptible=True,
        )

    choreographer_controller.plot_recordings()


if __name__ == "__main__":
    lite6_model_type = Lite6ModelType.ROBOT_WITH_ARP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.SIMULATION
    id_controller_pid_gains = get_tuned_pid_gains_for_pliant_id_controller(
        lite6_control_type=lite6_control_type,
    )
    plant_config = MultibodyPlantConfig(time_step=0.001)
    hardware_control_loop_time_step = 0.001

    lite6_pliant_config = Lite6PliantConfig(
        lite6_model_type=lite6_model_type,
        lite6_control_type=lite6_control_type,
        lite6_pliant_type=lite6_pliant_type,
        inverse_dynamics_pid_gains=id_controller_pid_gains,
        plant_config=plant_config,
        hardware_control_loop_time_step=hardware_control_loop_time_step,
    )
    choreographer = Lite6PliantChoreographer.from_yaml(
        yaml_filepath=get_choreographer_config_yaml_filepath(),
    )

    analyze_lite6_pliant(
        config=lite6_pliant_config,
        choreographer=choreographer,
    )
