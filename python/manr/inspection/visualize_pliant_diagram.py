from io import BytesIO

import pydot
from PIL import Image
from pydrake.all import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.framework import Diagram

from manr.lite6.pliant.lite6_pliant import create_lite6_pliant
from manr.lite6.pliant.lite6_pliant_utils import (
    Lite6PliantConfig,
    Lite6PliantType,
    get_tuned_pid_gains_for_pliant_id_controller,
)
from manr.lite6.utils.lite6_model_utils import Lite6ControlType, Lite6ModelType


def visualize_lite6_pliant_diagram(
    config: Lite6PliantConfig,
) -> None:
    builder = DiagramBuilder()

    lite6_pliant_container = create_lite6_pliant(
        config=config,
    )
    lite6_pliant: Diagram = lite6_pliant_container.pliant_diagram

    viz_png = pydot.graph_from_dot_data(
        s=lite6_pliant.GetGraphvizString(max_depth=1),
    )[0].create_png()

    Image.open(BytesIO(viz_png)).show()


if __name__ == "__main__":
    lite6_model_type = Lite6ModelType.ROBOT_WITH_ARP_GRIPPER
    lite6_control_type = Lite6ControlType.VELOCITY
    lite6_pliant_type = Lite6PliantType.HARDWARE
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

    visualize_lite6_pliant_diagram(
        config=lite6_pliant_config,
    )
