"""
Aegis top-level assembly.

Builds the full Diagram out of Helios, Talos, Metis, and Kyber, with backends
chosen according to ``AegisMode``. The graph shape (ports, wiring, sub-system
set) is identical in both modes; only the backends change.

Data flow:

    Helios.rgb_image        -> Metis.rgb_image
    Helios.depth_image      -> Metis.depth_image
    Talos.proprioception    -> Metis.proprioception
                            -> Kyber.proprioception
    Metis.action            -> Kyber.action
    Kyber.command           -> Talos.command
"""

from __future__ import annotations

import attr
from pydrake.systems.framework import Diagram, DiagramBuilder

from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend, HardwareSensorBackendConfig
from manor.common.aegis.helios.helios import Helios, HeliosPorts, SensorBackend
from manor.common.aegis.helios.sim_backend import SimSensorBackend, SimSensorBackendConfig
from manor.common.aegis.kyber.kyber import Kyber, KyberPorts
from manor.common.aegis.metis.metis import Metis, MetisPorts, Policy
from manor.common.aegis.mode import AegisMode
from manor.common.aegis.talos.hardware_backend import HardwareManipulatorBackend, HardwareManipulatorBackendConfig
from manor.common.aegis.talos.sim_backend import SimManipulatorBackend, SimManipulatorBackendConfig
from manor.common.aegis.talos.talos import ManipulatorBackend, Talos, TalosPorts
from manor.common.custom_types import FilePath
from manor.common.exceptions import InvalidDefinitionError


@attr.frozen
class AegisFrequencies:
    """
    Per-subsystem publish rates in Hz.

    Defaults are order-of-magnitude sensible for a research manipulation
    stack (cameras ~30 Hz, the actuation + state loop in the hundreds, a
    high-level policy in the tens, low-level control the fastest).
    """

    helios_hz: float = 30.0
    talos_hz: float = 200.0
    metis_hz: float = 10.0
    kyber_hz: float = 500.0


@attr.frozen
class AegisSystems:
    """
    Handles to each sub-system in the built diagram. Useful for tests and
    for wiring external driver / recorder systems after ``build_aegis``
    returns.
    """

    helios: Helios
    talos: Talos
    metis: Metis
    kyber: Kyber


def build_aegis(
    mode: AegisMode,
    policy: Policy,
    robot_model_path: FilePath | None = None,
    frequencies: AegisFrequencies | None = None,
) -> tuple[Diagram, AegisSystems]:
    """
    Build and wire the full Aegis diagram.

    ``mode`` selects the SensorBackend / ManipulatorBackend pair; all other
    sub-systems are mode-agnostic. Returns the built Diagram alongside a
    struct of handles to each leaf for external access.
    """

    frequencies = frequencies if frequencies is not None else AegisFrequencies()
    sensor_backend, manipulator_backend = _build_backends(mode)

    builder = DiagramBuilder()
    helios = builder.AddSystem(Helios(backend=sensor_backend, publish_frequency=frequencies.helios_hz))
    talos = builder.AddSystem(
        Talos(
            backend=manipulator_backend,
            robot_model_path=robot_model_path,
            publish_frequency=frequencies.talos_hz,
        )
    )
    metis = builder.AddSystem(Metis(policy=policy, publish_frequency=frequencies.metis_hz))
    kyber = builder.AddSystem(Kyber(publish_frequency=frequencies.kyber_hz))

    helios.set_name("helios")
    talos.set_name("talos")
    metis.set_name("metis")
    kyber.set_name("kyber")

    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        metis.GetInputPort(MetisPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        talos.GetOutputPort(TalosPorts.OUTPUT_PROPRIOCEPTION),
        kyber.GetInputPort(KyberPorts.INPUT_PROPRIOCEPTION),
    )
    builder.Connect(
        helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE),
        metis.GetInputPort(MetisPorts.INPUT_RGB_IMAGE),
    )
    builder.Connect(
        helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE),
        metis.GetInputPort(MetisPorts.INPUT_DEPTH_IMAGE),
    )
    builder.Connect(
        metis.GetOutputPort(MetisPorts.OUTPUT_ACTION),
        kyber.GetInputPort(KyberPorts.INPUT_ACTION),
    )
    builder.Connect(
        kyber.GetOutputPort(KyberPorts.OUTPUT_COMMAND),
        talos.GetInputPort(TalosPorts.INPUT_COMMAND),
    )

    diagram = builder.Build()
    diagram.set_name(f"aegis_{mode.value}")
    return diagram, AegisSystems(helios=helios, talos=talos, metis=metis, kyber=kyber)


def _build_backends(mode: AegisMode) -> tuple[SensorBackend, ManipulatorBackend]:
    if mode == AegisMode.SIM:
        return (
            SimSensorBackend(config=SimSensorBackendConfig()),
            SimManipulatorBackend(config=SimManipulatorBackendConfig()),
        )
    if mode == AegisMode.HARDWARE:
        return (
            HardwareSensorBackend(config=HardwareSensorBackendConfig()),
            HardwareManipulatorBackend(config=HardwareManipulatorBackendConfig()),
        )
    raise InvalidDefinitionError(f"Unknown AegisMode: {mode!r}")
