"""
Tests for Helios: port shape, periodic publish, and backend delegation.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator

from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend, HardwareSensorBackendConfig
from manor.common.aegis.helios.helios import Helios, HeliosPorts, SensorBackend
from manor.common.aegis.helios.sim_backend import SimSensorBackend, SimSensorBackendConfig
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.testing_utils import run_manor_tests


class _CountingBackend:
    def __init__(self) -> None:
        self.rgb_calls = 0
        self.depth_calls = 0

    def read_rgb(self) -> RGBImageData:
        self.rgb_calls += 1
        return RGBImageData.construct_default(height=8, width=8)

    def read_depth(self) -> DepthImageData:
        self.depth_calls += 1
        return DepthImageData.construct_default(height=8, width=8)


class TestHeliosConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        with pytest.raises(ValueError):
            Helios(backend=_CountingBackend(), publish_frequency=0.0)

    def test_declares_expected_ports(self) -> None:
        helios = Helios(backend=_CountingBackend(), publish_frequency=30.0)
        assert helios.num_input_ports() == 0
        assert helios.num_output_ports() == 2
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE) is not None
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE) is not None


class TestHeliosPublishing:
    def test_periodic_update_polls_backend(self) -> None:
        backend = _CountingBackend()
        helios = Helios(backend=backend, publish_frequency=100.0)
        context = helios.CreateDefaultContext()
        simulator = Simulator(helios, context)
        simulator.AdvanceTo(0.05)

        assert backend.rgb_calls >= 1
        assert backend.depth_calls >= 1

    def test_output_ports_carry_backend_frames(self) -> None:
        helios = Helios(backend=_CountingBackend(), publish_frequency=50.0)
        context = helios.CreateDefaultContext()
        simulator = Simulator(helios, context)
        simulator.AdvanceTo(0.05)

        rgb = helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE).Eval(simulator.get_context())
        depth = helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE).Eval(simulator.get_context())
        assert isinstance(rgb, RGBImageData)
        assert isinstance(depth, DepthImageData)
        assert rgb.height == 8 and rgb.width == 8
        assert depth.height == 8 and depth.width == 8


class TestSensorBackendProtocolCompliance:
    def test_sim_backend_satisfies_protocol(self) -> None:
        backend = SimSensorBackend(config=SimSensorBackendConfig())
        assert isinstance(backend, SensorBackend)
        assert isinstance(backend.read_rgb(), RGBImageData)
        assert isinstance(backend.read_depth(), DepthImageData)

    def test_hardware_backend_satisfies_protocol(self) -> None:
        backend = HardwareSensorBackend(config=HardwareSensorBackendConfig())
        assert isinstance(backend, SensorBackend)
        assert isinstance(backend.read_rgb(), RGBImageData)
        assert isinstance(backend.read_depth(), DepthImageData)


if __name__ == "__main__":
    run_manor_tests()
