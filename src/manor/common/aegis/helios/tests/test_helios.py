"""
Tests for Helios: port shape, periodic publish, backend delegation,
and the per-stream frequency knobs.
"""

from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.helios.hardware_backend import HardwareSensorBackend, HardwareSensorBackendConfig
from manor.common.aegis.helios.helios import Helios, HeliosPorts, SensorBackend
from manor.common.aegis.helios.sim_backend import SimSensorBackend, SimSensorBackendConfig
from manor.common.definitions.depth_image_data import DepthImageData
from manor.common.definitions.rgb_image_data import RGBImageData
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


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
    def test_rejects_negative_rgb_frequency(self) -> None:
        with pytest.raises(ValueError):
            Helios(backend=_CountingBackend(), publish_rgb_frequency_hz=-1.0)

    def test_rejects_negative_depth_frequency(self) -> None:
        with pytest.raises(ValueError):
            Helios(backend=_CountingBackend(), publish_depth_frequency_hz=-1.0)

    def test_declares_both_streams_when_both_frequencies_positive(self) -> None:
        helios = Helios(
            backend=_CountingBackend(),
            publish_rgb_frequency_hz=30.0,
            publish_depth_frequency_hz=30.0,
        )
        assert helios.num_input_ports() == 0
        assert helios.num_output_ports() == 2
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE) is not None
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE) is not None


class TestHeliosFrequencyFlags:
    def test_zero_depth_frequency_omits_depth_port(self) -> None:
        helios = Helios(
            backend=_CountingBackend(),
            publish_rgb_frequency_hz=30.0,
            publish_depth_frequency_hz=0.0,
        )
        assert helios.num_output_ports() == 1
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE) is not None
        with pytest.raises(RuntimeError):
            helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE)

    def test_zero_rgb_frequency_omits_rgb_port(self) -> None:
        helios = Helios(
            backend=_CountingBackend(),
            publish_rgb_frequency_hz=0.0,
            publish_depth_frequency_hz=30.0,
        )
        assert helios.num_output_ports() == 1
        assert helios.GetOutputPort(HeliosPorts.OUTPUT_DEPTH_IMAGE) is not None
        with pytest.raises(RuntimeError):
            helios.GetOutputPort(HeliosPorts.OUTPUT_RGB_IMAGE)

    def test_dummy_helios_has_no_ports(self) -> None:
        helios = Helios(
            backend=_CountingBackend(),
            publish_rgb_frequency_hz=0.0,
            publish_depth_frequency_hz=0.0,
        )
        assert helios.num_input_ports() == 0
        assert helios.num_output_ports() == 0


class TestHeliosPublishing:
    def test_periodic_update_polls_backend(self) -> None:
        backend = _CountingBackend()
        helios = Helios(backend=backend, publish_rgb_frequency_hz=100.0, publish_depth_frequency_hz=100.0)
        context = helios.CreateDefaultContext()
        simulator = Simulator(helios, context)
        simulator.AdvanceTo(0.05)

        assert backend.rgb_calls >= 1
        assert backend.depth_calls >= 1

    def test_independent_stream_rates(self) -> None:
        # Drive RGB much faster than depth and confirm RGB is polled
        # more often. A slow CI machine could drift, so use loose
        # multiples rather than exact ratios.
        backend = _CountingBackend()
        helios = Helios(backend=backend, publish_rgb_frequency_hz=200.0, publish_depth_frequency_hz=20.0)
        context = helios.CreateDefaultContext()
        simulator = Simulator(helios, context)
        simulator.AdvanceTo(0.2)
        assert backend.rgb_calls > backend.depth_calls

    def test_output_ports_carry_backend_frames(self) -> None:
        helios = Helios(backend=_CountingBackend(), publish_rgb_frequency_hz=50.0, publish_depth_frequency_hz=50.0)
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
        gaia = Gaia(manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL))
        gaia.finalize()
        backend = SimSensorBackend(gaia=gaia, config=SimSensorBackendConfig())
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
