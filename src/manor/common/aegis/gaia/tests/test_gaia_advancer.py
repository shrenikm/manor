from __future__ import annotations

import pytest
from pydrake.systems.analysis import Simulator

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.gaia.gaia_advancer import GaiaAdvancer
from manor.common.testing_utils import run_manor_tests
from manor.manipulators.lite6.model import Lite6Model
from manor.manipulators.lite6.variant import Lite6Variant


def _make_gaia() -> Gaia:
    gaia = Gaia(manipulator_model=Lite6Model(variant=Lite6Variant.PARALLEL_GRIPPER_NORMAL))
    gaia.finalize()
    return gaia


class TestConstruction:
    def test_rejects_non_positive_frequency(self) -> None:
        gaia = _make_gaia()
        with pytest.raises(ValueError):
            GaiaAdvancer(gaia=gaia, advance_frequency_hz=0.0)

    def test_stores_gaia_and_frequency(self) -> None:
        gaia = _make_gaia()
        advancer = GaiaAdvancer(gaia=gaia, advance_frequency_hz=200.0)
        assert advancer.gaia is gaia
        assert advancer.advance_frequency_hz == 200.0


class TestPeriodicUpdate:
    def test_advances_gaia_clock_in_step_with_diagram(self) -> None:
        gaia = _make_gaia()
        advancer = GaiaAdvancer(gaia=gaia, advance_frequency_hz=200.0)

        simulator = Simulator(advancer)
        simulator.AdvanceTo(0.05)

        assert gaia.simulator.get_context().get_time() > 0.0


if __name__ == "__main__":
    run_manor_tests()
