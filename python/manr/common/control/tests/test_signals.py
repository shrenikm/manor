import matplotlib.pyplot as plt
import numpy as np

from manr.common.control.signals import SineControlSignal, StepControlSignal
from manr.common.custom_types import ControlSignalVector, TimesVector
from manr.common.testing_utils import execute_pytest_file


def _plot_signals(
    times: TimesVector,
    signals: ControlSignalVector,
) -> None:
    fig = plt.figure()
    ax = fig.gca()
    ax.plot(times, signals)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("signal")
    plt.show()


def test_step_control_signal(debug: bool = False) -> None:
    value = 0.3
    delay_time = 0.5
    signal = StepControlSignal(
        value=value,
        delay_time=delay_time,
    )

    times = np.arange(0.0, 2.05, 0.05)
    signals = [signal.compute_signal(time_step=t) for t in times]

    for t, v in zip(times, signals):
        if t > delay_time:
            np.testing.assert_array_almost_equal(v, value, decimal=6)
        else:
            np.testing.assert_array_almost_equal(v, 0.0, decimal=6)

    if debug:
        _plot_signals(times, signals)


def test_sine_control_signal(debug: bool = False) -> None:
    # Test basic sine wave without any phase shift or offsets.
    amplitude = 0.3
    frequency = 2.0
    phase_shift = 0.0
    offset = 0.0

    signal = SineControlSignal(
        amplitude=amplitude,
        frequency=frequency,
        phase_shift=phase_shift,
        offset=offset,
    )

    times = np.arange(0.0, 1.05, 0.05)
    signals = [signal.compute_signal(time_step=t) for t in times]

    # Frequency = 2. means 2 time periods within the second.
    # Hence it must end exactly at 0.
    np.testing.assert_array_almost_equal(signals[0], 0.0, decimal=6)
    np.testing.assert_array_almost_equal(signals[-1], 0.0, decimal=6)

    if debug:
        _plot_signals(times, signals)


def test_sine_standard_positive_signal(debug: bool = False) -> None:
    amplitude = 0.3
    frequency = 2.0

    signal = SineControlSignal.standard_positive_signal(
        amplitude=amplitude,
        frequency=frequency,
    )

    times = np.arange(0.0, 1.05, 0.05)
    signals = [signal.compute_signal(time_step=t) for t in times]

    # Frequency = 2. means 2 time periods within the second.
    # Hence it must end exactly at 0.
    np.testing.assert_array_almost_equal(signals[0], 0.0, decimal=6)
    np.testing.assert_array_almost_equal(signals[-1], 0.0, decimal=6)

    # Min and max signal must be 0 and 0.3
    # We don't test for 0.3 directly as we're limited by the time resolution
    # It is enough to test that the min is 0.
    np.testing.assert_array_almost_equal(np.min(signals), 0.0, decimal=6)

    if debug:
        _plot_signals(times, signals)


if __name__ == "__main__":
    execute_pytest_file()
