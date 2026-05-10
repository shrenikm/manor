"""
Shared helpers for the per-block aegis runners.

Each per-block runner builds a Drake Diagram and a Simulator and then needs to advance it forever (until the
supervisor sends SIGTERM, or the user hits Ctrl-C in standalone mode). advance_until_signal is the canonical loop for
that -- it sets up SIGTERM/SIGINT handlers, advances the simulator in fixed system-time chunks, and returns cleanly
when a stop signal arrives.
"""

from __future__ import annotations

import signal
import sys

from pydrake.systems.analysis import Simulator

# Each AdvanceTo call is a blocking system-time-bounded slice; the runner returns to the loop between slices so signal
# handlers can fire promptly. One second strikes a balance between handler latency and Drake's per-call overhead.
_ADVANCE_SLICE_SECONDS = 1.0


def advance_until_signal(simulator: Simulator) -> None:
    """
    Advance simulator forever, returning cleanly when SIGTERM / SIGINT arrives. Used by every per-block runner as the
    "run forever" entry from a subprocess context.
    """
    stop = False

    def _on_signal(signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True
        # Mirror to stderr so the supervisor sees the block honoring the signal -- helps differentiate "clean
        # shutdown" from "hung process that had to be SIGKILLed".
        print(f"[aegis-run] received signal {signum}; stopping", file=sys.stderr, flush=True)

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    t = simulator.get_context().get_time()
    while not stop:
        t += _ADVANCE_SLICE_SECONDS
        try:
            simulator.AdvanceTo(t)
        except KeyboardInterrupt:
            break
