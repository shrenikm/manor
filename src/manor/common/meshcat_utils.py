"""
Utilities for spawning Drake meshcat instances with predictable port behavior.

Drake's default StartMeshcat() probes 7000-7999 and silently picks the first free port. That is convenient for
notebooks but painful for a development loop where the URL is expected to be stable across Ctrl-C / relaunch
cycles: an abandoned socket on 7000 pushes the next run to 7001, the bookmarked browser tab points at nothing,
and the drift compounds. start_meshcat() pins the port explicitly so a stale process produces a loud error
rather than a moving target, and waits briefly for a recently-released socket to clear TIME_WAIT before giving
up.
"""

from __future__ import annotations

import socket
import time
from typing import Final

from pydrake.geometry import Meshcat, MeshcatParams

from manor.common.exceptions import MeshcatPortBusyError

DEFAULT_MESHCAT_PORT: Final[int] = 7000

# Linux normally releases an abandoned listening socket immediately, but a lingering ESTABLISHED browser
# connection at shutdown can keep the 4-tuple in TIME_WAIT for a couple of seconds. A short bounded wait
# covers that case without masking a genuine "someone else is on this port" error.
_DEFAULT_PORT_WAIT_S: Final[float] = 5.0
_PORT_POLL_INTERVAL_S: Final[float] = 0.1
_LOOPBACK_HOST: Final[str] = "127.0.0.1"


def _is_port_bindable(port: int) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((_LOOPBACK_HOST, port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _wait_for_port(port: int, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s
    while True:
        if _is_port_bindable(port):
            return
        if time.monotonic() >= deadline_s:
            raise MeshcatPortBusyError(
                f"Port {port} is still in use after waiting {timeout_s:.1f}s. A previous meshcat process "
                f"may still be running; check with 'lsof -i :{port}' or 'fuser {port}/tcp' and kill it "
                f"before retrying."
            )
        time.sleep(_PORT_POLL_INTERVAL_S)


def start_meshcat(port: int = DEFAULT_MESHCAT_PORT, wait_timeout_s: float = _DEFAULT_PORT_WAIT_S) -> Meshcat:
    """
    Construct a Meshcat instance pinned to the given port. Waits up to wait_timeout_s for a stale TIME_WAIT
    socket to clear, then raises MeshcatPortBusyError if the port is genuinely held by another live process
    instead of silently drifting to the next free port the way StartMeshcat() does.
    """
    _wait_for_port(port=port, timeout_s=wait_timeout_s)
    return Meshcat(MeshcatParams(port=port))
