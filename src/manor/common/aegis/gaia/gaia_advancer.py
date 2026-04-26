"""
``GaiaAdvancer``: sim-mode-only LeafSystem that drives a ``Gaia`` forward.

The aegis backends call into a shared ``Gaia`` instance to read sensor
frames and joint state, but Gaia's clock is otherwise idle -- nothing
in those calls advances physics. ``GaiaAdvancer`` plugs that gap: at a
fixed period it calls ``gaia.advance_to(context.get_time())``, keeping
Gaia's simulated clock in step with the outer aegis diagram clock.

It has no input or output ports; it's pure side-effect. The default
period (~500 Hz) is fast relative to camera + policy rates so sensor
reads land on a freshly stepped Gaia state.
"""

from __future__ import annotations

from enum import StrEnum
from typing import ClassVar, Self

import attr
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.exceptions import AegisConfigError

_DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ = 500.0


class GaiaAdvancerYamlKey(StrEnum):
    """
    YAML field names for the ``gaia_advancer:`` block of an aegis config.
    """

    ADVANCE_FREQUENCY_HZ = "advance_frequency_hz"


@attr.frozen
class GaiaAdvancerConfig:
    """
    ``GaiaAdvancer`` configuration. ``advance_frequency_hz`` is the
    cadence at which the diagram clock is forwarded into
    ``Gaia.advance_to``. ``SYSTEM_NAME`` is the name applied to the
    GaiaAdvancer LeafSystem in the diagram.
    """

    SYSTEM_NAME: ClassVar[str] = "gaia_advancer"

    advance_frequency_hz: float = _DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        """
        Parse the ``gaia_advancer:`` block of an aegis YAML.
        """
        allowed = {key.value for key in GaiaAdvancerYamlKey}
        extras = set(d) - allowed
        if extras:
            raise AegisConfigError(f"gaia_advancer: unexpected keys {sorted(extras)!r}; allowed {sorted(allowed)!r}")
        advance_frequency_hz = d.get(GaiaAdvancerYamlKey.ADVANCE_FREQUENCY_HZ, _DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ)
        if isinstance(advance_frequency_hz, bool) or not isinstance(advance_frequency_hz, (int, float)):
            raise AegisConfigError(
                f"gaia_advancer.{GaiaAdvancerYamlKey.ADVANCE_FREQUENCY_HZ} must be a number; "
                f"got {type(advance_frequency_hz).__name__}"
            )
        return cls(advance_frequency_hz=float(advance_frequency_hz))


class GaiaAdvancer(LeafSystem):
    """
    LeafSystem that advances a ``Gaia`` on a fixed-period event.
    """

    def __init__(self, gaia: Gaia, advance_frequency_hz: float = _DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ) -> None:
        super().__init__()
        if advance_frequency_hz <= 0.0:
            raise ValueError(f"advance_frequency_hz must be positive, got {advance_frequency_hz}")

        self.gaia = gaia
        self.advance_frequency_hz = advance_frequency_hz

        self.DeclarePeriodicUnrestrictedUpdateEvent(
            period_sec=1.0 / advance_frequency_hz,
            offset_sec=0.0,
            update=self._periodic_update,
        )

    def _periodic_update(self, context: Context, state: State) -> EventStatus:
        del state
        self.gaia.advance_to(context.get_time())
        return EventStatus.Succeeded()
