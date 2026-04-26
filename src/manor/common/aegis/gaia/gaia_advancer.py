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

from typing import ClassVar, Self

import attr
from pydrake.systems.framework import Context, EventStatus, LeafSystem, State

from manor.common.aegis.gaia.gaia import Gaia
from manor.common.aegis.yaml_utils import assert_keys_match_attrs, require_number

_DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ = 500.0


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
        Parse the ``gaia_advancer_config:`` block of an aegis YAML.
        """
        assert_keys_match_attrs(cls, d, "gaia_advancer_config")
        return cls(
            advance_frequency_hz=require_number(
                d.get("advance_frequency_hz", _DEFAULT_GAIA_ADVANCE_FREQUENCY_HZ),
                "gaia_advancer_config.advance_frequency_hz",
            )
        )


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
