"""
CircleEEVelocityPolicy: emits a Cartesian-twist command that traces a
circle in the world xy-plane (z up) at constant linear speed.

Geometry:

* The first step latches the current EE position from
  observation.proprioception.cartesian_state.cartesian_pose.translation
  and treats that point as the bottom of the circle.
* The circle centre is offset from the start point by +radius in y
  (so the circle lives in the xy-plane and the start is on the
  -y rim).
* Direction of travel is anticlockwise viewed from +z (i.e. the
  angular position around the circle increases over time).
* The arm motion is purely translational; angular velocity is zero.

After duration_seconds elapse the policy emits a zero CartesianTwist --
"velocity hold", per the user's spec; not a cartesian-pose hold.

If the first step happens before proprioception with cartesian_state is
available, the policy emits zero twist until it is. Latching is
deferred, not synthesised from a default position, to keep the trace
geometry tied to the actual robot pose at start.
"""

from __future__ import annotations

from typing import ClassVar, Self

import attr
import numpy as np

from manor.common.aegis.metis.policies.policy_manager import MetisPolicyConfigBase, MetisPolicyType
from manor.common.aegis.yaml_utils import parse_attrs_yaml
from manor.common.definitions.action import Action
from manor.common.definitions.cartesian_command import CartesianCommand
from manor.common.definitions.cartesian_twist import CartesianTwist
from manor.common.definitions.observation import Observation
from manor.common.definitions.timestamp_header import TimestampHeader


@attr.frozen
class CircleEEVelocityPolicyConfig(MetisPolicyConfigBase):
    """
    Config for CircleEEVelocityPolicy.

    radius is the radius of the circle in metres. velocity_magnitude is
    the constant linear speed in metres per second. duration_seconds is
    how long the policy traces the circle before falling through to a
    zero CartesianTwist (velocity hold).
    """

    POLICY_TYPE: ClassVar[MetisPolicyType] = MetisPolicyType.CIRCLE_EE_VELOCITY

    radius: float = 0.05
    velocity_magnitude: float = 0.05
    duration_seconds: float = 5.0

    @classmethod
    def from_yaml_dict(cls, d: dict) -> Self:
        return cls(**parse_attrs_yaml(cls, d, "CircleEEVelocityPolicyConfig"))


@attr.define
class CircleEEVelocityPolicy:
    """
    Trace a circle in the world xy-plane with constant linear EE speed,
    starting from the latched current EE position as the bottom of the
    circle, anticlockwise viewed from +z. After duration_seconds the
    twist drops to zero (velocity hold).
    """

    radius: float
    velocity_magnitude: float
    duration_seconds: float
    _start_time_s: float | None = attr.field(default=None, init=False)
    _centre: np.ndarray | None = attr.field(default=None, init=False)

    @classmethod
    def from_config(cls, config: CircleEEVelocityPolicyConfig) -> Self:
        return cls(
            radius=config.radius,
            velocity_magnitude=config.velocity_magnitude,
            duration_seconds=config.duration_seconds,
        )

    def step(self, observation: Observation) -> Action:
        header = TimestampHeader.from_system_time()
        now_s = header.system_ns * 1e-9

        if self._centre is None:
            translation = self._extract_translation(observation)
            if translation is None:
                # Defer the latch until proprioception is populated.
                # Until then emit a zero CartesianTwist so the diagram
                # keeps ticking.
                return self._make_action(header, np.zeros(3, dtype=np.float64))
            # Bottom of the circle = latched start point. Centre sits
            # +radius in y so the start is on the -y rim and the
            # trajectory wraps anticlockwise around +z.
            self._centre = translation + np.array([0.0, self.radius, 0.0], dtype=np.float64)
            self._start_time_s = now_s

        elapsed = now_s - (self._start_time_s if self._start_time_s is not None else now_s)
        if elapsed >= self.duration_seconds:
            return self._make_action(header, np.zeros(3, dtype=np.float64))

        linear = self._tangent_velocity(elapsed)
        return self._make_action(header, linear)

    @staticmethod
    def _extract_translation(observation: Observation) -> np.ndarray | None:
        if observation.proprioception is None:
            return None
        cartesian_state = observation.proprioception.cartesian_state
        if cartesian_state is None:
            return None
        return np.asarray(cartesian_state.cartesian_pose.translation, dtype=np.float64).copy()

    def _tangent_velocity(self, elapsed_s: float) -> np.ndarray:
        # Angular speed sized so the linear speed at the rim equals
        # velocity_magnitude. Anticlockwise from +z view -> phase
        # increases with time.
        if self.radius <= 0.0:
            return np.zeros(3, dtype=np.float64)
        omega = self.velocity_magnitude / self.radius
        # Start phase: bottom of the circle (start = centre - radius * y_hat)
        # corresponds to phase = -pi/2 in standard convention. The
        # tangent at that phase, anticlockwise, points in +x. Tracking
        # phi(t) = -pi/2 + omega * t gives tangent = (-sin(phi), cos(phi)).
        phi = -np.pi / 2.0 + omega * elapsed_s
        tangent = np.array([-np.sin(phi), np.cos(phi), 0.0], dtype=np.float64)
        return tangent * self.velocity_magnitude

    def _make_action(self, header: TimestampHeader, linear: np.ndarray) -> Action:
        return Action(
            header=header,
            cartesian_command=CartesianCommand(
                header=header,
                cartesian_twist=CartesianTwist(
                    header=header,
                    linear=linear,
                    angular=np.zeros(3, dtype=np.float64),
                ),
            ),
        )
