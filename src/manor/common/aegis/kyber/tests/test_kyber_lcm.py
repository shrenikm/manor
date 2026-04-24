"""
In-process LCM roundtrip tests for the Kyber LCM wiring.

Uses Drake's ``memq://`` in-memory LCM URL so a single pytest process can
play both the publisher (``lcm_source``) and subscriber (``kyber_lcm``)
sides, with no network traffic. This exercises the full pipeline:

    source -> lcmt_proprioception / lcmt_action -> memq -> Kyber -> lcmt_command -> memq -> probe

A second LcmSubscriberSystem on AEGIS_COMMAND acts as a probe so the test
can observe what Kyber actually put on the wire.
"""

from __future__ import annotations

from pydrake.lcm import DrakeLcm
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmInterfaceSystem, LcmSubscriberSystem

from manor.common.aegis.kyber.kyber_lcm import (
    AEGIS_COMMAND_CHANNEL,
    build_kyber_lcm_diagram,
)
from manor.common.aegis.kyber.lcm_source import (
    AEGIS_ACTION_CHANNEL,
    AEGIS_PROPRIOCEPTION_CHANNEL,
    build_lcm_source_diagram,
)
from manor.common.definitions.action import Action
from manor.common.definitions.command import Command
from manor.common.definitions.lcmtypes.lcmt_action import lcmt_action
from manor.common.definitions.lcmtypes.lcmt_command import lcmt_command
from manor.common.definitions.lcmtypes.lcmt_proprioception import lcmt_proprioception
from manor.common.definitions.proprioception import Proprioception
from manor.common.testing_utils import run_manor_tests

_SOURCE_FREQUENCY_HZ = 20.0
_KYBER_FREQUENCY_HZ = 50.0


def _build_combined_diagram(lcm: DrakeLcm):
    """
    Single diagram that stands up source + Kyber-LCM + a probe subscriber on
    each of the three LCM channels. All systems share one DrakeLcm and one
    LcmInterfaceSystem (which is what pumps the memq queue during
    Simulator.AdvanceTo).
    """

    builder = DiagramBuilder()
    builder.AddSystem(LcmInterfaceSystem(lcm))

    builder.AddSystem(build_lcm_source_diagram(lcm=lcm, publish_frequency_hz=_SOURCE_FREQUENCY_HZ))
    builder.AddSystem(build_kyber_lcm_diagram(lcm=lcm, kyber_publish_frequency_hz=_KYBER_FREQUENCY_HZ))

    proprioception_probe = builder.AddSystem(
        LcmSubscriberSystem.Make(
            channel=AEGIS_PROPRIOCEPTION_CHANNEL,
            lcm_type=lcmt_proprioception,
            lcm=lcm,
        )
    )
    action_probe = builder.AddSystem(
        LcmSubscriberSystem.Make(
            channel=AEGIS_ACTION_CHANNEL,
            lcm_type=lcmt_action,
            lcm=lcm,
        )
    )
    command_probe = builder.AddSystem(
        LcmSubscriberSystem.Make(
            channel=AEGIS_COMMAND_CHANNEL,
            lcm_type=lcmt_command,
            lcm=lcm,
        )
    )

    diagram = builder.Build()
    return diagram, proprioception_probe, action_probe, command_probe


class TestLcmSourceToKyber:
    def test_source_publishes_messages_on_both_input_channels(self) -> None:
        lcm = DrakeLcm("memq://")
        diagram, proprioception_probe, action_probe, _ = _build_combined_diagram(lcm)

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.3)

        root_context = simulator.get_context()
        proprioception_lcm = proprioception_probe.get_output_port().Eval(
            proprioception_probe.GetMyContextFromRoot(root_context)
        )
        action_lcm = action_probe.get_output_port().Eval(action_probe.GetMyContextFromRoot(root_context))

        # A fresh (non-default) header confirms a real publish reached the probe.
        assert proprioception_lcm.header.monotonic_ns > 0
        assert action_lcm.header.monotonic_ns > 0

        # Round-trip through the aegis schema to confirm on-wire integrity.
        Proprioception.from_lcm_message(proprioception_lcm)
        Action.from_lcm_message(action_lcm)

    def test_kyber_publishes_command_sourced_from_incoming_action(self) -> None:
        lcm = DrakeLcm("memq://")
        diagram, _, _, command_probe = _build_combined_diagram(lcm)

        simulator = Simulator(diagram)
        simulator.AdvanceTo(0.3)

        root_context = simulator.get_context()
        command_lcm = command_probe.get_output_port().Eval(command_probe.GetMyContextFromRoot(root_context))
        assert command_lcm.header.monotonic_ns > 0

        command = Command.from_lcm_message(command_lcm)
        # Kyber's passthrough populates the joint_positions variant of Command
        # from the incoming Action. The source emits 6-joint defaults, so the
        # on-wire command should carry a 6-long position vector.
        assert command.joint_positions is not None
        assert command.joint_positions.positions.shape == (6,)

    def test_command_publish_frequency_is_roughly_kyber_frequency(self) -> None:
        """
        Frequency sanity check: advance for a known duration and count how
        many distinct system-time stamps landed on the AEGIS_COMMAND channel.
        Exact equality is not expected -- Drake's publisher ticks on sim time
        and memq delivery is synchronous, so the observed count is bounded
        by the publisher's rate.
        """

        lcm = DrakeLcm("memq://")
        diagram, _, _, command_probe = _build_combined_diagram(lcm)

        # Collect distinct (monotonic_ns) values of the probe output as we advance.
        seen_stamps: set[int] = set()

        simulator = Simulator(diagram)
        total_duration_sec = 1.0
        sample_period_sec = 1.0 / (2.0 * _KYBER_FREQUENCY_HZ)  # Nyquist-ish
        t = 0.0
        while t < total_duration_sec:
            t = min(t + sample_period_sec, total_duration_sec)
            simulator.AdvanceTo(t)
            command_lcm = command_probe.get_output_port().Eval(
                command_probe.GetMyContextFromRoot(simulator.get_context())
            )
            if command_lcm.header.monotonic_ns > 0:
                seen_stamps.add(command_lcm.header.monotonic_ns)

        # At _KYBER_FREQUENCY_HZ over total_duration_sec seconds we expect on
        # the order of _KYBER_FREQUENCY_HZ * total_duration_sec unique
        # command messages. Allow a generous floor (>= 1/3 expected) so a
        # slow CI machine doesn't flake, and an equally generous ceiling
        # (<= 3x) to catch runaway over-publishing.
        expected = _KYBER_FREQUENCY_HZ * total_duration_sec
        assert len(seen_stamps) >= expected / 3, (
            f"Expected ~{expected} distinct command stamps, saw {len(seen_stamps)} -- is the publisher ticking?"
        )
        assert len(seen_stamps) <= expected * 3, (
            f"Saw {len(seen_stamps)} distinct command stamps for expected ~{expected} -- "
            "publisher is ticking too often?"
        )


if __name__ == "__main__":
    run_manor_tests()
