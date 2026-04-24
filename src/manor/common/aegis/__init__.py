"""
Aegis: the full manipulation system.

Aegis is composed of independent Drake sub-systems (Helios, Talos, Metis,
Kyber) that communicate via AbstractValue ports carrying typed messages. Each
sub-system runs at its own publish frequency, independent of input message
rates, so the same graph works in simulation or against hardware with only the
sensor / actuator leaves swapped.
"""
