"""
Talos: the actuation sub-system.

Consumes Command messages and publishes joint + EE state. The sim/hardware
split is implemented as a ManipulatorBackend protocol: Talos itself is
mode-agnostic, and the backend either drives a Drake simulation or calls a
real manipulator SDK.
"""
