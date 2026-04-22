"""
Helios: the sensor-observation sub-system.

Helios has no input ports. It publishes one output port per sensor (RGB image,
depth image, ...). The sim/hardware split is implemented as a SensorBackend
protocol: Helios itself is mode-agnostic, and the backend either reads a
Drake simulation or a real sensor SDK.
"""
