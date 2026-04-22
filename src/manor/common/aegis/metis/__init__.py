"""
Metis: the policy sub-system.

Consumes an Observation assembled from proprioception + sensor messages,
feeds it through a Policy, and publishes Action messages. The Policy
protocol is the plug-in point for classical planners, trajectory
optimization, diffusion policies, VLAs, etc.
"""
