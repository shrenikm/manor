"""
Soma: the proprioception sub-system.

Consumes JointState and EEFState, performs forward kinematics on the joint
state to compute an EEF pose and twist, and publishes a combined
Proprioception message. Has no sim/hardware split -- the kinematic model is
the same in either mode.
"""
