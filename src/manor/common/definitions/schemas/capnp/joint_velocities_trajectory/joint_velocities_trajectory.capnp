@0xe6d42be845d59003;

using V1 = import "joint_velocities_trajectory_v1.capnp";

struct VersionedJointVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.JointVelocitiesTrajectoryV1;
    }
}
