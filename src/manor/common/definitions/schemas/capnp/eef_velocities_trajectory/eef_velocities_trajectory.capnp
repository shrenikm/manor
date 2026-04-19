@0xff0c9c784a72ec0e;

using V1 = import "eef_velocities_trajectory_v1.capnp";

struct VersionedEefVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :V1.EefVelocitiesTrajectoryV1;
    }
}
