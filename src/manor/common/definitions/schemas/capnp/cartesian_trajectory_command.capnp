@0xfc00ed11ced85ce0;

using Header = import "/timestamp_header.capnp";
using CPT = import "/cartesian_pose_trajectory.capnp";
using CTT = import "/cartesian_twist_trajectory.capnp";

struct CartesianTrajectoryCommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        cartesianPoseTrajectory  @1 :CPT.VersionedCartesianPoseTrajectory;
        cartesianTwistTrajectory @2 :CTT.VersionedCartesianTwistTrajectory;
    }
}

struct VersionedCartesianTrajectoryCommand {
    union {
        unset @0 :Void;
        v1    @1 :CartesianTrajectoryCommandV1;
    }
}
