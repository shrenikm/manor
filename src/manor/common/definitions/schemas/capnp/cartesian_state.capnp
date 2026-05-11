@0x93ce511df3f6dd01;

using Header = import "/timestamp_header.capnp";
using CP = import "/cartesian_pose.capnp";
using CT = import "/cartesian_twist.capnp";

struct CartesianStateV1 {
    header         @0 :Header.VersionedTimestampHeader;
    cartesianPose  @1 :CP.VersionedCartesianPose;
    cartesianTwist @2 :CT.VersionedCartesianTwist;
}

struct VersionedCartesianState {
    union {
        unset @0 :Void;
        v1    @1 :CartesianStateV1;
    }
}
