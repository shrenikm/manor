@0xa7b47a4355e88d21;

using Header = import "/timestamp_header.capnp";
using CP = import "/cartesian_pose.capnp";
using CT = import "/cartesian_twist.capnp";

struct CartesianCommandV1 {
    header @0 :Header.VersionedTimestampHeader;

    union {
        cartesianPose  @1 :CP.VersionedCartesianPose;
        cartesianTwist @2 :CT.VersionedCartesianTwist;
    }
}

struct VersionedCartesianCommand {
    union {
        unset @0 :Void;
        v1    @1 :CartesianCommandV1;
    }
}
