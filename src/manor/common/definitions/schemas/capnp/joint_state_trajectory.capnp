@0xaad98fe4b2ed9326;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct JointStateTrajectoryV1 {
    header               @0 :Header.TimestampHeaderV1;
    times                @1 :Common.Float64Array;
    jointPositionsArray  @2 :Common.Float64Array;
    jointVelocitiesArray @3 :Common.Float64Array;
}

struct VersionedJointStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :JointStateTrajectoryV1;
    }
}
