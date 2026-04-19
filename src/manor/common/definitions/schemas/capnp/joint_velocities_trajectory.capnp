@0xe6d42be845d59003;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct JointVelocitiesTrajectoryV1 {
    header               @0 :Header.TimestampHeaderV1;
    times                @1 :Common.Float64Array;
    jointVelocitiesArray @2 :Common.Float64Array;
}

struct VersionedJointVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :JointVelocitiesTrajectoryV1;
    }
}
