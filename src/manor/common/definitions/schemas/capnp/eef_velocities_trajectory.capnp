@0xff0c9c784a72ec0e;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EefVelocitiesTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    eefVelocitiesArray @2 :Common.Float64Array;
}

struct VersionedEefVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EefVelocitiesTrajectoryV1;
    }
}
