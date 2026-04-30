@0xff0c9c784a72ec0e;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEVelocitiesTrajectoryV1 {
    header             @0 :Header.VersionedTimestampHeader;
    times              @1 :Common.Float64Array;
    eeVelocitiesArray @2 :Common.Float64Array;
}

struct VersionedEEVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEVelocitiesTrajectoryV1;
    }
}
