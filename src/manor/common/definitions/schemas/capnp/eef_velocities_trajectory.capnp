@0xff0c9c784a72ec0e;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFVelocitiesTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    eefVelocitiesArray @2 :Common.Float64Array;
}

struct VersionedEEFVelocitiesTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEFVelocitiesTrajectoryV1;
    }
}
