@0x9d41350d0ccfbc30;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEStateTrajectoryV1 {
    header             @0 :Header.VersionedTimestampHeader;
    times              @1 :Common.Float64Array;
    eePositionsArray  @2 :Common.Float64Array;
    eeVelocitiesArray @3 :Common.Float64Array;
}

struct VersionedEEStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEStateTrajectoryV1;
    }
}
