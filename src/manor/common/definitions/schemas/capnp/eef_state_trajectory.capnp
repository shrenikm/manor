@0x9d41350d0ccfbc30;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EefStateTrajectoryV1 {
    header             @0 :Header.TimestampHeaderV1;
    times              @1 :Common.Float64Array;
    eefPositionsArray  @2 :Common.Float64Array;
    eefVelocitiesArray @3 :Common.Float64Array;
}

struct VersionedEefStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EefStateTrajectoryV1;
    }
}
