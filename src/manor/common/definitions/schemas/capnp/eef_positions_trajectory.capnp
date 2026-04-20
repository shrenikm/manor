@0xbd9a3b7de48b956a;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFPositionsTrajectoryV1 {
    header            @0 :Header.TimestampHeaderV1;
    times             @1 :Common.Float64Array;
    eefPositionsArray @2 :Common.Float64Array;
}

struct VersionedEEFPositionsTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEFPositionsTrajectoryV1;
    }
}
