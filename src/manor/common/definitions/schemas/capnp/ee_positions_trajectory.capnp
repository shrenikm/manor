@0xbd9a3b7de48b956a;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEPositionsTrajectoryV1 {
    header            @0 :Header.VersionedTimestampHeader;
    times             @1 :Common.Float64Array;
    eePositionsArray @2 :Common.Float64Array;
}

struct VersionedEEPositionsTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEPositionsTrajectoryV1;
    }
}
