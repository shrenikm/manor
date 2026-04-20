@0x8bcd32629790343e;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFPoseTrajectoryV1 {
    header             @0 :Header.VersionedTimestampHeader;
    times              @1 :Common.Float64Array;
    translationsArray  @2 :Common.Float64Array;
    orientationsArray  @3 :Common.Float64Array;
}

struct VersionedEEFPoseTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEFPoseTrajectoryV1;
    }
}
