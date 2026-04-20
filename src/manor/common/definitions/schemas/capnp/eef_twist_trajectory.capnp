@0x9249eeeb81572076;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFTwistTrajectoryV1 {
    header       @0 :Header.VersionedTimestampHeader;
    times        @1 :Common.Float64Array;
    linearArray  @2 :Common.Float64Array;
    angularArray @3 :Common.Float64Array;
}

struct VersionedEEFTwistTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :EEFTwistTrajectoryV1;
    }
}
