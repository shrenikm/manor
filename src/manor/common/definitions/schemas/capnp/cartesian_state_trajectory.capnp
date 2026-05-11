@0xd73c731709639513;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct CartesianStateTrajectoryV1 {
    header             @0 :Header.VersionedTimestampHeader;
    times              @1 :Common.Float64Array;
    translationsArray  @2 :Common.Float64Array;
    orientationsArray  @3 :Common.Float64Array;
    linearArray        @4 :Common.Float64Array;
    angularArray       @5 :Common.Float64Array;
}

struct VersionedCartesianStateTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :CartesianStateTrajectoryV1;
    }
}
