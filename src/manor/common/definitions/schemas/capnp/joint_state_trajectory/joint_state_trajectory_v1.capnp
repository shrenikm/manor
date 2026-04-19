@0xbb04b1322e185b78;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct JointStateTrajectoryV1 {
    header               @0 :Header.TimestampHeaderV1;
    times                @1 :Common.Float64Array;
    jointPositionsArray  @2 :Common.Float64Array;
    jointVelocitiesArray @3 :Common.Float64Array;
}
