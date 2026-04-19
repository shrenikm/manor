@0xf34ab3d541a3a030;

using Common = import "/common.capnp";
using Header = import "/timestamp_header/timestamp_header_v1.capnp";

struct JointVelocitiesTrajectoryV1 {
    header               @0 :Header.TimestampHeaderV1;
    times                @1 :Common.Float64Array;
    jointVelocitiesArray @2 :Common.Float64Array;
}
