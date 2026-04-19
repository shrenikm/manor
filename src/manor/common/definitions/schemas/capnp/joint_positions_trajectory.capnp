@0x93b1fd518508cf90;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct JointPositionsTrajectoryV1 {
    header              @0 :Header.TimestampHeaderV1;
    times               @1 :Common.Float64Array;
    jointPositionsArray @2 :Common.Float64Array;
}

struct VersionedJointPositionsTrajectory {
    union {
        unset @0 :Void;
        v1    @1 :JointPositionsTrajectoryV1;
    }
}
