@0x93842928d8f76085;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct CartesianPoseV1 {
    header      @0 :Header.VersionedTimestampHeader;
    translation @1 :Common.Float64Array;
    orientation @2 :Common.Float64Array;
}

struct VersionedCartesianPose {
    union {
        unset @0 :Void;
        v1    @1 :CartesianPoseV1;
    }
}
