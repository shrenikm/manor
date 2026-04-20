@0x93842928d8f76085;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EEFPoseV1 {
    header      @0 :Header.TimestampHeaderV1;
    translation @1 :Common.Float64Array;
    orientation @2 :Common.Float64Array;
}

struct VersionedEEFPose {
    union {
        unset @0 :Void;
        v1    @1 :EEFPoseV1;
    }
}
