@0xf6ba0d1f08d199ed;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct EefTwistV1 {
    header  @0 :Header.TimestampHeaderV1;
    linear  @1 :Common.Float64Array;
    angular @2 :Common.Float64Array;
}

struct VersionedEefTwist {
    union {
        unset @0 :Void;
        v1    @1 :EefTwistV1;
    }
}
