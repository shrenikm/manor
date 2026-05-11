@0xf6ba0d1f08d199ed;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct CartesianTwistV1 {
    header  @0 :Header.VersionedTimestampHeader;
    linear  @1 :Common.Float64Array;
    angular @2 :Common.Float64Array;
}

struct VersionedCartesianTwist {
    union {
        unset @0 :Void;
        v1    @1 :CartesianTwistV1;
    }
}
