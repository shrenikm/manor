@0xc9e1e9ad93ec68aa;

using Common = import "/common.capnp";
using Header = import "/timestamp_header.capnp";

struct JointPositionsV1 {
    header    @0 :Header.VersionedTimestampHeader;
    positions @1 :Common.Float64Array;
}

struct VersionedJointPositions {
    union {
        unset @0 :Void;
        v1    @1 :JointPositionsV1;
    }
}
