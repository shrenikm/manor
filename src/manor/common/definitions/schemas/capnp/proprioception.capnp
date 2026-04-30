@0xd9cdde76041c9a7f;

using Header = import "/timestamp_header.capnp";
using JS = import "/joint_state.capnp";
using CS = import "/cartesian_state.capnp";
using ES = import "/ee_state.capnp";

struct ProprioceptionV1 {
    header     @0 :Header.VersionedTimestampHeader;
    jointState @1 :JS.VersionedJointState;

    cartesianState :union {
        none @2 :Void;
        some @3 :CS.VersionedCartesianState;
    }
    eeState :union {
        none @4 :Void;
        some @5 :ES.VersionedEEState;
    }
}

struct VersionedProprioception {
    union {
        unset @0 :Void;
        v1    @1 :ProprioceptionV1;
    }
}
