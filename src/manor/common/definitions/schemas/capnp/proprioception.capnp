@0xd9cdde76041c9a7f;

using Header = import "/timestamp_header.capnp";
using JS = import "/joint_state.capnp";
using ES = import "/eef_state.capnp";
using EP = import "/eef_pose.capnp";
using ET = import "/eef_twist.capnp";

struct ProprioceptionV1 {
    header     @0 :Header.VersionedTimestampHeader;
    jointState @1 :JS.VersionedJointState;

    eefState :union {
        none @2 :Void;
        some @3 :ES.VersionedEEFState;
    }
    eefPose :union {
        none @4 :Void;
        some @5 :EP.VersionedEEFPose;
    }
    eefTwist :union {
        none @6 :Void;
        some @7 :ET.VersionedEEFTwist;
    }
}

struct VersionedProprioception {
    union {
        unset @0 :Void;
        v1    @1 :ProprioceptionV1;
    }
}
